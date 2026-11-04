#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionPlanRequest,
    PlanningOptions,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class MoveItXyzGoal(Node):
    ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5")

    def __init__(self):
        super().__init__("moveit_xyz_goal")
        self.declare_parameter("x", 0.30)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.20)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("tip_link", "tcp_link")
        self.declare_parameter("position_tolerance_m", 0.01)
        self.declare_parameter("planning_time_s", 5.0)
        self.declare_parameter("velocity_scale", 0.1)
        self.declare_parameter("acceleration_scale", 0.1)
        self.declare_parameter("execute", False)
        for joint_name in self.ARM_JOINTS:
            self.declare_parameter(f"constrain_{joint_name}", False)
            self.declare_parameter(f"{joint_name}_target", 0.0)
            self.declare_parameter(f"{joint_name}_tolerance", 0.1)

        self._client = ActionClient(self, MoveGroup, "/move_action")

    def _build_goal(self):
        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        z = float(self.get_parameter("z").value)
        frame_id = str(self.get_parameter("frame_id").value)
        tip_link = str(self.get_parameter("tip_link").value)
        position_tolerance = float(
            self.get_parameter("position_tolerance_m").value
        )
        planning_time = float(self.get_parameter("planning_time_s").value)
        velocity_scale = float(self.get_parameter("velocity_scale").value)
        acceleration_scale = float(self.get_parameter("acceleration_scale").value)

        if not all(math.isfinite(value) for value in (x, y, z)):
            raise ValueError("XYZ target must contain finite numbers")
        if position_tolerance <= 0.0 or planning_time <= 0.0:
            raise ValueError("position tolerance and planning time must be positive")
        if not 0.0 < velocity_scale <= 1.0 or not 0.0 < acceleration_scale <= 1.0:
            raise ValueError("velocity and acceleration scales must be in (0, 1]")

        goal_constraints = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame_id
        position_constraint.header.stamp = self.get_clock().now().to_msg()
        position_constraint.link_name = tip_link
        position_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0
        position_constraint.constraint_region.primitives.append(sphere)
        position_constraint.constraint_region.primitive_poses.append(target_pose)
        goal_constraints.position_constraints.append(position_constraint)

        for joint_name in self.ARM_JOINTS:
            if not self.get_parameter(f"constrain_{joint_name}").value:
                continue
            joint_target = float(self.get_parameter(f"{joint_name}_target").value)
            joint_tolerance = float(
                self.get_parameter(f"{joint_name}_tolerance").value
            )
            if not math.isfinite(joint_target):
                raise ValueError(f"{joint_name} target must be finite")
            if joint_tolerance <= 0.0:
                raise ValueError(f"{joint_name} tolerance must be positive")
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_target
            joint_constraint.tolerance_above = joint_tolerance
            joint_constraint.tolerance_below = joint_tolerance
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)

        request = MotionPlanRequest()
        request.group_name = "arm"
        request.pipeline_id = "ompl"
        request.planner_id = "RRTConnectkConfigDefault"
        request.num_planning_attempts = 5
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = velocity_scale
        request.max_acceleration_scaling_factor = acceleration_scale
        request.start_state.is_diff = True
        request.goal_constraints.append(goal_constraints)

        goal = MoveGroup.Goal()
        goal.request = request
        options = PlanningOptions()
        options.plan_only = not bool(self.get_parameter("execute").value)
        options.replan = False
        goal.planning_options = options
        return goal

    def run_goal(self):
        goal = self._build_goal()
        self.get_logger().info(
            "Waiting for MoveIt; planning XYZ target in base_link "
            "without an end-effector yaw constraint"
        )
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server /move_action is unavailable")
            return 1

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the planning request")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("MoveIt returned no result")
            return 1

        result = wrapped_result.result
        error_code = result.error_code.val
        point_count = len(result.planned_trajectory.joint_trajectory.points)
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"MoveIt failed with error code {error_code}")
            return 1

        mode = "execution" if bool(self.get_parameter("execute").value) else "plan-only"
        final_point = result.planned_trajectory.joint_trajectory.points[-1]
        joint_targets = dict(zip(
            result.planned_trajectory.joint_trajectory.joint_names,
            final_point.positions,
        ))
        joint_summary = ", ".join(
            f"{name}={joint_targets[name]:.3f} rad"
            for name in self.ARM_JOINTS
            if name in joint_targets
        )
        self.get_logger().info(
            f"MoveIt {mode} succeeded with {point_count} trajectory points; "
            f"final joints: {joint_summary}"
        )
        return 0


def main():
    rclpy.init()
    node = MoveItXyzGoal()
    try:
        return node.run_goal()
    except (ValueError, RuntimeError) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
