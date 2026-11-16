#!/usr/bin/env python3
"""Home both arms before the unified launch starts the chassis."""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_arm_interfaces.action import MoveJoints


PARAMETERS = (
    "dry_run", "allow_motion", "left_ticks_calibrated", "right_ticks_calibrated",
    "use_motor", "home_arm_speeds", "home_move_duration_ms", "home_timeout_ms",
    "gripper_speed", "gripper_duration_ms", "gripper_open_native_rad", "open_delay_ms",
)


def make_home_goal(params):
    """Use the coordinator's effective home settings, including launch overrides."""
    speeds = params["home_arm_speeds"]
    if len(speeds) != 5 or any(speed < 0 or speed > 3400 for speed in speeds):
        raise ValueError("home_arm_speeds needs five values in 0..3400")
    duration = params["home_move_duration_ms"]
    timeout = params["home_timeout_ms"]
    if not 20 <= duration <= timeout <= 30000:
        raise ValueError("invalid home travel duration or timeout")
    goal = MoveJoints.Goal()
    goal.wait_for_completion = True
    # Leave optional completion overrides at their defaults: startup home uses
    # strict target tolerance, including on drivers predating stationary fallback.
    for side in ("left", "right"):
        for index in range(5):
            goal.joint_names.append(f"{side}_joint{index + 1}")
            goal.positions.append(0.0)
            goal.speeds.append(speeds[index])
            goal.start_delays_ms.append(0)
            goal.durations_ms.append(duration)
        if not params["use_motor"]:
            goal.joint_names.append(f"{side}_gripper")
            goal.positions.append(params["gripper_open_native_rad"])
            goal.speeds.append(params["gripper_speed"])
            goal.start_delays_ms.append(params["open_delay_ms"])
            goal.durations_ms.append(params["gripper_duration_ms"])
    goal.duration.sec, remainder = divmod(timeout, 1000)
    goal.duration.nanosec = remainder * 1000000
    return goal


class StartupHome(Node):
    def __init__(self):
        super().__init__("startup_arm_home")
        timeout = self.declare_parameter("startup_timeout_s", 60.0).value
        if not math.isfinite(timeout) or not 1.0 <= timeout <= 120.0:
            raise ValueError("startup_timeout_s must be in [1, 120]")
        self.deadline = time.monotonic() + timeout
        self.parameters_client = self.create_client(
            GetParameters, "/dual_arm_harvest_node/get_parameters")
        self.motion = ActionClient(self, MoveJoints, "/move_joints")

    def wait_result(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(0.0, timeout))
        if not future.done():
            raise RuntimeError("startup home communication timed out")
        return future.result()

    def read_profile(self):
        while rclpy.ok() and time.monotonic() < self.deadline:
            if self.parameters_client.wait_for_service(timeout_sec=0.2):
                break
        else:
            raise RuntimeError("harvest coordinator parameters unavailable")
        request = GetParameters.Request(names=list(PARAMETERS))
        response = self.wait_result(
            self.parameters_client.call_async(request), self.deadline - time.monotonic())
        if len(response.values) != len(PARAMETERS):
            raise RuntimeError("incomplete home profile response")
        fields = {
            ParameterType.PARAMETER_BOOL: "bool_value",
            ParameterType.PARAMETER_INTEGER: "integer_value",
            ParameterType.PARAMETER_DOUBLE: "double_value",
            ParameterType.PARAMETER_INTEGER_ARRAY: "integer_array_value",
        }
        params = {}
        for name, value in zip(PARAMETERS, response.values):
            if value.type not in fields:
                raise RuntimeError(f"missing or invalid coordinator parameter: {name}")
            params[name] = getattr(value, fields[value.type])
        return params

    def run(self):
        params = self.read_profile()
        if params["dry_run"] or not params["allow_motion"]:
            self.get_logger().info("Startup home skipped: dry_run or motion disabled")
            return
        if not params["left_ticks_calibrated"] or not params["right_ticks_calibrated"]:
            raise RuntimeError("startup home requires calibrated ticks for both arms")
        goal = make_home_goal(params)
        while rclpy.ok() and time.monotonic() < self.deadline:
            if self.motion.wait_for_server(timeout_sec=0.2):
                break
        else:
            raise RuntimeError("/move_joints unavailable for startup home")
        self.get_logger().info(
            f"Startup home: {len(goal.joint_names)} joints, speeds={list(params['home_arm_speeds'])}, "
            f"V3 duration={params['home_move_duration_ms']} ms; chassis waits")
        handle = self.wait_result(self.motion.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError("startup home rejected; check servo feedback and calibration")
        future = handle.get_result_async()
        try:
            result = self.wait_result(future, params["home_timeout_ms"] / 1000.0 + 5.0)
        except RuntimeError:
            self.wait_result(handle.cancel_goal_async(), 2.0)
            raise
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
            raise RuntimeError("startup home failed: " + result.result.message)
        self.get_logger().info("Startup home confirmed: both arms at native zero; chassis may start")


def main():
    rclpy.init()
    node = None
    code = 1
    try:
        node = StartupHome()
        node.run()
        code = 0
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(f"Startup home stopped; chassis will not start: {error}")
        else:
            print(f"Startup home initialization failed: {error}", flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
