"""Run the real coordinator against a fake Action server, never a servo driver."""
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from robot_arm_interfaces.action import MoveJoints, TriggerStrike
from robot_arm_interfaces.msg import HarvestTarget, HarvestTargetArray, VisionStopState, HarvestResult
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml


SOURCE = Path(os.environ.get("MANGO_TEST_SOURCE_ROOT", Path(__file__).resolve().parents[2]))
LAUNCH_PATH = Path(os.environ.get(
    "MANGO_TEST_LAUNCH", SOURCE / "robot_arm_bringup/launch/dual_arm_harvest.launch.py"))
_spec = importlib.util.spec_from_file_location("dual_grasp_test_launch", LAUNCH_PATH)
launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(launch)


class GraspProfiles(unittest.TestCase):
    def setUp(self):
        os.environ["ROS_DOMAIN_ID"] = str(160 + os.getpid() % 30)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("grasp_profiles_test", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.goals = []
        self.motor_goals = []
        self.events = []
        self.goal_times = []
        self.holding = True
        self.fail_motor = False
        self.fail_motion = False
        self.state_names = None
        self.stop_id = 0
        self.stage_id = 0
        self.results = []
        self.delay_first_goal_s = 0.0
        self.delayed_motion_goal_number = 1
        self.cancelled = 0
        self.fail_home = False
        self.reject_home_goals = 0
        self.home_attempts = 0
        self.delay_home_goal_s = 0.0
        self.server = ActionServer(self.node, MoveJoints, "/move_joints", self.execute,
                                   goal_callback=self.accept_goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.result_sub = self.node.create_subscription(HarvestResult, '/harvest/result',
                                                        self.results.append, 10)
        self.motor_server = ActionServer(
            self.node, TriggerStrike, "/end_effector/trigger_strike", self.execute_motor)
        self.names = [side + "_" + joint for side in ("left", "right")
                      for joint in ("joint1", "joint2", "joint3", "joint4", "joint5", "gripper")]
        self.state_pub = self.node.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
        self.stop_pub = self.node.create_publisher(VisionStopState, "/base/vision_stop_state", 10)
        self.target_pub = self.node.create_publisher(HarvestTargetArray, "/vision/harvest_targets", 10)
        self.description_pub = self.node.create_publisher(
            String, "/robot_description", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        identity = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.]
        transforms = {side + "_camera_to_base": list(identity) for side in ("left", "right")}
        single = (SOURCE / "robot_arm_description/urdf/imported/arm_urdf_v1.urdf").read_text()
        self.description = launch._dual_arm_description(single, transforms)
        self.parameters = {
            "dry_run": False, "allow_motion": True, "stop_at_target": False,
            "left_transform_calibrated": True, "right_transform_calibrated": True,
            "left_ticks_calibrated": True, "right_ticks_calibrated": True,
            "driver_joint_names": self.names, "zero_ticks": [2000.] * 12,
            "urdf_zero_ticks": [2000.] * 12, "ticks_per_radian": [650.] * 12,
            "directions": [1.] * 12, "min_ticks": [-20000] * 12, "max_ticks": [20000] * 12,
            "left_ik_lower_limits": [-math.pi] * 5, "left_ik_upper_limits": [math.pi] * 5,
            "right_ik_lower_limits": [-math.pi] * 5, "right_ik_upper_limits": [math.pi] * 5,
            "gripper_duration_ms": 2000,
            "gripper_open_native_rad": 0., "open_delay_ms": 500,
            **transforms,
        }
        self.directory = tempfile.TemporaryDirectory()
        self.log = tempfile.TemporaryFile()
        self.process = None

    def accept_goal(self, request):
        if all(p == 0.0 for p in request.positions):
            self.home_attempts += 1
            if self.reject_home_goals:
                self.reject_home_goals -= 1
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def execute(self, handle):
        self.goals.append(handle.request)
        self.goal_times.append(time.monotonic())
        self.events.append("arm")
        delay = (self.delay_first_goal_s if len(self.goals) == self.delayed_motion_goal_number else
                 self.delay_home_goal_s if all(p == 0.0 for p in handle.request.positions) else 0.0)
        if delay:
            ready = Future()
            deadline = time.monotonic() + delay
            def tick():
                if not ready.done() and (handle.is_cancel_requested or time.monotonic() >= deadline):
                    ready.set_result(True)
            timer = self.node.create_timer(.01, tick)
            await ready
            self.node.destroy_timer(timer)
            if handle.is_cancel_requested:
                self.cancelled += 1
                handle.canceled()
                return MoveJoints.Result(success=False, message='fake move cancelled')
        result = MoveJoints.Result()
        failed = self.fail_motion or (self.fail_home and all(p == 0.0 for p in handle.request.positions))
        result.success = not failed
        result.message = "fake servos reached target"
        if failed:
            result.message = "fake motion failure"
            handle.abort()
        else:
            handle.succeed()
        return result

    def execute_motor(self, handle):
        self.motor_goals.append(handle.request)
        self.events.append("motor_started")
        result = TriggerStrike.Result()
        result.success = not self.fail_motor
        result.message = "fake serial delay elapsed" if result.success else "fake serial failure"
        self.events.append("motor_stopped")
        if result.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    def start(self, **overrides):
        self.parameters.update(overrides)
        path = Path(self.directory.name) / "params.yaml"
        path.write_text(yaml.safe_dump({"dual_arm_harvest_node": {"ros__parameters": self.parameters}}))
        binary = os.environ.get("MANGO_TEST_COORDINATOR") or str(
            Path(get_package_prefix("robot_arm_node")) / "lib/robot_arm_node/dual_arm_harvest_node")
        self.process = subprocess.Popen(
            [binary, "--ros-args", "--params-file", str(path)], stdout=self.log, stderr=self.log)
        self.until(lambda: self.target_pub.get_subscription_count() > 0)
        self.pump(0.5)

    def pump(self, seconds=0.03):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            state = JointState()
            state.header.stamp = self.node.get_clock().now().to_msg()
            state.name = self.state_names if self.state_names is not None else self.names
            state.position = [0.] * len(state.name)
            self.state_pub.publish(state)
            stop = VisionStopState()
            stop.header.stamp = state.header.stamp
            stop.state = VisionStopState.HOLDING if self.holding else VisionStopState.IDLE
            stop.stage_id = self.stage_id
            stop.stop_id = self.stop_id
            self.stop_pub.publish(stop)
            self.description_pub.publish(String(data=self.description))
            self.executor.spin_once(timeout_sec=0.01)

    def until(self, condition, timeout=8.):
        deadline = time.monotonic() + timeout
        while not condition():
            self.pump()
            if self.process is not None and self.process.poll() is not None:
                self.fail("coordinator exited:\n" + self.logs())
            if time.monotonic() >= deadline:
                self.fail("coordinator timeout:\n" + self.logs())

    def logs(self):
        self.log.seek(0)
        return self.log.read().decode(errors="replace")

    def send(self, targets):
        msg = HarvestTargetArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.stage_id = self.stage_id
        msg.stop_id = self.stop_id
        msg.valid = True
        msg.targets = targets
        self.target_pub.publish(msg)
        return msg

    @staticmethod
    def target(side, small, color):
        target = HarvestTarget()
        target.slot = side + "_bottom"
        target.is_small = small
        target.color = color
        target.surface_point.x = .35
        target.surface_point.z = .25
        target.confidence = .95
        return target

    @staticmethod
    def positions(goal):
        return dict(zip(goal.joint_names, goal.positions))

    def test_mixed_sizes_on_both_arms_and_switching_on_next_round(self):
        self.start(approach_complete_when_stationary=True,
                   left_small_offset_m=[.001, .002, .003],
                   left_large_offset_m=[.004, .005, .006],
                   right_small_offset_m=[-.001, -.002, -.003],
                   right_large_offset_m=[-.004, -.005, -.006])
        self.send([self.target("left", True, "yellow_mango"),
                   self.target("right", False, "green_mango")])
        self.until(lambda: len(self.goals) >= 3)
        move, close, home = self.goals[:3]
        self.assertTrue(move.complete_when_stationary)
        self.assertFalse(close.complete_when_stationary)
        self.assertFalse(home.complete_when_stationary)
        self.assertEqual(self.positions(move)["left_gripper"], -1.)
        self.assertEqual(self.positions(move)["right_gripper"], 0.)
        self.assertEqual(self.positions(close), {"left_gripper": -2., "right_gripper": -1.})
        for index, name in enumerate(move.joint_names):
            if name.endswith("gripper"):
                self.assertEqual(move.durations_ms[index], 2000)
        for index, name in enumerate(home.joint_names):
            self.assertEqual(home.positions[index], 0.)
            self.assertEqual(home.start_delays_ms[index], 500 if name.endswith("gripper") else 0)
        self.pump(.15)
        # Color must not override is_small; both sides must reselect for each fruit.
        self.send([self.target("left", False, "yellow_mango"),
                   self.target("right", True, "green_mango")])
        self.until(lambda: len(self.goals) >= 6)
        self.assertEqual(self.positions(self.goals[3])["left_gripper"], 0.)
        self.assertEqual(self.positions(self.goals[3])["right_gripper"], -1.)
        self.assertEqual(self.positions(self.goals[4]), {"left_gripper": -1., "right_gripper": -2.})
        logs = self.logs()
        for key, offset in (("left_small", "0.001 0.002 0.003"),
                            ("left_large", "0.004 0.005 0.006"),
                            ("right_small", "-0.001 -0.002 -0.003"),
                            ("right_large", "-0.004 -0.005 -0.006")):
            self.assertIn("Grasp profile=" + key, logs)
            self.assertIn("offset_base_m=[" + offset + "]", logs)
        self.assertIn("base=(0.351, 0.002, 0.253)", logs)
        self.assertIn("base=(0.346, -0.005, 0.244)", logs)
        # Different base-frame offsets must affect IK angles, not only diagnostics.
        self.assertNotEqual(self.positions(move)["left_joint1"],
                            self.positions(self.goals[3])["left_joint1"])

    def test_legacy_fixed_angles_can_be_selected_explicitly(self):
        self.start(use_size_profiles=False, gripper_pregrasp_native_rad=-.4,
                   gripper_close_native_rad=-1.4)
        self.send([self.target("left", True, "green_mango")])
        self.until(lambda: len(self.goals) >= 3)
        self.assertEqual(self.positions(self.goals[0])["left_gripper"], -.4)
        self.assertEqual(self.positions(self.goals[1])["left_gripper"], -1.4)

    def check_separate_motion_profiles(self, use_motor):
        top_speeds = [61, 310, 510, 520, 21]
        bottom_speeds = [42, 220, 330, 440, 22]
        home_speeds = [31, 110, 210, 310, 23]
        self.start(use_motor=use_motor, skip_strike=True, motor_return_delay_s=0.0,
                   top_arm_speeds=top_speeds, bottom_arm_speeds=bottom_speeds,
                   home_arm_speeds=home_speeds, top_move_duration_ms=7000,
                   bottom_move_duration_ms=3000, home_move_duration_ms=1000)
        count = 2 if use_motor else 3
        for index, top_side in enumerate(('left', 'right')):
            # Swap height between arms, independently of fruit size and color.
            left = self.target('left', False, 'yellow_mango')
            right = self.target('right', True, 'green_mango')
            if top_side == 'left':
                left.slot = 'left_top'
            else:
                right.slot = 'right_top'
            self.send([left, right])
            self.until(lambda: len(self.goals) >= (index + 1) * count)
            move, home = self.goals[index * count], self.goals[(index + 1) * count - 1]
            for side in ('left', 'right'):
                for joint in range(1, 6):
                    position = move.joint_names.index(f'{side}_joint{joint}')
                    expected = top_speeds if side == top_side else bottom_speeds
                    self.assertEqual(move.speeds[position], expected[joint - 1])
                    self.assertEqual(move.durations_ms[position], 7000 if side == top_side else 3000)
                    home_index = home.joint_names.index(f'{side}_joint{joint}')
                    self.assertEqual(home.speeds[home_index], home_speeds[joint - 1])
                    self.assertEqual(home.durations_ms[home_index], 1000)
            self.pump(.1)

    def test_top_bottom_home_speeds_are_separate_in_motor_mode(self):
        self.check_separate_motion_profiles(True)

    def test_top_bottom_home_speeds_are_separate_in_gripper_mode(self):
        self.check_separate_motion_profiles(False)

    def test_motor_mode_both_arms_only_move_and_return_without_gripper_feedback(self):
        self.state_names = [name for name in self.names if not name.endswith("gripper")]
        self.start(use_motor=True, motor_return_delay_s=1.0,
                   left_small_offset_m=[.001, .002, .003], right_large_offset_m=[-.004, -.005, -.006])
        self.send([self.target("left", True, "yellow_mango"),
                   self.target("right", False, "green_mango")])
        self.until(lambda: len(self.goals) >= 2)
        self.pump(.2)
        self.assertEqual(len(self.goals), 2)
        move, home = self.goals
        self.assertEqual(set(move.joint_names), set(self.state_names))
        self.assertEqual(set(home.joint_names), set(self.state_names))
        self.assertTrue(all(value == 0.0 for value in home.positions))
        self.assertTrue(all(delay == 0 for delay in home.start_delays_ms))
        self.assertEqual(len(self.motor_goals), 1)
        self.assertEqual(set(self.motor_goals[0].arms), {"left", "right"})
        self.assertEqual(self.motor_goals[0].wait_s, 1.0)
        self.assertEqual(self.events, ["arm", "motor_started", "motor_stopped", "arm"])
        self.assertIn("local strike delay elapsed; MCU completion unverified", self.logs())
        self.assertIn("base=(0.351, 0.002, 0.253)", self.logs())
        self.assertIn("base=(0.346, -0.005, 0.244)", self.logs())

    def run_startup_home(self):
        binary = Path(get_package_prefix('robot_arm_bringup')) / 'lib/robot_arm_bringup/startup_home.py'
        self.startup_log = tempfile.TemporaryFile()
        process = subprocess.Popen([str(binary)], stdout=self.startup_log, stderr=self.startup_log)
        def cleanup():
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            self.startup_log.close()
        self.addCleanup(cleanup)
        self.until(lambda: process.poll() is not None)
        self.startup_log.seek(0)
        output = self.startup_log.read().decode()
        return process.returncode, output

    def test_startup_home_uses_ten_joints_and_effective_return_profile(self):
        self.holding = False  # No chassis or parking permission is needed at startup.
        self.start(use_motor=True, home_arm_speeds=[20, 700, 1000, 700, 20],
                   home_move_duration_ms=2000)
        code, log = self.run_startup_home()
        self.assertEqual(code, 0, log)
        self.assertEqual(len(self.goals), 1)
        goal = self.goals[0]
        self.assertEqual(list(goal.speeds), [20, 700, 1000, 700, 20] * 2)
        self.assertEqual(list(goal.durations_ms), [2000] * 10)
        self.assertTrue(all(position == 0.0 for position in goal.positions))
        self.assertFalse(goal.complete_when_stationary)
        self.assertEqual(self.motor_goals, [])
        self.assertIn('Startup home confirmed', log)

    def test_startup_home_preserves_legacy_gripper_release(self):
        self.holding = False
        self.start(use_motor=False, gripper_open_native_rad=-0.1)
        code, log = self.run_startup_home()
        self.assertEqual(code, 0, log)
        goal, = self.goals
        self.assertEqual(len(goal.joint_names), 12)
        for index, name in enumerate(goal.joint_names):
            self.assertEqual(goal.positions[index], -0.1 if name.endswith('gripper') else 0.0)
            self.assertEqual(goal.start_delays_ms[index], 500 if name.endswith('gripper') else 0)
        self.assertEqual(self.motor_goals, [])

    def test_failed_startup_home_reports_failure_without_strike(self):
        self.holding = False
        self.fail_home = True
        self.start(use_motor=True)
        code, log = self.run_startup_home()
        self.assertNotEqual(code, 0)
        self.assertIn('startup home failed', log)
        self.assertEqual(self.motor_goals, [])

    def test_dry_run_startup_does_not_send_motion(self):
        self.start(use_motor=True, dry_run=True, allow_motion=False)
        code, log = self.run_startup_home()
        self.assertEqual(code, 0, log)
        self.assertEqual(self.goals, [])
        self.assertIn('Startup home skipped', log)

    def test_serial_strike_defaults_to_one_second_wait(self):
        self.start(use_motor=True)
        self.send([self.target("left", True, "yellow_mango")])
        self.until(lambda: len(self.goals) >= 2)
        self.pump(.1)
        self.assertEqual(len(self.motor_goals), 1)
        self.assertTrue(all(not goal.complete_when_stationary for goal in self.goals))
        self.assertEqual(self.motor_goals[0].wait_s, 1.0)
        self.assertIn("local return delay=1.000 s; no MCU feedback", self.logs())

    def test_serial_strike_can_override_local_return_delay(self):
        self.start(use_motor=True, motor_return_delay_s=2.5)
        self.send([self.target("left", False, "green_mango"), self.target("right", True, "yellow_mango")])
        self.until(lambda: len(self.goals) >= 2)
        request = self.motor_goals[0]
        self.assertEqual(request.wait_s, 2.5)
        self.assertEqual(self.events, ["arm", "motor_started", "motor_stopped", "arm"])

    def test_stop_at_target_does_not_require_serial_server(self):
        self.motor_server.destroy()
        self.motor_server = None
        self.start(use_motor=True, stop_at_target=True)
        self.send([self.target("left", False, "green_mango")])
        self.until(lambda: len(self.goals) >= 1)
        self.pump(.15)
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(self.motor_goals, [])

    def test_skip_strike_waits_and_returns_both_arms_without_serial_server(self):
        self.motor_server.destroy()
        self.motor_server = None
        self.state_names = [name for name in self.names if not name.endswith("gripper")]
        self.start(use_motor=True, skip_strike=True, motor_return_delay_s=.35,
                   left_small_offset_m=[.001, .002, .003],
                   right_large_offset_m=[-.004, -.005, -.006])
        self.send([self.target("left", True, "yellow_mango"),
                   self.target("right", False, "green_mango")])
        self.until(lambda: len(self.goals) == 2)
        self.pump(.1)
        move, home = self.goals
        self.assertEqual(set(move.joint_names), set(self.state_names))
        self.assertEqual(set(home.joint_names), set(self.state_names))
        self.assertTrue(all(value == 0.0 for value in home.positions))
        self.assertGreaterEqual(self.goal_times[1] - self.goal_times[0], .35)
        self.assertEqual(self.motor_goals, [])
        self.assertEqual(self.events, ["arm", "arm"])
        self.assertIn("Strike skipped: no serial trigger", self.logs())
        self.assertIn("base=(0.351, 0.002, 0.253)", self.logs())
        self.assertIn("base=(0.346, -0.005, 0.244)", self.logs())

    def test_skip_strike_stops_if_parking_permission_lost_during_wait(self):
        self.start(use_motor=True, skip_strike=True, motor_return_delay_s=1.0)
        self.send([self.target("left", True, "yellow_mango")])
        self.until(lambda: "Strike skipped: no serial trigger" in self.logs())
        self.holding = False
        self.until(lambda: "HOLDING lost during local wait" in self.logs())
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(self.motor_goals, [])

    def test_skip_strike_does_not_override_target_only_mode(self):
        self.start(use_motor=True, skip_strike=True, stop_at_target=True)
        self.send([self.target("right", False, "green_mango")])
        self.until(lambda: len(self.goals) == 1)
        self.pump(.2)
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(self.motor_goals, [])

    def test_skip_strike_has_no_effect_on_legacy_grippers(self):
        self.start(use_motor=False, skip_strike=True)
        self.send([self.target("left", True, "yellow_mango")])
        self.until(lambda: len(self.goals) == 3)
        self.assertEqual(self.positions(self.goals[1]), {"left_gripper": -2.0})
        self.assertEqual(self.motor_goals, [])

    def test_motor_mode_repeats_for_two_targets_on_one_side(self):
        self.start(use_motor=True, approach_complete_when_stationary=True)
        bottom = self.target("right", False, "green_mango")
        top = self.target("right", True, "yellow_mango")
        top.slot = "right_top"
        self.send([top, bottom])
        self.until(lambda: len(self.goals) >= 4)
        self.pump(.15)
        self.assertEqual(len(self.goals), 4)
        self.assertEqual([list(goal.arms) for goal in self.motor_goals], [["right"], ["right"]])
        for index, goal in enumerate(self.goals):
            self.assertEqual(set(goal.joint_names), {"right_joint" + str(i) for i in range(1, 6)})
            self.assertEqual(goal.complete_when_stationary, index % 2 == 0)
            if index % 2 == 1:
                self.assertTrue(all(value == 0.0 for value in goal.positions))

    def test_stop_at_target_overrides_motor_return(self):
        self.start(use_motor=True, stop_at_target=True)
        self.send([self.target("left", True, "yellow_mango")])
        self.until(lambda: len(self.goals) >= 1)
        self.pump(.2)
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(len(self.goals[0].joint_names), 5)
        self.assertEqual(self.motor_goals, [])

    def test_motor_mode_does_not_return_after_failed_move(self):
        self.fail_motion = True
        self.start(use_motor=True)
        self.send([self.target("left", True, "yellow_mango")])
        self.until(lambda: "fake motion failure" in self.logs())
        self.pump(.1)
        self.assertEqual(len(self.goals), 1)

    def test_motor_mode_dry_run_sends_no_motion(self):
        self.start(use_motor=True, dry_run=True)
        self.send([self.target("left", True, "yellow_mango")])
        self.until(lambda: "DRY RUN round" in self.logs())
        self.assertEqual(self.goals, [])
        self.assertEqual(self.motor_goals, [])

    def test_motor_failure_holds_arm_at_target(self):
        self.fail_motor = True
        self.start(use_motor=True, motor_return_delay_s=2.0)
        self.send([self.target("left", False, "green_mango")])
        self.until(lambda: "fake serial failure" in self.logs())
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(len(self.motor_goals), 1)
        self.assertEqual(self.motor_goals[0].wait_s, 2.0)

    def test_motor_server_required_before_moving(self):
        self.motor_server.destroy()
        self.motor_server = None
        self.start(use_motor=True)
        self.send([self.target("left", False, "green_mango")])
        self.until(lambda: "STM32 strike server unavailable" in self.logs())
        self.assertEqual(self.goals, [])

    def test_result_only_after_all_rounds_home_and_no_duplicate_batch(self):
        self.stop_id = 123
        self.start(use_motor=True, skip_strike=True, motor_return_delay_s=0.0)
        bottom = self.target('left', True, 'yellow_mango')
        top = self.target('left', False, 'green_mango')
        top.slot = 'left_top'
        self.send([bottom, top])
        self.until(lambda: len(self.results) == 1)
        self.assertEqual(len(self.goals), 4)
        self.assertTrue(self.results[0].success and self.results[0].returned_home)
        self.assertEqual(self.results[0].stop_id, 123)
        self.send([bottom])  # New snapshot stamp, same parking session.
        self.pump(.2)
        self.assertEqual(len(self.goals), 4)

    def test_timeout_cancels_move_then_home_then_result(self):
        self.stop_id = 124
        self.delay_first_goal_s = 5.0
        self.start(use_motor=True, skip_strike=True, harvest_timeout_s=1.0,
                   home_arm_speeds=[31, 110, 210, 310, 23], home_move_duration_ms=900)
        self.send([self.target('left', True, 'yellow_mango')])
        self.until(lambda: bool(self.results))
        self.assertEqual(self.cancelled, 1)
        self.assertEqual(len(self.goals), 2)
        self.assertTrue(self.goals[0].hold_on_cancel)
        self.assertFalse(self.goals[1].hold_on_cancel)
        self.assertEqual(len(self.goals[-1].joint_names), 10)
        self.assertEqual(list(self.goals[-1].speeds), [31, 110, 210, 310, 23] * 2)
        self.assertEqual(list(self.goals[-1].durations_ms), [900] * 10)
        self.assertTrue(all(p == 0.0 for p in self.goals[-1].positions))
        self.assertTrue(self.results[0].returned_home)
        self.assertIn('timed out', self.results[0].message)

    def test_timeout_without_vision_snapshot_still_returns_home(self):
        self.stop_id = 125
        self.start(use_motor=True, skip_strike=True, harvest_timeout_s=.6)
        self.until(lambda: bool(self.results))
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(len(self.goals[0].joint_names), 10)
        self.assertTrue(self.results[0].returned_home)

    def test_second_fruit_timeout_cancels_then_returns_both_arms_home(self):
        self.stop_id, self.stage_id = 129, 12
        self.delay_first_goal_s, self.delayed_motion_goal_number = 5.0, 3
        self.start(use_motor=True, skip_strike=True, motor_return_delay_s=0.0,
                   harvest_timeout_s=1.2)
        bottom = self.target('right', False, 'green_mango')
        top = self.target('right', True, 'yellow_mango')
        top.slot = 'right_top'
        self.send([top, bottom])
        self.until(lambda: bool(self.results))
        self.assertEqual(self.cancelled, 1)
        self.assertEqual(len(self.goals), 4)
        self.assertEqual(len(self.goals[1].joint_names), 5)
        self.assertEqual(len(self.goals[-1].joint_names), 10)
        self.assertTrue(all(p == 0.0 for p in self.goals[-1].positions))
        self.assertTrue(self.results[-1].returned_home)
        self.assertIn('timed out', self.results[-1].message)

    def test_failed_home_never_releases_chassis(self):
        self.stop_id = 126
        self.fail_home = True
        self.start(use_motor=True, skip_strike=True, motor_return_delay_s=0.0)
        self.send([self.target('left', True, 'yellow_mango')])
        self.until(lambda: bool(self.results))
        self.assertFalse(self.results[0].success)
        self.assertFalse(self.results[0].returned_home)

    def test_rejected_dual_arm_home_is_retried_before_publishing_result(self):
        self.stop_id, self.stage_id = 200, 12
        self.reject_home_goals = 2
        self.start(use_motor=True, skip_strike=True, motor_return_delay_s=0.0)
        self.send([self.target('left', False, 'green_mango'),
                   self.target('right', True, 'yellow_mango')])
        self.until(lambda: bool(self.results))
        self.assertEqual(self.home_attempts, 3)
        self.assertEqual(len(self.goals), 2)
        self.assertEqual(len(self.goals[-1].joint_names), 10)
        self.assertTrue(self.results[-1].returned_home)

    def test_persistent_home_rejection_finishes_recovery_without_false_resume(self):
        self.stop_id, self.stage_id = 201, 6
        self.reject_home_goals = 100
        self.start(use_motor=True, skip_strike=True, motor_return_delay_s=0.0)
        self.send([self.target('left', False, 'green_mango')])
        self.until(lambda: bool(self.results))
        self.assertEqual(self.home_attempts, 6)  # Normal return and bounded recovery.
        self.assertFalse(self.results[-1].success or self.results[-1].returned_home)
        self.assertEqual(len(self.goals), 1)

    def test_return_in_progress_is_not_cancelled_when_picking_budget_expires(self):
        self.stop_id, self.stage_id = 202, 8
        self.delay_home_goal_s = 1.1
        self.start(use_motor=True, skip_strike=True, motor_return_delay_s=0.0,
                   harvest_timeout_s=1.0)
        self.send([self.target('left', False, 'green_mango'),
                   self.target('right', True, 'yellow_mango')])
        self.until(lambda: bool(self.results))
        self.assertEqual(self.cancelled, 0)
        self.assertEqual(len(self.goals), 2)
        self.assertFalse(self.goals[-1].hold_on_cancel)
        self.assertTrue(self.results[-1].returned_home)

    def test_two_parking_sessions_per_lane_on_every_harvesting_stage(self):
        self.start(use_motor=True, skip_strike=True, motor_return_delay_s=0.0)
        expected = 0
        for stage in (4, 6, 8, 10, 12, 14):
            for case, sides in enumerate((('left',), ('right',), ('left', 'right'))):
                for tree in (1, 2):
                    with self.subTest(stage=stage, sides=sides, tree=tree):
                        self.stage_id, self.stop_id = stage, stage * 100 + case * 10 + tree
                        self.pump(.1)
                        self.send([self.target(side, side == 'right',
                                               'yellow_mango' if side == 'right' else 'green_mango')
                                   for side in sides])
                        expected += 1
                        self.until(lambda: len(self.results) == expected)
                        self.assertEqual(self.results[-1].stop_id, self.stop_id)
                        self.assertTrue(self.results[-1].returned_home)
                        self.assertEqual(len(self.goals), expected * 2)
                        self.assertEqual(len(self.goals[-1].joint_names), 5 * len(sides))

    def test_unreachable_fruit_is_skipped_after_confirmed_home(self):
        self.stop_id = 127
        self.start(use_motor=True, skip_strike=True)
        target = self.target('left', True, 'yellow_mango')
        target.surface_point.x = 100.0
        self.send([target])
        self.until(lambda: bool(self.results))
        self.assertEqual(len(self.goals), 1)
        self.assertTrue(all(p == 0.0 for p in self.goals[0].positions))
        self.assertTrue(self.results[0].returned_home)

    def test_target_only_and_dry_run_never_publish_resume(self):
        self.stop_id = 128
        self.start(use_motor=True, stop_at_target=True, harvest_timeout_s=.5)
        self.send([self.target('left', True, 'yellow_mango')])
        self.until(lambda: len(self.goals) == 1)
        self.pump(.7)
        self.assertEqual(self.results, [])

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=4)
        self.server.destroy()
        if self.motor_server is not None:
            self.motor_server.destroy()
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.log.close()
        self.directory.cleanup()


if __name__ == "__main__":
    unittest.main()
