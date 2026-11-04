"""Exercise the real chassis nodes with a PTY and simulated sensors, never hardware."""

import itertools
import os
import pty
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from robot_arm_interfaces.action import FollowRow, ShiftLane
from robot_arm_interfaces.msg import (RouteCommand, StageFeedback, UltrasonicArray,
                                      VisionRowObservation, VisionStopState)


_domains = itertools.count(80 + os.getpid() % 100)


class ChassisHarness:
    def __init__(self):
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('photoelectric_test', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.processes = []
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.buffer = b''
        self.lines = []
        self.commands = []
        self.feedback = []
        self.route_pub = self.node.create_publisher(RouteCommand, '/base/route_command', 10)
        self.twist_pub = self.node.create_publisher(Twist, '/base/cmd_vel', 10)
        self.node.create_subscription(RouteCommand, '/base/route_command', self.commands.append, 10)
        self.node.create_subscription(StageFeedback, '/base/stage_feedback', self.feedback.append, 10)
        self.ultra_pub = self.node.create_publisher(
            UltrasonicArray, '/base/ultrasonic', qos_profile_sensor_data)
        self.odom_pub = self.node.create_publisher(Odometry, '/base/odom', qos_profile_sensor_data)
        self.sensor_timer = None
        self.front_m = 2.0
        self.odom_enabled = True
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.vision = None
        self.vision_states = []
        self.vision_pub = self.node.create_publisher(
            VisionRowObservation, '/vision/row_observation', qos_profile_sensor_data)
        self.node.create_subscription(VisionStopState, '/base/vision_stop_state',
                                      self.vision_states.append, 10)
        self.odom_x = 0.0

    def start(self, executable, **parameters):
        binary = os.path.join(get_package_prefix('robot_chassis'), 'lib', 'robot_chassis', executable)
        args = [binary, '--ros-args']
        for name, value in parameters.items():
            value = str(value).lower() if isinstance(value, bool) else str(value)
            args.extend(['-p', f'{name}:={value}'])
        log = tempfile.TemporaryFile()
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        self.processes.append((process, log))
        return process

    def start_bridge(self, staged):
        self.start('base_serial_node', serial_device=os.ttyname(self.slave), dry_run=False,
                   allow_motion=True, stage_protocol_enabled=staged, command_timeout_s=0.3)
        topic = '/base/route_command' if staged else '/base/cmd_vel'
        # Only this process is attached to the pseudo-terminal, regardless of real YAML settings.
        self.until(lambda: self.node.count_subscribers(topic) >= (2 if staged else 1))
        self.pump(0.1)
        self.lines.clear()

    def start_navigation(self, **parameters):
        self.sensor_timer = self.node.create_timer(0.02, self.publish_sensors)
        self.start('row_navigation_node', stop_settle_s=0.08,
                   photoelectric_stop_timeout_s=0.8, ultrasonic_filter_window=1,
                   **parameters)
        self.follow = ActionClient(self.node, FollowRow, '/follow_row')
        self.shift = ActionClient(self.node, ShiftLane, '/shift_lane')
        self.until(lambda: self.follow.server_is_ready() and self.shift.server_is_ready())
        self.pump(0.2)

    def publish_sensors(self):
        msg = UltrasonicArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.front_m = self.front_m
        msg.rear_m = msg.left_m = msg.right_m = 2.0
        msg.front_valid = msg.rear_valid = msg.left_valid = msg.right_valid = True
        self.ultra_pub.publish(msg)
        if self.odom_enabled:
            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.pose.pose.orientation.w = 1.0
            odom.pose.pose.position.x = self.odom_x
            odom.twist.twist.linear.x = self.odom_vx
            odom.twist.twist.linear.y = self.odom_vy
            self.odom_pub.publish(odom)
        if self.vision is not None:
            self.vision.header.stamp = msg.header.stamp
            self.vision.frame_sequence += 1
            self.vision_pub.publish(self.vision)

    def read_serial(self):
        while True:
            try:
                data = os.read(self.master, 4096)
            except BlockingIOError:
                break
            if not data:
                break
            self.buffer += data
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            self.lines.append(line.decode().strip())

    def spin(self):
        self.executor.spin_once(timeout_sec=0.01)
        self.read_serial()
        for process, log in self.processes:
            assert process.poll() is None, os.pread(log.fileno(), 65536, 0).decode()

    def pump(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.spin()

    def until(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.spin()
        logs = '\n'.join(os.pread(log.fileno(), 65536, 0).decode()
                         for _, log in self.processes)
        assert predicate(), f'Timed out; serial={self.lines[-12:]}\n{logs}'

    def route(self, stage, vx=0.2):
        command = RouteCommand()
        command.stage_id = stage
        command.velocity.linear.x = vx
        self.route_pub.publish(command)

    def serial_feedback(self, text):
        os.write(self.master, (text + '\r\n').encode())

    def goal(self, stage, shift=False):
        if shift:
            goal = ShiftLane.Goal()
            goal.direction = -1
            goal.distance_m = 0.3
            goal.max_speed_mps = 0.1
            client = self.shift
        else:
            goal = FollowRow.Goal()
            goal.direction = 1
            goal.max_speed_mps = 0.2
            goal.front_slow_distance_m = 0.3
            goal.front_stop_distance_m = 0.05
            client = self.follow
        goal.stage_id = stage
        future = client.send_goal_async(goal)
        self.until(future.done)
        handle = future.result()
        assert handle.accepted
        result = handle.get_result_async()
        self.until(lambda: any(c.stage_id == stage and moving(c) for c in self.commands))
        if any('/base_serial_node' in p.args[0] for p, _ in self.processes):
            self.until(lambda: any(line.endswith(f' {stage}') for line in self.lines))
        return result

    def close(self):
        for process, log in reversed(self.processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            log.close()
        self.node.destroy_node()
        self.executor.shutdown()
        self.context.shutdown()
        os.close(self.master)
        os.close(self.slave)


def moving(command):
    v = command.velocity
    return abs(v.linear.x) + abs(v.linear.y) + abs(v.angular.z) > 1e-6


@pytest.fixture
def chassis(monkeypatch):
    # Isolate all absolute topic names from the robot and from neighboring tests.
    monkeypatch.setenv('ROS_DOMAIN_ID', str(next(_domains)))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    harness = ChassisHarness()
    try:
        yield harness
    finally:
        harness.close()


def test_legacy_wire_format(chassis):
    h = chassis
    h.start_bridge(staged=False)
    command = Twist()
    command.linear.x = 0.2
    h.twist_pub.publish(command)
    h.until(lambda: 'CMD_WHEELS 200 200 200 200' in h.lines)
    h.serial_feedback('STAGE_FB 1 1')
    h.route(1)
    h.pump(0.1)
    assert not h.feedback
    assert all(len(line.split()) == 5 for line in h.lines)


def test_stage_latch_parser_and_watchdog(chassis):
    h = chassis
    h.start_bridge(staged=True)
    h.route(1)
    h.until(lambda: 'CMD_WHEELS 200 200 200 200 1' in h.lines)
    for invalid in ('STAGE_FB 2 1', 'STAGE_FB 0 1', 'STAGE_FB 16 1',
                    'STAGE_FB 1 2', 'STAGE_FB 1 1 extra', 'STAGE_FB 1 true'):
        h.serial_feedback(invalid)
    h.pump(0.1)
    assert not h.feedback
    h.serial_feedback('STAGE_FB 1 1')
    h.until(lambda: any(f.stage_id == 1 and f.reached for f in h.feedback))
    h.lines.clear()
    h.route(1)
    h.serial_feedback('STAGE_FB 1 0')
    h.pump(0.1)
    assert h.lines and all(line == 'CMD_WHEELS 0 0 0 0 1' for line in h.lines)
    assert all(f.reached for f in h.feedback)
    h.route(2)
    h.until(lambda: 'CMD_WHEELS 200 200 200 200 2' in h.lines)
    h.serial_feedback('STAGE_FB 1 1')  # delayed old-stage arrival must not stop stage 2
    command = Twist()
    command.linear.x = -0.5
    h.twist_pub.publish(command)  # cmd_vel is a monitor mirror in staged mode
    h.pump(0.05)
    assert not any('-500' in line for line in h.lines)
    h.lines.clear()
    h.route(2)
    h.until(lambda: 'CMD_WHEELS 200 200 200 200 2' in h.lines)
    h.until(lambda: 'CMD_WHEELS 0 0 0 0 2' in h.lines)  # watchdog retains stage


def test_photoelectric_finishes_follow_and_shift(chassis):
    h = chassis
    h.start_bridge(staged=True)
    h.start_navigation()
    first = h.goal(1)
    h.serial_feedback('STAGE_FB 1 1')
    h.until(first.done)
    assert first.result().status == GoalStatus.STATUS_SUCCEEDED
    assert 'photoelectric' in first.result().result.message
    h.commands.clear()
    second = h.goal(2, shift=True)
    h.until(lambda: any(line.endswith(' 2') and 'CMD_WHEELS' in line for line in h.lines))
    h.serial_feedback('STAGE_FB 1 1')
    h.pump(0.15)
    assert not second.done()
    assert moving(h.commands[-1])
    h.serial_feedback('STAGE_FB 2 1')
    h.until(second.done)
    assert second.result().status == GoalStatus.STATUS_SUCCEEDED
    assert 'photoelectric' in second.result().result.message
    # Resuming the same stage cannot clear its stop latch.
    h.commands.clear()
    future = h.shift.send_goal_async(ShiftLane.Goal(
        stage_id=2, direction=-1, distance_m=0.3, max_speed_mps=0.1))
    h.until(future.done)
    handle = future.result()
    assert handle.accepted
    result = handle.get_result_async()
    h.until(result.done)
    assert h.commands and not any(moving(c) for c in h.commands)


@pytest.mark.parametrize('odom_mode', ['stale', 'moving'])
def test_no_next_stage_without_stopped_feedback(chassis, odom_mode):
    h = chassis
    h.start_bridge(staged=True)
    h.start_navigation()
    result = h.goal(1)
    if odom_mode == 'stale':
        h.odom_enabled = False
        h.pump(0.06)  # drain pre-trigger samples while they are still fresh
    else:
        h.odom_vx = 0.2
        h.pump(0.06)
    h.serial_feedback('STAGE_FB 1 1')
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert not result.result().result.success
    assert 'timeout' in result.result().result.message
    assert all(c.stage_id == 1 for c in h.commands)
    assert not moving(h.commands[-1])


def test_ultrasonic_arrival_still_works(chassis):
    h = chassis
    h.start_navigation()
    result = h.goal(0)  # standalone action: stage feedback must have no effect
    h.front_m = 0.04
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
    assert result.result().result.message == 'front wall distance reached'


def test_mission_sends_all_sixteen_stages(chassis):
    h = chassis
    h.start_bridge(staged=True)
    h.start_navigation(max_acceleration_mps2=100.0, max_deceleration_mps2=100.0)
    h.start('mission_behavior_node', step_count=16, reverse_stop_on_zero_velocity=False,
            shift_speed_mps=0.3, shift_first_lateral_speed_mps=0.2,
            shift_route_step_15_speed_mps=0.1)
    for stage in range(1, 16):
        h.until(lambda: any(c.stage_id == stage and moving(c) for c in h.commands))
        stage_commands = [c for c in h.commands if c.stage_id == stage]
        assert not moving(stage_commands[0])  # zero announces every stage switch
        v = next(c.velocity for c in stage_commands if moving(c))
        if stage == 1:
            assert v.linear.x > 0
        elif stage in (2, 4, 6, 8, 10, 12, 14, 15):
            expected_sign = -1 if stage in (2, 6, 10, 14) else 1
            assert v.linear.y * expected_sign > 0
            if stage in (2, 15):
                expected_speed = 0.2 if stage == 2 else 0.1
                h.until(lambda: any(
                    c.stage_id == stage and
                    abs(abs(c.velocity.linear.y) - expected_speed) < 1e-6
                    for c in h.commands))
        else:
            assert v.linear.x < 0
        h.until(lambda: any(line.endswith(f' {stage}') for line in h.lines))
        h.serial_feedback(f'STAGE_FB {stage} 1')
        h.until(lambda: h.commands[-1].stage_id == stage and not moving(h.commands[-1]))
    h.until(lambda: any(c.stage_id == 0 and c.velocity.linear.x < 0 for c in h.commands))
    h.odom_x = -0.70
    h.until(lambda: b'mission completed' in os.pread(h.processes[-1][1].fileno(), 65536, 0))
    assert sorted({c.stage_id for c in h.commands}) == list(range(0, 16))
    assert not moving(h.commands[-1])


def test_visual_shift_holds_until_measured_stop_and_never_advances(chassis):
    h = chassis
    h.start_navigation(vision_stop_settle_s=0.12)
    future = h.shift.send_goal_async(ShiftLane.Goal(
        stage_id=4, direction=-1, distance_m=0.3, max_speed_mps=0.3,
        vision_stop_enabled=True))
    h.until(future.done)
    goal = future.result()
    assert goal.accepted
    result = goal.get_result_async()
    h.until(lambda: h.commands and h.commands[-1].velocity.linear.y < -0.1)
    h.odom_vy = -0.2
    h.vision = VisionRowObservation(mango_in_range=True, tree_valid=True,
                                  tree_center_error_ratio=-0.10)
    h.until(lambda: any(s.state == s.ALIGNING for s in h.vision_states))
    h.vision.tree_center_error_ratio = 0.0
    h.until(lambda: any(s.state == s.SETTLING for s in h.vision_states))
    h.pump(0.3)
    assert not any(s.state == s.HOLDING for s in h.vision_states)
    h.odom_vy = 0.0
    h.until(lambda: any(s.state == s.HOLDING for s in h.vision_states))
    h.commands.clear()
    h.vision = None
    h.pump(0.8)  # no camera frames after HOLDING must never resume route motion
    assert not result.done()
    assert h.commands and not any(moving(c) for c in h.commands)


def test_visual_shift_camera_loss_aborts_with_zero(chassis):
    h = chassis
    h.start_navigation(vision_observation_timeout_s=0.2)
    future = h.shift.send_goal_async(ShiftLane.Goal(
        stage_id=6, direction=-1, distance_m=0.3, max_speed_mps=0.1,
        vision_stop_enabled=True))
    h.until(future.done)
    handle = future.result()
    result = handle.get_result_async()
    h.vision = VisionRowObservation(mango_in_range=True, tree_valid=True,
                                  tree_center_error_ratio=-0.1)
    h.until(lambda: any(s.state == s.ALIGNING for s in h.vision_states))
    h.vision = None
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert 'stale' in result.result().result.message
    assert not moving(h.commands[-1])
