"""Exercise the real chassis nodes with a PTY and simulated sensors, never hardware."""

import itertools
import math
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
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from sensor_msgs.msg import Imu
from robot_arm_interfaces.action import FollowRow, ShiftLane
from robot_arm_interfaces.msg import (RouteCommand, StageFeedback, UltrasonicArray,
                                      VisionRowObservation, VisionStopState, HarvestResult)


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
        self.imu_pub = self.node.create_publisher(Imu, '/imu/wit/data', qos_profile_sensor_data)
        self.imu_enabled = True
        self.imu_yaw = 0.0
        self.imu_mode = 'valid'
        self.frozen_imu_stamp = None
        self.sensor_timer = None
        self.front_m = 2.0
        self.rear_m = 2.0
        self.left_m = 2.0
        self.right_m = 2.0
        self.left_valid = True
        self.right_valid = True
        self.ultrasonic_enabled = True
        self.odom_enabled = True
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.vision = None
        self.vision_states = []
        self.harvest_result_pub = self.node.create_publisher(HarvestResult, '/harvest/result', 10)
        self.vision_pub = self.node.create_publisher(
            VisionRowObservation, '/vision/row_observation', qos_profile_sensor_data)
        self.node.create_subscription(VisionStopState, '/base/vision_stop_state',
                                      self.vision_states.append, 10)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

    def start(self, executable, **parameters):
        # ament APPEND_ENV may prefix an unset variable with a path separator.
        binary_dir = next((part for part in os.environ.get(
            'MANGO_TEST_CHASSIS_BIN_DIR', '').split(os.pathsep) if part), None)
        if not binary_dir:
            binary_dir = os.path.join(get_package_prefix('robot_chassis'), 'lib', 'robot_chassis')
        binary = os.path.join(binary_dir, executable)
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
        defaults = dict(stop_settle_s=0.08, photoelectric_stop_timeout_s=0.8,
                        ultrasonic_filter_window=1)
        defaults.update(parameters)
        self.start('row_navigation_node', **defaults)
        self.follow = ActionClient(self.node, FollowRow, '/follow_row')
        self.shift = ActionClient(self.node, ShiftLane, '/shift_lane')
        self.until(lambda: self.follow.server_is_ready() and self.shift.server_is_ready())
        self.pump(0.2)

    def publish_sensors(self):
        msg = UltrasonicArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.front_m = self.front_m
        msg.rear_m = self.rear_m
        msg.left_m = self.left_m
        msg.right_m = self.right_m
        msg.front_valid = msg.rear_valid = msg.left_valid = msg.right_valid = True
        msg.left_valid = self.left_valid
        msg.right_valid = self.right_valid
        if self.ultrasonic_enabled:
            self.ultra_pub.publish(msg)
        if self.odom_enabled:
            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.pose.pose.orientation.w = math.cos(self.odom_yaw / 2.0)
            odom.pose.pose.orientation.z = math.sin(self.odom_yaw / 2.0)
            odom.pose.pose.position.x = self.odom_x
            odom.pose.pose.position.y = self.odom_y
            odom.twist.twist.linear.x = self.odom_vx
            odom.twist.twist.linear.y = self.odom_vy
            self.odom_pub.publish(odom)
        if self.imu_enabled:
            imu = Imu()
            imu.header.stamp = msg.header.stamp
            imu.header.frame_id = 'wit_imu_link'
            imu.orientation.w = math.cos(self.imu_yaw / 2.0)
            imu.orientation.z = math.sin(self.imu_yaw / 2.0)
            if self.imu_mode == 'zero_quaternion':
                imu.orientation.w = imu.orientation.z = 0.0
            elif self.imu_mode == 'nan':
                imu.orientation.z = float('nan')
            elif self.imu_mode == 'no_orientation':
                imu.orientation_covariance[0] = -1.0
            elif self.imu_mode == 'old_stamp':
                imu.header.stamp.sec -= 5
            elif self.imu_mode == 'future_stamp':
                imu.header.stamp.sec += 5
            elif self.imu_mode == 'zero_stamp':
                imu.header.stamp.sec = imu.header.stamp.nanosec = 0
            elif self.imu_mode == 'frozen_stamp':
                if self.frozen_imu_stamp is None:
                    self.frozen_imu_stamp = msg.header.stamp
                imu.header.stamp = self.frozen_imu_stamp
            self.imu_pub.publish(imu)
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

    def goal(self, stage, shift=False, **fields):
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
        for name, value in fields.items():
            setattr(goal, name, value)
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


@pytest.mark.parametrize('shift,direction', [(False, 1), (False, -1), (True, -1)])
def test_external_heading_controls_all_motion_and_stops_on_disconnect(chassis, shift, direction):
    h = chassis
    h.start_navigation(yaw_kp=1.0, yaw_ki=0.0, imu_timeout_s=0.15)
    result = h.goal(0, shift=shift, direction=direction)
    h.odom_yaw = 0.7
    h.pump(0.15)
    assert h.commands and all(abs(c.velocity.angular.z) < 1e-6 for c in h.commands)
    h.imu_yaw = 0.1
    h.until(lambda: h.commands[-1].velocity.angular.z == pytest.approx(-0.1, abs=1e-5))
    h.imu_yaw = -0.1
    h.until(lambda: h.commands[-1].velocity.angular.z == pytest.approx(0.1, abs=1e-5))
    h.imu_enabled = False
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.reason == 1
    assert 'heading' in result.result().result.message
    # Action results and command topics are delivered independently by DDS.
    h.until(lambda: h.commands and not moving(h.commands[-1]))
    stopped_count = len(h.commands)
    h.pump(0.2)  # Fresh old odometry must not reactivate motion after IMU failure.
    assert not any(moving(c) for c in h.commands[stopped_count:])


@pytest.mark.parametrize('mode', [
    'zero_quaternion', 'nan', 'no_orientation', 'old_stamp',
    'future_stamp', 'zero_stamp', 'frozen_stamp'])
def test_invalid_external_heading_never_keeps_motion_alive(chassis, mode):
    h = chassis
    h.start_navigation(imu_timeout_s=0.15)
    result = h.goal(0, shift=True)
    h.imu_mode = mode
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.reason == 1
    h.until(lambda: h.commands and not moving(h.commands[-1]))


@pytest.mark.parametrize('shift', [False, True])
def test_no_motion_until_external_imu_is_ready(chassis, shift):
    h = chassis
    h.imu_enabled = False
    h.start_navigation()
    if shift:
        goal = ShiftLane.Goal(direction=-1, distance_m=0.3, max_speed_mps=0.1)
        client = h.shift
    else:
        goal = FollowRow.Goal(direction=1, max_speed_mps=0.1,
                              front_slow_distance_m=0.3, front_stop_distance_m=0.05)
        client = h.follow
    sent = client.send_goal_async(goal)
    h.until(sent.done)
    handle = sent.result()
    assert handle.accepted
    result = handle.get_result_async()
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert 'heading' in result.result().result.message
    h.until(lambda: bool(h.commands))
    assert h.commands and not any(moving(c) for c in h.commands)


@pytest.mark.parametrize('shift', [False, True])
def test_encoder_distance_retains_odometry_frame_with_new_heading(chassis, shift):
    h = chassis
    h.imu_yaw = math.pi / 2  # A different heading zero must not rotate encoder positions.
    if shift:
        h.right_valid = False  # Encoder distance applies only without a valid target-side wall.
    h.start_navigation()
    result = h.goal(0, shift=shift, direction=-1)
    if shift:
        h.odom_y = -0.32
    else:
        h.odom_x = -0.72
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
    assert result.result().result.traveled_m > (0.3 if shift else 0.7)
    h.until(lambda: h.commands and not moving(h.commands[-1]))


def test_explicit_legacy_heading_mode_remains_available(chassis):
    h = chassis
    h.imu_enabled = False
    h.start_navigation(use_external_imu=False, yaw_kp=1.0, yaw_ki=0.0)
    result = h.goal(1)
    h.odom_yaw = 0.1
    h.until(lambda: h.commands[-1].velocity.angular.z == pytest.approx(-0.1, abs=1e-5))
    h.front_m = 0.04
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED


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


def test_first_row_left_correction_starts_only_after_entry(chassis):
    h = chassis
    h.start_navigation()
    result = h.goal(1, use_side_centering=True, side_target_m=0.05)
    h.imu_yaw = 0.1  # Heading control must still work before row entry.
    h.until(lambda: h.commands[-1].velocity.angular.z < -0.05)
    assert h.commands[-1].velocity.linear.x > 0.0
    assert h.commands[-1].velocity.linear.y == 0.0

    for distance, valid in ((0.10, True), (0.08, False), (0.0, True),
                            (-0.01, True), (float('nan'), True)):
        h.left_m, h.left_valid = distance, valid
        h.pump(0.15)
        h.commands.clear()
        h.pump(0.15)
        assert h.commands and all(c.velocity.linear.y == 0.0 for c in h.commands)

    h.left_m, h.left_valid = 0.08, True
    h.until(lambda: h.commands[-1].velocity.linear.y == pytest.approx(0.036, abs=1e-6))
    h.left_m = 0.15  # Once inside the row, do not toggle off at 10 cm again.
    h.until(lambda: h.commands[-1].velocity.linear.y == pytest.approx(0.12, abs=1e-6))
    h.left_valid = False
    h.until(lambda: h.commands[-1].velocity.linear.y == 0.0)
    h.front_m = 0.04  # Front stopping is still active after the gate opens.
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
    h.until(lambda: not moving(h.commands[-1]))


@pytest.mark.parametrize('stage,direction,side_sensor,centering,expected_y', [
    (0, 1, 0, True, 0.15),     # Standalone forward action has no first-stage gate.
    (3, -1, 0, True, 0.15),    # Later reverse stage still uses its configured side.
    (1, 1, 1, True, -0.15),    # The new entry gate applies only to the left sensor.
    (1, 1, 0, False, 0.0),    # It must never turn on explicitly disabled centering.
])
def test_first_row_gate_does_not_change_other_follow_modes(
        chassis, stage, direction, side_sensor, centering, expected_y):
    h = chassis
    h.start_navigation()
    h.goal(stage, direction=direction, side_sensor=side_sensor,
           use_side_centering=centering, side_target_m=0.05)
    h.until(lambda: h.commands[-1].velocity.linear.y == pytest.approx(expected_y))


def test_first_row_gate_resets_on_new_goal_and_keeps_stale_stop(chassis):
    h = chassis
    h.left_m = 0.08
    h.start_navigation()
    result = h.goal(1, use_side_centering=True, side_target_m=0.05)
    h.until(lambda: h.commands[-1].velocity.linear.y > 0.02)
    h.front_m = 0.04
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
    h.front_m, h.left_m = 2.0, 2.0
    h.pump(0.15)
    h.commands.clear()
    second = h.goal(1, use_side_centering=True, side_target_m=0.05)
    h.pump(0.2)
    assert h.commands and all(c.velocity.linear.y == 0.0 for c in h.commands)
    h.ultrasonic_enabled = False
    h.until(second.done)
    assert second.result().status == GoalStatus.STATUS_ABORTED
    assert 'stale' in second.result().result.message
    h.until(lambda: not moving(h.commands[-1]))


def test_first_row_entry_threshold_can_be_changed_at_runtime(chassis):
    h = chassis
    h.left_m = 0.12
    h.start_navigation(ultrasonic_filter_window=3)
    h.goal(1, use_side_centering=True, side_target_m=0.05)
    h.pump(0.2)
    assert h.commands[-1].velocity.linear.y == 0.0
    client = h.node.create_client(SetParametersAtomically,
                                  '/row_navigation_node/set_parameters_atomically')
    h.until(client.service_is_ready)
    for value in (0.0, -0.1, float('nan'), 0.13):
        request = SetParametersAtomically.Request(parameters=[Parameter(
            name='first_row_left_enable_distance_m',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))])
        future = client.call_async(request)
        h.until(future.done)
        assert future.result().result.successful == (value == 0.13)
    h.until(lambda: h.commands[-1].velocity.linear.y == pytest.approx(0.084, abs=1e-6))


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


@pytest.mark.parametrize('stage,direction,wall_sensor', [
    (4, 1, 1), (6, -1, 1), (8, 1, 2),
    (10, -1, 1), (12, 1, 1), (14, -1, 1),
])
def test_visual_shift_keeps_wall_tracking_until_settling(
        chassis, stage, direction, wall_sensor):
    h = chassis
    wall_attr = 'rear_m' if wall_sensor == 2 else 'front_m'
    wall_sign = -1.0 if wall_sensor == 2 else 1.0
    h.front_m = h.rear_m = 0.4
    setattr(h, wall_attr, 0.13)
    h.start_navigation(shift_front_kp=2.0, max_front_correction_mps=0.08,
                       vision_tree_kp=0.5, vision_tree_ki=0.0,
                       vision_stop_settle_s=0.12, yaw_kp=1.0, yaw_ki=0.0)
    result = h.goal(stage, shift=True, direction=direction, distance_m=2.0,
                    max_speed_mps=0.3, wall_sensor=wall_sensor,
                    wall_target_m=0.10, wall_enable_tolerance_m=0.05,
                    vision_stop_enabled=True)

    def expect_velocity(vx, vy):
        h.until(lambda: h.commands[-1].velocity.linear.x == pytest.approx(vx) and
                h.commands[-1].velocity.linear.y == pytest.approx(vy))

    expect_velocity(wall_sign * 0.06, direction * 0.3)
    h.vision = VisionRowObservation(mango_in_range=True, tree_valid=False)
    h.until(lambda: h.vision_states and h.vision_states[-1].state ==
            VisionStopState.SEARCHING)
    expect_velocity(wall_sign * 0.06, direction * 0.12)
    # The original enable latch, correction cap, and sensor-specific sign survive.
    setattr(h, wall_attr, 0.30)
    expect_velocity(wall_sign * 0.08, direction * 0.12)
    setattr(h, wall_attr, 0.07)
    expect_velocity(-wall_sign * 0.06, direction * 0.12)

    h.vision.tree_valid = True
    h.vision.tree_center_error_ratio = direction * 0.1
    h.until(lambda: h.vision_states[-1].state == VisionStopState.ALIGNING)
    expect_velocity(-wall_sign * 0.06, direction * 0.05)
    h.imu_yaw = 0.1
    h.until(lambda: h.commands[-1].velocity.angular.z == pytest.approx(-0.1))

    # A nonzero wall error must not move the base while confirming a parked tree.
    h.odom_vx = 0.1
    h.vision.tree_center_error_ratio = 0.0
    h.until(lambda: h.vision_states[-1].state == VisionStopState.SETTLING and
            not moving(h.commands[-1]))
    h.commands.clear()
    h.pump(0.2)
    assert h.commands and not any(moving(c) for c in h.commands)
    assert h.vision_states[-1].state == VisionStopState.SETTLING
    h.odom_vx = 0.0
    h.until(lambda: h.vision_states[-1].state == VisionStopState.HOLDING)
    parking = h.vision_states[-1]
    setattr(h, wall_attr, 0.30)
    h.commands.clear()
    h.pump(0.2)
    assert h.commands and not any(moving(c) for c in h.commands)
    assert not result.done()

    # A matching home confirmation resumes both lane travel and the same wall loop.
    setattr(h, wall_attr, 0.13)
    complete = HarvestResult(stage_id=stage, stop_id=parking.stop_id,
                             success=True, returned_home=True)
    complete.header.stamp = h.node.get_clock().now().to_msg()
    h.harvest_result_pub.publish(complete)
    expect_velocity(wall_sign * 0.06, direction * 0.1)
    assert not result.done()


@pytest.mark.parametrize('wall_sensor,distance', [(0, 0.13), (1, 0.30), (1, float('nan'))])
def test_visual_shift_retains_wall_sensor_and_activation_gates(chassis, wall_sensor, distance):
    h = chassis
    h.front_m = distance
    h.start_navigation(shift_front_kp=2.0)
    h.goal(4, shift=True, direction=1, distance_m=2.0, max_speed_mps=0.3,
           wall_sensor=wall_sensor, wall_target_m=0.10,
           wall_enable_tolerance_m=0.05, vision_stop_enabled=True)
    h.vision = VisionRowObservation(mango_in_range=True, tree_valid=False)
    h.until(lambda: h.vision_states and h.vision_states[-1].state ==
            VisionStopState.SEARCHING)
    h.vision.tree_valid = True
    h.vision.tree_center_error_ratio = 0.1
    h.until(lambda: h.vision_states[-1].state == VisionStopState.ALIGNING)
    h.commands.clear()
    h.pump(0.2)
    assert h.commands and all(c.velocity.linear.x == 0.0 for c in h.commands)
    h.front_m = 0.13
    if wall_sensor:
        h.until(lambda: h.commands[-1].velocity.linear.x == pytest.approx(0.06))
    else:
        h.pump(0.2)
        assert h.commands[-1].velocity.linear.x == 0.0


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
    h.until(lambda: h.commands and not moving(h.commands[-1]))


@pytest.mark.parametrize('stage', [4, 6, 8, 10, 12, 14])
def test_harvest_home_resumes_same_lane_and_adjacent_tree_without_empty_scene(chassis, stage):
    h = chassis
    h.start_navigation(vision_stop_settle_s=0.08, vision_rearm_delay_s=0.1,
                       vision_rearm_distance_m=0.1, vision_rearm_speed_mps=0.08)
    future = h.shift.send_goal_async(ShiftLane.Goal(
        stage_id=stage, direction=-1, distance_m=2.0, max_speed_mps=0.7,
        vision_stop_enabled=True))
    h.until(future.done)
    result = future.result().get_result_async()
    h.vision = VisionRowObservation(mango_in_range=True, tree_valid=True,
                                   tree_center_error_ratio=0.0)
    h.until(lambda: any(s.state == s.HOLDING for s in h.vision_states))
    parking = next(s for s in reversed(h.vision_states) if s.state == s.HOLDING)
    assert parking.stop_id != 0

    def complete(**overrides):
        msg = HarvestResult(stage_id=stage, stop_id=parking.stop_id,
                            success=True, returned_home=True)
        msg.header.stamp = h.node.get_clock().now().to_msg()
        for key, value in overrides.items():
            setattr(msg, key, value)
        h.harvest_result_pub.publish(msg)
        h.pump(.15)

    for fields in ({'stage_id': stage + 1}, {'stop_id': parking.stop_id + 1},
                   {'success': False}, {'returned_home': False}, {'dry_run': True}):
        complete(**fields)
        assert not moving(h.commands[-1]) and not result.done()
    complete()
    h.until(lambda: h.commands[-1].velocity.linear.y < -0.01)
    assert abs(h.commands[-1].velocity.linear.y) <= .08 + 1e-8
    assert not result.done()  # Continues the existing lane, not the next route step.
    h.vision_states.clear()
    h.pump(.3)
    assert not any(s.state == s.HOLDING for s in h.vision_states)
    h.vision.tree_center_error_ratio = .15
    h.pump(.1)
    h.odom_y = -.12
    # The old trunk is behind us. Release the departure cap even though no
    # next trunk has appeared, and keep suppressing the old target.
    h.until(lambda: h.commands[-1].velocity.linear.y < -.69)
    assert not any(s.state in (s.ALIGNING, s.HOLDING) for s in h.vision_states)
    h.vision.tree_center_error_ratio = -.12  # Acquire the next trunk before it is centered.
    h.until(lambda: any(s.state == s.ALIGNING for s in h.vision_states))
    h.vision.tree_center_error_ratio = 0.0
    h.until(lambda: any(s.state == s.HOLDING for s in h.vision_states))
    second = next(s for s in reversed(h.vision_states) if s.state == s.HOLDING)
    assert second.stop_id != parking.stop_id
    complete()  # Late duplicate completion for the first tree cannot release the second.
    assert not moving(h.commands[-1])
    complete(stop_id=second.stop_id)
    h.until(lambda: h.commands[-1].velocity.linear.y < -0.01)
    h.vision_states.clear()
    h.vision = VisionRowObservation(mango_in_range=False, tree_valid=False)
    h.odom_y = -.24
    # After the last tree, an empty lane must also return to route speed.
    h.until(lambda: h.commands[-1].velocity.linear.y < -.69)
    h.pump(.2)
    assert not result.done()
    assert not any(s.state in (s.ALIGNING, s.HOLDING) for s in h.vision_states)
    h.right_m = .05  # Existing valid side-wall arrival completes the lane.
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED


def test_tree_alignment_timeout_skips_tree_and_continues_route(chassis):
    h = chassis
    h.start_navigation(vision_tree_alignment_timeout_s=.4)
    future = h.shift.send_goal_async(ShiftLane.Goal(
        stage_id=4, direction=-1, distance_m=2.0, max_speed_mps=.1,
        vision_stop_enabled=True))
    h.until(future.done)
    result = future.result().get_result_async()
    h.vision = VisionRowObservation(mango_in_range=True, tree_valid=True,
                                   tree_center_error_ratio=.2)
    h.until(lambda: any(s.state == s.ALIGNING for s in h.vision_states))
    h.vision_states.clear()  # Require a new timeout IDLE, not the initial route status.
    h.until(lambda: any(s.state == s.IDLE for s in h.vision_states))
    assert not result.done()
    h.until(lambda: h.commands[-1].velocity.linear.y < -.01)
    h.right_m = .05
    h.until(result.done)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
