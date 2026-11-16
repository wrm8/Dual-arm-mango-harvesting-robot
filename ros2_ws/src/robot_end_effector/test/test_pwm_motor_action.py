"""Exercise the actual ROS Action server with its explicit simulation backend."""
import os
import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from robot_arm_interfaces.action import RunMotors
from std_srvs.srv import Trigger
from robot_end_effector.pwm_motor_node import PwmMotorNode


@pytest.fixture
def motor(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(80 + os.getpid() % 40))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init(args=['--ros-args', '-p', 'simulate:=true'])
    node = PwmMotorNode()
    client_node = rclpy.create_node('pwm_test_client')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    client = ActionClient(client_node, RunMotors, '/end_effector/run_motors')
    assert client.wait_for_server(timeout_sec=3)
    try:
        yield node, client, client_node
    finally:
        node.close()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        client.destroy()
        node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


def wait(future, timeout=3):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.005)
    assert future.done()
    return future.result()


def goal(client, arms=None, duty=.5, duration=.1, use_encoder=False, revolutions=10.):
    return wait(client.send_goal_async(RunMotors.Goal(
        arms=arms or ['left', 'right'], duty_cycle=float(duty), duration_s=float(duration),
        use_encoder=use_encoder, revolutions=revolutions)))


def test_timed_action_stops_before_reporting_success(motor):
    node, client, _ = motor
    started = time.monotonic()
    handle = goal(client, duration=.15)
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
    assert time.monotonic() - started >= .15
    assert node.io.active == set()


def test_at8236_pin_defaults_avoid_spi_header_lines(motor):
    node, _, _ = motor
    assert node.get_parameter('driver_type').value == 'at8236'
    assert [node.get_parameter(name).value for name in (
        'left.encoder_a_pin', 'left.encoder_b_pin',
        'right.encoder_a_pin', 'right.encoder_b_pin')] == [12, 29, 31, 15]


@pytest.mark.parametrize('duty,duration', [(0, .1), (1.1, .1), (.5, -1), (.5, 31),
                                        (float('nan'), .1), (.5, float('inf'))])
def test_invalid_pwm_or_time_is_rejected(motor, duty, duration):
    node, client, _ = motor
    assert not goal(client, duty=duty, duration=duration).accepted
    assert node.io.active == set()


def test_cancel_and_busy_rejection_stop_motor(motor):
    node, client, _ = motor
    handle = goal(client, arms=['right'], duration=2)
    assert handle.accepted
    assert not goal(client, duration=.1).accepted
    wait(handle.cancel_goal_async())
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert not result.result.success and node.io.active == set()


def test_stop_service_aborts_active_run(motor):
    node, client, client_node = motor
    handle = goal(client, duration=2)
    service = client_node.create_client(Trigger, '/end_effector/stop')
    assert service.wait_for_service(timeout_sec=3)
    assert wait(service.call_async(Trigger.Request())).success
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED and not result.result.success
    assert node.io.active == set()


def test_continuous_output_persists_after_action_and_stop_service_disables_it(motor):
    node, client, client_node = motor
    result = wait(goal(client, arms=['left'], duration=0).get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
    assert node.io.active == {'left'}
    service = client_node.create_client(Trigger, '/end_effector/stop')
    assert service.wait_for_service(timeout_sec=3)
    assert wait(service.call_async(Trigger.Request())).success
    assert node.io.active == set()


def test_stop_failure_does_not_report_action_success(motor):
    node, client, _ = motor
    original_stop = node.io.stop
    def failed_stop():
        original_stop()
        raise RuntimeError('fake output disable failure')
    node.io.stop = failed_stop
    try:
        result = wait(goal(client).get_result_async())
        assert result.status == GoalStatus.STATUS_ABORTED and not result.result.success
        assert 'failed to stop PWM' in result.result.message
    finally:
        node.io.stop = original_stop


def test_default_interlocks_do_not_load_gpio_or_accept_motion(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(80 + os.getpid() % 40))
    rclpy.init(args=[])
    node = PwmMotorNode()
    try:
        assert node.io is None
        request = RunMotors.Goal(arms=['left'], duty_cycle=.5, duration_s=1.0)
        assert node.accept(request) == GoalResponse.REJECT
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


class FakeEncoders:
    def __init__(self):
        self.counts = {'left': 0, 'right': 0}
    def snapshot(self):
        return dict(self.counts)
    def close(self):
        pass


def test_encoder_move_waits_for_both_counts_and_disables_outputs(motor):
    node, client, _ = motor
    node.encoders = FakeEncoders()
    node.counts_per_rev = {'left': 10., 'right': 20.}
    handle = goal(client, duration=2, use_encoder=True)
    result = handle.get_result_async()
    deadline = time.monotonic() + 1
    while not node.io.active and time.monotonic() < deadline:
        time.sleep(.005)
    assert node.io.active
    node.encoders.counts['left'] = 100
    time.sleep(.05)
    assert not result.done()
    node.encoders.counts['right'] = 200
    result = wait(result)
    assert result.result.success and node.io.active == set()


def test_encoder_mode_rejects_unknown_counts_per_rev(motor):
    node, client, _ = motor
    node.encoders = FakeEncoders()
    assert not goal(client, duration=2, use_encoder=True).accepted
    assert node.io.active == set()


def test_encoder_timeout_stops_both_outputs(motor):
    node, client, _ = motor
    node.encoders = FakeEncoders()
    node.counts_per_rev = {'left': 10., 'right': 20.}
    result = wait(goal(client, duration=.1, use_encoder=True).get_result_async())
    assert not result.result.success and 'timed out' in result.result.message
    assert node.io.active == set()
