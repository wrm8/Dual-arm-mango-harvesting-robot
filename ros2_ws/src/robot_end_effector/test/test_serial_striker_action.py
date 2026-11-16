"""Real ROS Action and virtual UART: success needs no reply from the MCU."""
import os
import pty
import select
import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from robot_arm_interfaces.action import TriggerStrike

from robot_end_effector.serial_striker_node import SerialStrikerNode


def wait(future, timeout=3):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.005)
    assert future.done()
    return future.result()


@pytest.fixture
def striker(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(80 + os.getpid() % 40))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    master, slave = pty.openpty()
    rclpy.init()
    # Use the actual node's default v1 frames, not test-only protocol overrides.
    params = {'dry_run': False, 'allow_motion': True, 'serial_device': os.ttyname(slave)}
    node = SerialStrikerNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    client_node = rclpy.create_node('serial_striker_test_client')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    client = ActionClient(client_node, TriggerStrike, '/end_effector/trigger_strike')
    assert client.wait_for_server(timeout_sec=3)
    try:
        yield node, client, master
    finally:
        node.close()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        client.destroy()
        node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


def send(client, arms=('left',), wait_s=.15):
    return wait(client.send_goal_async(TriggerStrike.Goal(arms=list(arms), wait_s=wait_s)))


def test_trigger_once_then_wait_without_any_mcu_reply(striker):
    _, client, master = striker
    started = time.monotonic()
    handle = send(client, arms=['left', 'right'])
    assert handle.accepted
    assert select.select([master], [], [], .5)[0]
    assert os.read(master, 256) == b'\xAA\x55\x01\x03\x0D\x0A'
    result = wait(handle.get_result_async())
    assert time.monotonic() - started >= .15
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
    assert 'not verified' in result.result.message
    assert not select.select([master], [], [], .02)[0]
    # A new goal must cause a new single trigger; no latched or repeated publication.
    assert wait(send(client, arms=['right'], wait_s=0.).get_result_async()).result.success
    assert os.read(master, 256) == b'\xAA\x55\x01\x02\x0D\x0A'


@pytest.mark.parametrize('arms,delay', [([], .1), (['left', 'left'], .1), (['other'], .1),
                                      (['left'], -1.), (['left'], float('nan')),
                                      (['left'], float('inf')), (['left'], 31.)])
def test_invalid_goals_never_transmit(striker, arms, delay):
    _, client, master = striker
    assert not send(client, arms, delay).accepted
    assert not select.select([master], [], [], .02)[0]


def test_busy_and_cancel_do_not_send_extra_frames(striker):
    _, client, master = striker
    handle = send(client, wait_s=2.)
    assert select.select([master], [], [], .5)[0]
    assert os.read(master, 256) == b'\xAA\x55\x01\x01\x0D\x0A'
    assert not send(client).accepted
    wait(handle.cancel_goal_async())
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED and not result.result.success
    assert 'STM32 may still run' in result.result.message
    assert not select.select([master], [], [], .02)[0]


def test_serial_failure_aborts_instead_of_claiming_completion(striker):
    node, client, _ = striker
    node.io.close()
    result = wait(send(client).get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED and not result.result.success
    assert 'not retried' in result.result.message


def test_default_interlocks_never_open_a_serial_port():
    rclpy.init()
    node = SerialStrikerNode()
    try:
        assert node.io is None
        assert node.accept(TriggerStrike.Goal(arms=['left'], wait_s=1.)) == GoalResponse.REJECT
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
