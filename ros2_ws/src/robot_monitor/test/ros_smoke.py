#!/usr/bin/env python3
"""Isolated ROS adapter smoke test. Publishes sensor telemetry ONLY, no motion commands."""
import argparse
import json
import os
import signal
import subprocess
import time
from urllib.request import urlopen


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--domain-id', type=int, default=157)
    p.add_argument('--port', type=int, default=18765)
    args = p.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from robot_arm_interfaces.msg import WheelState, UltrasonicArray

    rclpy.init()
    probe = Node('robot_monitor_sensor_smoke')
    # Refuse any pre-existing hardware/control nodes on the chosen test domain.
    time.sleep(1.0)
    others = [name for name, _ in probe.get_node_names_and_namespaces()
              if name not in ('robot_monitor_sensor_smoke', '_ros2cli_daemon_0') and not name.startswith('_ros2cli')]
    if others:
        probe.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Test domain already has nodes: ' + ', '.join(others))
    parameters = Node('base_serial_node')
    for name, value in {'wheel_base_m': .26, 'track_width_m': .24,
                        'max_wheel_speed_mps': .9, 'feedback_timeout_s': .30,
                        'command_timeout_s': .20}.items():
        parameters.declare_parameter(name, value)
    wheels = probe.create_publisher(WheelState, '/base/wheel_state', qos_profile_sensor_data)
    ultra = probe.create_publisher(UltrasonicArray, '/base/ultrasonic', qos_profile_sensor_data)
    # Start the installed executable, exercising its installed web asset paths.
    env = dict(os.environ)
    process = subprocess.Popen(['ros2', 'run', 'robot_monitor', 'robot_monitor.py',
                                '--no-record', '--port', str(args.port)],
                               env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, start_new_session=True)
    success = False
    try:
        deadline = time.monotonic()+18
        state = None
        while time.monotonic() < deadline:
            msg = WheelState()
            msg.header.stamp = probe.get_clock().now().to_msg()
            msg.valid = True
            msg.speed_mps = [0.0, 0.0, 0.0, 0.0]
            msg.encoder_delta = [0, 0, 0, 0]
            msg.yaw_rad = .123
            wheels.publish(msg)
            u = UltrasonicArray()
            u.header.stamp = msg.header.stamp
            u.front_m, u.rear_m, u.left_m, u.right_m = .20, .21, .4, .6
            u.front_valid = u.rear_valid = u.left_valid = u.right_valid = True
            ultra.publish(u)
            rclpy.spin_once(parameters, timeout_sec=.01)
            rclpy.spin_once(probe, timeout_sec=.01)
            try:
                with urlopen('http://127.0.0.1:%d/api/state' % args.port, timeout=.3) as response:
                    state = json.load(response)
                if (state['topics']['/base/wheel_state']['count'] >= 5 and
                    state['topics']['/base/ultrasonic']['message']['front_valid'] and
                    state['settings']['wheel_base_m'] == .26 and
                    state['settings']['command_timeout_s'] == .20):
                    with urlopen('http://127.0.0.1:%d/' % args.port, timeout=1) as response:
                        assert '四轮速度' in response.read().decode()
                    assert not state['events'], 'No commands should have been sent'
                    assert not state['faults'], state['faults']
                    success = True
                    print(json.dumps({'result': 'ROS_SMOKE_OK', 'domain_id': args.domain_id,
                                      'received_wheel_samples': state['topics']['/base/wheel_state']['count'],
                                      'ultrasonic': state['topics']['/base/ultrasonic']['message']['front_m'],
                                      'geometry_source': state['geometry_source'],
                                      'no_motion_publishers': True}, ensure_ascii=False))
                    break
            except (OSError, KeyError):
                pass
            if process.poll() is not None:
                break
            time.sleep(.03)
        if not success:
            raise AssertionError('ROS adapter smoke test did not receive expected data/parameters: ' + str(state))
    finally:
        os.killpg(process.pid, signal.SIGINT) if process.poll() is None else None
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            output, _ = process.communicate(timeout=3)
        print(output[-6000:])
        parameters.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
