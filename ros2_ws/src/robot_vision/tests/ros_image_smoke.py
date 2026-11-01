"""Test installed node + actual model + ROS messages without any motion nodes."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

# Keep test discovery separate from the robot's normal ROS domain.
os.environ['ROS_DOMAIN_ID'] = '73'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from robot_arm_interfaces.msg import MangoTarget
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True, type=Path)
    parser.add_argument('--device', default='0')
    args = parser.parse_args()
    rclpy.init()
    observer = rclpy.create_node('mango_image_smoke_observer')
    observations, targets = [], []
    observer.create_subscription(String, '/vision/observations',
                                 lambda msg: observations.append(json.loads(msg.data)), 10)
    observer.create_subscription(MangoTarget, '/vision/mango_target', targets.append,
                                 qos_profile_sensor_data)
    executable = Path(get_package_prefix('robot_vision')) / 'lib/robot_vision/mango_vision_node.py'
    with tempfile.TemporaryFile(mode='w+', encoding='utf-8') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-p',
                                    f'source:={args.image.resolve()}', '-p',
                                    f'device:="{args.device}"', '-p', 'publish_targets:=false'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline and not (observations and targets):
                if process.poll() is not None:
                    raise RuntimeError(f'Vision node exited with {process.returncode}')
                rclpy.spin_once(observer, timeout_sec=.2)
            if not observations or not targets:
                raise RuntimeError('Did not receive both ROS topics')
            observation = observations[-1]
            if not observation['fruits']:
                raise AssertionError('Real trained model did not detect any fruit in the test image')
            if any(message.valid for message in targets) or observation['target_valid']:
                raise AssertionError('RGB-only smoke test must never produce a valid 3D pick target')
            print(json.dumps({'check': 'trained_weights_ros_image_inference', 'passed': True,
                              'device': args.device, 'fruits': len(observation['fruits']),
                              'decision': observation['decision']['status'],
                              'target_valid': False, 'ros_domain_id': 73}, indent=2))
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
