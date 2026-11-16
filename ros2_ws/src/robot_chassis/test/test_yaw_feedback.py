"""Verify actual serial parsing publishes consistently filtered yaw, using only a PTY."""
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from robot_arm_interfaces.msg import BaseState, WheelState
from sensor_msgs.msg import Imu


def test_yaw_deadband_is_applied_to_all_feedback_topics():
    os.environ["ROS_DOMAIN_ID"] = str(195 + os.getpid() % 20)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("yaw_feedback_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = {}
    for key, message_type, topic, qos in (
            ("state", BaseState, "/base/state", 10),
            ("wheel", WheelState, "/base/wheel_state", qos_profile_sensor_data),
            ("imu", Imu, "/base/imu", qos_profile_sensor_data),
            ("odom", Odometry, "/base/odom", qos_profile_sensor_data)):
        node.create_subscription(message_type, topic,
                                 lambda msg, key=key: received.__setitem__(key, msg), qos)
    master, slave = pty.openpty()
    configured_binary = os.environ.get("MANGO_TEST_BASE_SERIAL", "")
    binary = next((candidate for candidate in configured_binary.split(os.pathsep) if candidate), str(
        Path(get_package_prefix("robot_chassis")) / "lib/robot_chassis/base_serial_node")
    )
    log = tempfile.TemporaryFile()
    startup_delay_s = 1.0
    process = subprocess.Popen([
        binary, "--ros-args", "-p", "serial_device:=" + os.ttyname(slave),
        "-p", "dry_run:=false", "-p", "allow_motion:=false",
        "-p", "imu_yaw_increment_deadband_rad:=0.001",
        "-p", "imu_yaw_increment_deadband_startup_delay_s:=" + str(startup_delay_s),
        "-p", "feedback_timeout_s:=2.0"],
        stdout=log, stderr=log)

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            assert process.poll() is None, "fake-serial node exited"

    def feed(sequence, yaw_mdeg):
        os.write(master, f"FB {sequence} 50 0 0 0 0 {yaw_mdeg}\r\n".encode())

    def assert_topics(expected_yaw, expected_rate):
        assert set(received) == {"state", "wheel", "imu", "odom"}
        imu = received["imu"]
        odom = received["odom"]
        for actual in (received["state"].yaw_rad, received["wheel"].yaw_rad,
                       2 * math.atan2(imu.orientation.z, imu.orientation.w),
                       2 * math.atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)):
            assert abs(actual - expected_yaw) < 1e-9
        for actual in (received["state"].wz_rad_s, imu.angular_velocity.z, odom.twist.twist.angular.z):
            assert abs(actual - expected_rate) < 1e-9

    try:
        deadline = time.monotonic() + 5
        while "state" not in received:
            pump(.02)
            assert time.monotonic() < deadline, "base node never published state"
        feed(0, 0)
        pump(.05)
        for sequence in range(1, 11):
            feed(sequence, sequence * 20)
            pump(.015)
        pump(.05)
        # Small increments pass during the startup grace period.
        warmup_yaw = math.radians(.2)
        assert_topics(warmup_yaw, math.radians(.02) / .05)
        # Leave margin for process initialization before starting the timer.
        pump(startup_delay_s + .2)
        feed(11, 220)
        pump(.1)
        # The same small increments are rejected after the startup grace period.
        assert_topics(warmup_yaw, 0.)
        feed(12, 420)
        pump(.1)
        accepted = warmup_yaw + math.radians(.2)
        assert_topics(accepted, math.radians(.2) / .05)
        feed(13, 400)
        pump(.1)
        assert_topics(accepted, 0.)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        os.close(master)
        os.close(slave)
        log.close()
