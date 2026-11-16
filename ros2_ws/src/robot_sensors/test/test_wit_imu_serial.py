"""Exercise the real ROS executable through a PTY; no physical serial or motion."""
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import tempfile
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def angle_frame(roll=0, pitch=0, yaw=0, kind=0x53):
    data = bytes((0x55, kind)) + struct.pack("<hhhH", roll, pitch, yaw, 0)
    return data + bytes((sum(data) & 0xff,))


def test_serial_angles_stale_and_reconnect(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(160 + os.getpid() % 40))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("wit_serial_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = {key: [] for key in ("raw", "deg", "rad", "imu")}
    subscriptions = []
    for key, msg_type, topic in (
        ("raw", Float64, "/imu/wit/yaw_raw_deg"),
        ("deg", Float64, "/imu/wit/yaw_deg"),
        ("rad", Float64, "/imu/wit/yaw_rad"),
        ("imu", Imu, "/imu/wit/data"),
    ):
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, key=key: received[key].append(msg),
            qos_profile_sensor_data))
    binary = next(part for part in os.environ["MANGO_TEST_WIT_IMU"].split(os.pathsep) if part)
    serial_path = tmp_path / "imu_serial"
    master, slave = pty.openpty()
    opened_fds = [master, slave]
    log = tempfile.TemporaryFile()
    process = subprocess.Popen([
        binary, "--ros-args", "-p", "serial_device:=" + str(serial_path),
        "-p", "baud_rate:=9600", "-p", "frame_id:=test_wit_link",
        "-p", "reconnect_interval_s:=0.1", "-p", "stale_timeout_s:=0.15",
        "-p", "zero_yaw_on_start:=true", "-p", "yaw_sign:=-1.0",
        "-p", "yaw_offset_deg:=5.0"], stdout=log, stderr=log)

    def read_log():
        log.seek(0)
        return log.read().decode(errors="replace")

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            assert process.poll() is None, read_log()

    def wait_for(predicate, timeout=6.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            pump(0.02)
            assert time.monotonic() < deadline, read_log()

    def all_counts():
        return [len(messages) for messages in received.values()]

    def assert_yaw(raw, output, roll=0.0, pitch=0.0):
        assert received["raw"][-1].data == pytest.approx(raw)
        assert received["deg"][-1].data == pytest.approx(output)
        assert received["rad"][-1].data == pytest.approx(math.radians(output))
        imu = received["imu"][-1]
        assert imu.header.frame_id == "test_wit_link"
        assert imu.header.stamp.sec > 0
        sr, cr = math.sin(roll / 2), math.cos(roll / 2)
        sp, cp = math.sin(pitch / 2), math.cos(pitch / 2)
        sy, cy = math.sin(math.radians(output) / 2), math.cos(math.radians(output) / 2)
        q = imu.orientation
        assert [q.x, q.y, q.z, q.w] == pytest.approx([
            sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy])
        assert list(imu.orientation_covariance) == [0.0] * 9
        assert imu.angular_velocity_covariance[0] == -1.0
        assert imu.linear_acceleration_covariance[0] == -1.0

    try:
        wait_for(lambda: all(sub.get_publisher_count() for sub in subscriptions))
        pump(0.2)
        assert all_counts() == [0, 0, 0, 0], "missing device must not create zero yaw"
        serial_path.symlink_to(os.ttyname(slave))
        wait_for(lambda: "WIT serial connected" in read_log())

        bad = bytearray(angle_frame(yaw=8192))
        bad[-1] ^= 1
        os.write(master, angle_frame(yaw=8192, kind=0x52) + bad)
        pump(0.25)
        assert all_counts() == [0, 0, 0, 0], "gyro/bad checksum must not publish"

        initial = angle_frame(yaw=4096)  # 22.5 degrees becomes the startup reference.
        os.write(master, initial[:4])
        pump(0.1)
        assert all_counts() == [0, 0, 0, 0], "partial frame must not publish"
        os.write(master, initial[4:])
        wait_for(lambda: all(count == 1 for count in all_counts()))
        assert_yaw(22.5, 5.0)
        first_stamp = received["imu"][-1].header.stamp

        os.write(master, angle_frame(4096, -8192, 16384))
        wait_for(lambda: all(count == 2 for count in all_counts()))
        assert_yaw(90.0, -62.5, math.radians(22.5), math.radians(-45.0))
        latest_stamp = received["imu"][-1].header.stamp
        assert (latest_stamp.sec, latest_stamp.nanosec) > (first_stamp.sec, first_stamp.nanosec)
        pump(0.4)
        assert all_counts() == [2, 2, 2, 2], "silence must not republish old yaw"

        # Leave a partial frame before unplugging; a new connection must discard it.
        os.write(master, initial[:5])
        pump(0.05)
        os.close(master)
        os.close(slave)
        opened_fds.clear()
        serial_path.unlink()
        pump(0.25)
        assert all_counts() == [2, 2, 2, 2]
        master, slave = pty.openpty()
        opened_fds.extend([master, slave])
        serial_path.symlink_to(os.ttyname(slave))
        wait_for(lambda: read_log().count("WIT serial connected") >= 2)
        os.write(master, angle_frame(yaw=-8192))
        wait_for(lambda: all(count == 3 for count in all_counts()))
        # The original 22.5-degree reference survives reconnect; it does not re-zero.
        assert_yaw(-45.0, 72.5)
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
        for fd in opened_fds:
            os.close(fd)
        log.close()
