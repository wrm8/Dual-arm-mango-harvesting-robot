"""Exercise the real driver on a virtual mixed HX/V3 serial bus, without hardware."""

import errno
import json
import os
import pty
import select
import subprocess
import sys
import tempfile
import threading
import time
import unittest


DRIVER = sys.argv.pop(1)
# Never discover the user's running hardware nodes, even on the Jetson itself.
os.environ["ROS_DOMAIN_ID"] = str(170 + os.getpid() % 25)
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from robot_arm_interfaces.action import MoveJoints  # noqa: E402


NAMES = [f"{side}_{joint}" for side in ("left", "right")
         for joint in ("joint1", "joint2", "joint3", "joint4", "joint5", "gripper")]
V3_IDS = {5, 6, 11, 12}


def frame(header, servo_id, command, payload):
    body = bytes([servo_id, len(payload) + (2 if header == 0xFF else 3), command]) + payload
    return bytes([header, header]) + body + bytes([(~sum(body)) & 0xFF])


def signed_hx(raw):
    return -(raw & 0x7FFF) if raw & 0x8000 else raw


class FakeBus:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.device = os.ttyname(self.slave)
        self.positions = {i: 500 if i in V3_IDS else 2048 for i in range(1, 13)}
        self.records = []
        self.torque_writes = []
        self.unsupported = []
        self.frozen = set()
        self.silent = set()
        self.silent_after_write = set()
        self.missed_reads = {}
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)

    def serve(self):
        pending = bytearray()
        try:
            while not self.stop.is_set():
                ready, _, _ = select.select([self.master], [], [], 0.05)
                if not ready:
                    continue
                try:
                    pending.extend(os.read(self.master, 4096))
                except OSError as exc:
                    if exc.errno == errno.EIO:
                        time.sleep(0.01)
                        continue
                    raise
                while len(pending) >= 4:
                    if pending[:2] not in (b"\xff\xff", b"\x55\x55"):
                        del pending[0]
                        continue
                    size = pending[3] + (4 if pending[0] == 0xFF else 3)
                    if len(pending) < size:
                        break
                    packet = bytes(pending[:size])
                    del pending[:size]
                    if sum(packet[2:]) & 0xFF != 0xFF:
                        raise AssertionError(f"invalid outgoing checksum: {packet.hex()}")
                    self.reply(packet)
        except Exception as exc:
            self.error = exc

    def reply(self, packet):
        header, servo_id, command = packet[0], packet[2], packet[4]
        payload = packet[5:-1]
        if ((header == 0xFF and command == 2) or (header == 0x55 and command == 28)):
            if self.missed_reads.get(servo_id, 0):
                self.missed_reads[servo_id] -= 1
                return
        if header == 0xFF:
            if command in (4, 5, 131):
                # Model units which ACK cached writes but never apply ACTION.
                self.unsupported.append((servo_id, command))
                if servo_id != 254:
                    os.write(self.master, frame(header, servo_id, 0, b""))
            elif command == 2 and servo_id not in self.silent:
                raw = self.positions[servo_id]
                raw = abs(raw) | (0x8000 if raw < 0 else 0)
                os.write(self.master, frame(header, servo_id, 0, raw.to_bytes(2, "little")))
            elif command == 3:
                if payload[0] == 40:
                    self.torque_writes.append((servo_id, payload[1]))
                if payload[0] == 41:
                    target = signed_hx(int.from_bytes(payload[2:4], "little"))
                    speed = int.from_bytes(payload[6:8], "little")
                    self.records.append((servo_id, target, speed, None, time.monotonic()))
                    if servo_id in self.silent_after_write:
                        self.silent.add(servo_id)
                    if servo_id not in self.frozen:
                        self.positions[servo_id] = target
                os.write(self.master, frame(header, servo_id, 0, b""))
        elif command == 28 and servo_id not in self.silent:
            os.write(self.master, frame(header, servo_id, command,
                                       self.positions[servo_id].to_bytes(2, "little")))
        elif command == 1:
            target = int.from_bytes(payload[:2], "little")
            duration = int.from_bytes(payload[2:4], "little")
            self.records.append((servo_id, target, None, duration, time.monotonic()))
            if servo_id in self.silent_after_write:
                self.silent.add(servo_id)
            if servo_id not in self.frozen:
                self.positions[servo_id] = target


class DirectDispatchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bus = FakeBus()
        cls.directory = tempfile.TemporaryDirectory(prefix="hx_direct_dispatch_")
        config_path = os.path.join(cls.directory.name, "driver.json")
        zeros = [cls.bus.positions[i] * 1.0 for i in range(1, 13)]
        config = {"/**": {"ros__parameters": {
            "serial_device": cls.bus.device, "baud_rate": 115200,
            "joint_names": NAMES, "servo_ids": list(range(1, 13)),
            "servo_protocols": ["v3" if i in V3_IDS else "hx" for i in range(1, 13)],
            "zero_ticks": zeros, "urdf_zero_ticks": zeros,
            "ticks_per_radian": [100.0] * 12, "directions": [1.0] * 12,
            "min_ticks": [0] * 12, "max_ticks": [4095] * 12,
            "allow_motion": True, "enable_torque_on_start": False,
            "feedback_poll_period_ms": 0, "feedback_timeout_ms": 20,
            "stationary_tolerance_ticks": 3, "stationary_duration_ms": 150,
            "stationary_min_wait_ms": 100,
        }}}
        with open(config_path, "w", encoding="utf-8") as stream:
            json.dump(config, stream)
        cls.log = open(os.path.join(cls.directory.name, "driver.log"), "w+", encoding="utf-8")
        env = dict(os.environ, ROS_LOG_DIR=cls.directory.name)
        cls.process = subprocess.Popen(
            [DRIVER, "--ros-args", "-r", "__node:=hx_dispatch_test_driver",
             "--params-file", config_path], stdout=cls.log, stderr=subprocess.STDOUT, env=env)
        rclpy.init()
        cls.node = rclpy.create_node("hx_dispatch_test_client")
        cls.client = ActionClient(cls.node, MoveJoints, "/move_joints")
        if not cls.client.wait_for_server(timeout_sec=12):
            cls.log.seek(0)
            raise AssertionError("test driver unavailable:\n" + cls.log.read())

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.process.terminate()
        try:
            cls.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            cls.process.kill()
            cls.process.wait(timeout=5)
        cls.bus.close()
        cls.log.close()
        cls.directory.cleanup()

    def setUp(self):
        self.start = len(self.bus.records)
        self.bus.frozen.clear()
        self.bus.silent.clear()
        self.bus.missed_reads.clear()
        self.bus.silent_after_write.clear()
        self.assertIsNone(self.bus.error)

    def tearDown(self):
        self.bus.frozen.clear()
        self.bus.silent.clear()
        self.assertIsNone(self.bus.error)
        self.assertEqual(self.bus.unsupported, [], "MoveJoints must never rely on REG_WRITE/ACTION")

    def send(self, names, positions, speeds, delays=(), durations=(), timeout_ms=2000,
             expect_accepted=True, stationary=False):
        goal = MoveJoints.Goal()
        goal.joint_names = names
        goal.positions = positions
        goal.speeds = speeds
        goal.start_delays_ms = list(delays)
        goal.durations_ms = list(durations)
        goal.duration.sec, remainder = divmod(timeout_ms, 1000)
        goal.duration.nanosec = remainder * 1000000
        goal.wait_for_completion = True
        goal.complete_when_stationary = stationary
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=4)
        self.assertTrue(sent.done())
        handle = sent.result()
        if not expect_accepted:
            self.assertFalse(handle.accepted)
            return None
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=5)
        self.assertTrue(result.done())
        return result.result().result

    def check_records(self, names, positions, speeds, durations):
        records = self.bus.records[self.start:]
        self.assertEqual(len(records), len(names))
        by_id = {record[0]: record for record in records}
        for name, position, speed, duration in zip(names, positions, speeds, durations):
            servo_id = NAMES.index(name) + 1
            record = by_id[servo_id]
            zero = 500 if servo_id in V3_IDS else 2048
            self.assertEqual(record[1], round(zero + position * 100))
            if servo_id in V3_IDS:
                self.assertEqual(record[3], duration)
            else:
                self.assertEqual(record[2], speed)
        return records

    def test_01_single_joint(self):
        result = self.send([NAMES[0]], [0.3], [50])
        self.assertTrue(result.success, result.message)
        self.check_records([NAMES[0]], [0.3], [50], [2000])

    def test_02_legacy_twelve_joint_command(self):
        positions = [0.3, -0.4, 0.5, -0.6, 0.2, -0.2] * 2
        speeds = [50, 300, 400, 200, 20, 25] * 2
        result = self.send(NAMES, positions, speeds)
        self.assertTrue(result.success, result.message)
        self.check_records(NAMES, positions, speeds, [2000] * 12)

    def test_03_ik_goal_with_per_joint_fields(self):
        names = NAMES[:5]
        positions = [-0.17, 0.66, -1.61, 0.55, 0.03]
        speeds = [50, 300, 300, 300, 20]
        result = self.send(names, positions, speeds, [0] * 5, [1000] * 5)
        self.assertTrue(result.success, result.message)
        self.check_records(names, positions, speeds, [1000] * 5)

    def test_04_both_arms_return_and_delayed_grippers(self):
        positions = [0.0] * 12
        positions[5] = positions[11] = -0.2
        speeds = [50, 300, 300, 300, 20, 20] * 2
        delays = [500 if i in (5, 11) else 0 for i in range(12)]
        result = self.send(NAMES, positions, speeds, delays, [1000] * 12)
        self.assertTrue(result.success, result.message)
        records = self.check_records(NAMES, positions, speeds, [1000] * 12)
        first_arm_time = min(record[4] for record in records if record[0] not in (6, 12))
        for record in records:
            if record[0] in (6, 12):
                self.assertGreaterEqual(record[4] - first_arm_time, 0.45)
                self.assertLess(record[4] - first_arm_time, 1.5)

    def test_05_timeout_identifies_stalled_joint(self):
        self.bus.frozen.add(3)
        result = self.send([NAMES[2]], [1.0], [300], timeout_ms=250)
        self.assertFalse(result.success)
        self.assertIn("position tolerance=20 ticks", result.message)
        self.assertIn("left_joint3 actual=2048 target=2148 error=-100 ticks", result.message)

    def test_06_timeout_identifies_missing_feedback(self):
        self.bus.silent.add(2)
        result = self.send([NAMES[1]], [0.5], [300], timeout_ms=250)
        self.assertFalse(result.success)
        self.assertIn("left_joint2 no fresh feedback", result.message)

    def cancel_move(self, hold_on_cancel):
        self.bus.frozen.add(1)
        goal = MoveJoints.Goal(joint_names=[NAMES[0]], positions=[1.0], speeds=[50],
                               wait_for_completion=True, hold_on_cancel=hold_on_cancel)
        goal.duration.sec = 4
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=3)
        self.assertTrue(sent.done() and sent.result().accepted)
        handle = sent.result()
        deadline = time.monotonic() + 2
        while len(self.bus.records) == self.start and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.01)
        self.assertGreater(len(self.bus.records), self.start)
        cancelled = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancelled, timeout_sec=3)
        terminal = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, terminal, timeout_sec=3)
        self.assertTrue(terminal.done())
        return terminal.result()

    def test_07_automatic_cancel_holds_and_accepts_immediate_home(self):
        actual = self.bus.positions[1]
        torque_start = len(self.bus.torque_writes)
        result = self.cancel_move(True)
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertIn("held for recovery", result.result.message)
        self.assertEqual(self.bus.records[-1][:2], (1, actual))
        self.assertEqual(self.bus.torque_writes[torque_start:], [])
        self.bus.frozen.clear()
        home = self.send([NAMES[0]], [0.0], [50])
        self.assertTrue(home.success, home.message)
        self.assertEqual(self.bus.positions[1], 2048)

    def test_08_legacy_cancel_still_disables_torque(self):
        torque_start = len(self.bus.torque_writes)
        result = self.cancel_move(False)
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertIn("torque disabled", result.result.message)
        self.assertIn((1, 0), self.bus.torque_writes[torque_start:])

    def test_09_transient_offline_joint_is_rechecked_before_dual_arm_home(self):
        self.assertTrue(self.send([NAMES[1]], [0.2], [300]).success)
        self.bus.silent.add(2)
        failed = self.send([NAMES[1]], [0.5], [300], timeout_ms=250)
        self.assertFalse(failed.success)
        self.bus.silent.clear()
        self.bus.missed_reads[2] = 1  # First recheck loses a reply; second succeeds.
        # Removed servo grippers must not block a ten-joint home action.
        self.bus.silent.update({6, 12})
        names = [name for name in NAMES if not name.endswith('gripper')]
        result = self.send(names, [0.0] * 10, [50, 300, 500, 500, 20] * 2)
        self.assertTrue(result.success, result.message)

    def test_10_persistently_offline_joint_rejects_before_any_home_write(self):
        self.assertTrue(self.send([NAMES[7]], [0.2], [300]).success)
        self.bus.silent.add(8)
        failed = self.send([NAMES[7]], [0.5], [300], timeout_ms=250)
        self.assertFalse(failed.success)
        count = len(self.bus.records)
        names = [name for name in NAMES if not name.endswith('gripper')]
        self.send(names, [0.0] * 10, [50, 300, 500, 500, 20] * 2,
                  expect_accepted=False)
        self.assertEqual(len(self.bus.records), count)
        self.log.flush()
        self.log.seek(0)
        self.assertIn('right_joint2 (ID 8)', self.log.read())


    def test_11_stationary_off_target_is_explicit_and_keeps_target_command(self):
        self.assertTrue(self.send([NAMES[0]], [0.0], [50]).success)
        self.bus.frozen.add(1)
        start = time.monotonic()
        result = self.send([NAMES[0]], [1.0], [50], stationary=True)
        self.assertTrue(result.success, result.message)
        self.assertIn('requested joints stationary', result.message)
        self.assertIn('left_joint1=-100 ticks', result.message)
        self.assertGreaterEqual(time.monotonic() - start, .14)
        self.assertEqual(self.bus.records[-1][1], 2148)  # Original target, not actual.
        # The same stationary joint must still fail a strict return-home goal.
        strict = self.send([NAMES[0]], [1.0], [50], timeout_ms=250)
        self.assertFalse(strict.success)

    def test_12_normal_target_completion_still_applies_with_fallback_enabled(self):
        result = self.send([NAMES[1]], [0.1], [50], stationary=True)
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.message, 'HX target reached')

    def test_13_lost_feedback_cannot_complete_stationary_goal(self):
        self.assertTrue(self.send([NAMES[0]], [0.0], [50]).success)
        self.bus.silent_after_write.add(1)
        result = self.send([NAMES[0]], [1.0], [50], timeout_ms=450, stationary=True)
        self.assertFalse(result.success)
        self.assertIn('left_joint1 no fresh feedback', result.message)

    def test_14_v3_stationary_waits_for_travel_time(self):
        self.assertTrue(self.send([NAMES[4]], [0.0], [20]).success)
        self.bus.frozen.add(5)
        start = time.monotonic()
        result = self.send([NAMES[4]], [1.0], [20], durations=[450], stationary=True)
        self.assertTrue(result.success, result.message)
        self.assertGreaterEqual(time.monotonic() - start, .44)
        self.assertEqual(self.bus.records[-1][1:4], (600, None, 450))

    def test_15_delayed_dispatch_and_new_goal_each_need_fresh_stability(self):
        for _ in range(2):
            self.assertTrue(self.send([NAMES[0]], [0.0], [50]).success)
            self.bus.frozen.add(1)
            start = time.monotonic()
            result = self.send([NAMES[0], NAMES[6]], [1.0, 0.0], [50, 50],
                               delays=[0, 250], stationary=True)
            self.assertTrue(result.success, result.message)
            self.assertGreaterEqual(time.monotonic() - start, .39)


if __name__ == "__main__":
    unittest.main(verbosity=2)
