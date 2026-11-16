"""One-way STM32 strike trigger followed by a local, configurable return delay."""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_arm_interfaces.action import TriggerStrike

from .serial_striker_io import SerialStrikerIO, STRIKE_BAUD_RATE, STRIKE_COMMAND_HEX


class SerialStrikerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('serial_striker_node', **kwargs)
        dry_run = self.declare_parameter('dry_run', True).value
        allow_motion = self.declare_parameter('allow_motion', False).value
        device = self.declare_parameter('serial_device', '/dev/stm32_striker').value
        chassis_device = self.declare_parameter(
            'chassis_serial_device', '/dev/stm32_chassis').value
        baud_rate = self.declare_parameter('baud_rate', STRIKE_BAUD_RATE).value
        write_timeout = self.declare_parameter('write_timeout_s', 0.5).value
        self.max_wait = self.declare_parameter('max_wait_s', 30.0).value
        if not math.isfinite(self.max_wait) or not 0 < self.max_wait <= 60:
            raise ValueError('max_wait_s must be in (0, 60]')
        frames = {key: self.declare_parameter(key + '_command_hex', STRIKE_COMMAND_HEX[key]).value
                  for key in ('left', 'right', 'both')}
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.busy = False
        self.io = None
        # Opening the port and validating all frames precedes advertising the action.
        if allow_motion and not dry_run:
            self.io = SerialStrikerIO(device, baud_rate, frames, chassis_device, write_timeout)
        self.server = ActionServer(
            self, TriggerStrike, '/end_effector/trigger_strike', self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(
            f'STM32 striker ready: serial={device}, baud={baud_rate}, '
            f'transmit_enabled={self.io is not None}; no MCU feedback; no trigger sent at startup')

    def accept(self, goal):
        if (not goal.arms or len(set(goal.arms)) != len(goal.arms) or
                not set(goal.arms) <= {'left', 'right'} or
                not math.isfinite(goal.wait_s) or not 0 <= goal.wait_s <= self.max_wait):
            return GoalResponse.REJECT
        with self.lock:
            if self.busy or self.closed.is_set() or self.io is None:
                self.get_logger().warning('Strike rejected: busy, closed, or motion disabled')
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def cancel(self, _handle):
        self.get_logger().warning('Cancel stops the local wait only; STM32 may continue its strike')
        return CancelResponse.ACCEPT

    def execute(self, handle):
        result = TriggerStrike.Result()
        try:
            with self.lock:
                if self.closed.is_set():
                    raise RuntimeError('node closing; no trigger sent')
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = 'canceled before trigger'
                    return result
                frame = self.io.trigger(handle.request.arms)
            started = time.monotonic()
            self.get_logger().info(
                f'TX arms={list(handle.request.arms)} hex={frame.hex(" ")}; '
                f'local wait={handle.request.wait_s:.3f}s; MCU execution unverified')
            next_feedback = 0.0
            while True:
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = 'local wait canceled; trigger already sent, STM32 may still run'
                    return result
                if self.closed.is_set() or not rclpy.ok(context=self.context):
                    raise RuntimeError('local wait interrupted; STM32 may still run')
                elapsed = time.monotonic() - started
                if elapsed >= handle.request.wait_s:
                    break
                if elapsed >= next_feedback:
                    handle.publish_feedback(TriggerStrike.Feedback(elapsed_s=elapsed))
                    next_feedback = elapsed + 0.1
                self.closed.wait(min(0.01, handle.request.wait_s - elapsed))
            result.success = True
            result.message = 'trigger written; local wait elapsed; MCU completion is not verified'
            handle.succeed()
        except Exception as error:
            result.message = f'strike failed: {error}; trigger not retried; MCU state unverified'
            self.get_logger().error(result.message)
            handle.abort()
        finally:
            with self.lock:
                self.busy = False
        return result

    def close(self):
        self.closed.set()
        with self.lock:
            if self.io is not None:
                self.io.close()
                self.io = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = SerialStrikerNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
