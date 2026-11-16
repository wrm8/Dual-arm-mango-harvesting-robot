"""Continuous or timed PWM control; hardware access is isolated in pwm_io.py."""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_arm_interfaces.action import RunMotors
from std_srvs.srv import Trigger
from std_msgs.msg import Int64MultiArray

from .pwm_io import At8236MotorIO, At8236Pins, MotorIO, MotorPins, SimulatedMotorIO
from .encoder import EncoderBank, TurnController


class PwmMotorNode(Node):
    def __init__(self):
        super().__init__('pwm_motor_node')
        self.dry_run = self.declare_parameter('dry_run', True).value
        self.allow_motion = self.declare_parameter('allow_motion', False).value
        self.simulate = self.declare_parameter('simulate', False).value
        driver_type = self.declare_parameter('driver_type', 'at8236').value
        if driver_type not in ('at8236', 'tb6612'):
            raise ValueError('driver_type must be at8236 or tb6612')
        self.enable_encoders = self.declare_parameter('enable_encoders', True).value
        self.counts_per_rev = {}
        channels = {}
        default_encoder_pins = {'left': (12, 29), 'right': (31, 15)}
        for side, pins in default_encoder_pins.items():
            self.counts_per_rev[side] = self.declare_parameter(side + '.counts_per_output_rev', 0.0).value
            channels[side] = (
                self.declare_parameter(side + '.encoder_a_pin', pins[0]).value,
                self.declare_parameter(side + '.encoder_b_pin', pins[1]).value,
                self.declare_parameter(side + '.encoder_direction_sign', 1).value)
        self.encoder_options = {
            'slowdown_revolutions': self.declare_parameter('encoder_slowdown_revolutions', .5).value,
            'minimum_duty': self.declare_parameter('encoder_minimum_duty', .15).value,
            'tolerance_ticks': self.declare_parameter('encoder_tolerance_ticks', 2).value,
            'settle_s': self.declare_parameter('encoder_settle_s', .15).value,
            'no_pulse_timeout_s': self.declare_parameter('encoder_no_pulse_timeout_s', 2.0).value,
        }
        self.max_duration = self.declare_parameter('max_duration_s', 30.0).value
        if not math.isfinite(self.max_duration) or not 0 < self.max_duration <= 60:
            raise ValueError('max_duration_s must be in (0, 60]')
        standby = self.declare_parameter('standby_pin', 7).value
        frequency = self.declare_parameter('pwm_frequency_hz', 20000).value
        motors = {}
        for side, pins in (('left', (32, 11, 13)), ('right', (33, 16, 18))):
            pwm = self.declare_parameter(side + '.pwm_pin', pins[0]).value
            if driver_type == 'at8236':
                motors[side] = At8236Pins(
                    pwm, self.declare_parameter(side + '.low_pin', pins[1]).value)
            else:
                motors[side] = MotorPins(
                    pwm, self.declare_parameter(side + '.in1_pin', pins[1]).value,
                    self.declare_parameter(side + '.in2_pin', pins[2]).value,
                    self.declare_parameter(side + '.direction', 1).value)
        self.io = None
        self.encoders = None
        if self.simulate:
            self.io = SimulatedMotorIO()
            self.get_logger().warning('SIMULATED motor backend; no physical PWM or motor feedback')
        elif self.allow_motion and not self.dry_run:
            self.io = (At8236MotorIO(motors, frequency) if driver_type == 'at8236'
                       else MotorIO(motors, standby, frequency))
            if self.enable_encoders:
                output_pins = [pin for p in motors.values() for pin in (
                    (p.pwm, p.low) if driver_type == 'at8236' else (p.pwm, p.in1, p.in2))]
                if driver_type == 'tb6612':
                    output_pins.append(standby)
                try:
                    self.encoders = EncoderBank(channels, self.io.gpio, output_pins)
                except Exception:
                    self.io.close()
                    raise
        self.lock = threading.RLock()
        self.busy = False
        self.closed = False
        self.interrupted = threading.Event()
        self.group = ReentrantCallbackGroup()
        self.server = ActionServer(
            self, RunMotors, '/end_effector/run_motors', self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel,
            callback_group=self.group)
        self.stop_service = self.create_service(
            Trigger, '/end_effector/stop', self.stop, callback_group=self.group)
        self.counts_pub = self.create_publisher(Int64MultiArray, '/end_effector/encoder_counts', 10)
        self.counts_timer = self.create_timer(.1, self.publish_counts, callback_group=self.group)
        self.get_logger().info(
            f'PWM motor server ready: driver={driver_type}; outputs off until an accepted action')

    def accept(self, goal):
        if (not goal.arms or len(set(goal.arms)) != len(goal.arms) or
                not set(goal.arms) <= {'left', 'right'} or
                not math.isfinite(goal.duty_cycle) or not 0 < goal.duty_cycle <= 1 or
                not math.isfinite(goal.duration_s) or not 0 <= goal.duration_s <= self.max_duration):
            self.get_logger().error('Rejecting invalid motor arms/duty_cycle/duration_s')
            return GoalResponse.REJECT
        if goal.use_encoder:
            if (not self.encoders or goal.duration_s <= 0 or
                    not math.isfinite(goal.revolutions) or not 0 < goal.revolutions <= 1000 or
                    any(not math.isfinite(self.counts_per_rev[side]) or self.counts_per_rev[side] <= 0
                        for side in goal.arms)):
                self.get_logger().error('Encoder mode requires connected encoders, calibrated counts/output rev, positive turns and timeout')
                return GoalResponse.REJECT
        with self.lock:
            if self.closed or self.busy or self.io is None:
                self.get_logger().error('Motor action unavailable: busy or dry_run/allow_motion disables GPIO')
                return GoalResponse.REJECT
            self.busy = True  # Reserve before the executor starts the action callback.
            self.interrupted.clear()
        return GoalResponse.ACCEPT

    def publish_counts(self):
        with self.lock:
            if self.closed or not self.encoders:
                return
            try:
                counts = self.encoders.snapshot()
                self.counts_pub.publish(Int64MultiArray(data=[counts['left'], counts['right']]))
            except Exception as exc:
                self.interrupted.set()
                try:
                    self.io.stop()
                except Exception as stop_exc:
                    self.get_logger().fatal(f'Failed to stop motor after encoder fault: {stop_exc}')
                self.get_logger().error(str(exc), throttle_duration_sec=5)

    def cancel(self, handle):
        self.interrupted.set()
        return CancelResponse.ACCEPT

    def stop(self, request, response):
        del request
        with self.lock:
            self.interrupted.set()
            try:
                if self.io:
                    self.io.stop()
                response.success, response.message = True, 'Both motor outputs disabled'
            except Exception as exc:
                response.success, response.message = False, str(exc)
        return response

    def execute(self, handle):
        result = RunMotors.Result()
        started = time.monotonic()
        completed = False
        continuous = not handle.request.use_encoder and handle.request.duration_s == 0.0
        controller = None
        try:
            with self.lock:
                if self.interrupted.is_set() or handle.is_cancel_requested:
                    raise RuntimeError('motor operation interrupted before start')
                if handle.request.use_encoder:
                    counts = self.encoders.snapshot()
                    controller = TurnController(
                        {side: counts[side] for side in handle.request.arms},
                        {side: self.counts_per_rev[side] for side in handle.request.arms},
                        handle.request.revolutions, handle.request.duty_cycle, time.monotonic(),
                        **self.encoder_options)
                self.io.start(handle.request.arms, handle.request.duty_cycle)
            operation = (f'target={handle.request.revolutions:.3f} output turns, timeout='
                         if controller else 'duration=')
            self.get_logger().info(
                f'Running {list(handle.request.arms)} at duty={handle.request.duty_cycle:.3f}; '
                f'{operation}{handle.request.duration_s:.3f} s (0=continuous)')
            started = time.monotonic()
            next_feedback = started
            if continuous:
                completed = True
            while not continuous and rclpy.ok(context=self.context):
                if self.interrupted.is_set() or handle.is_cancel_requested:
                    raise RuntimeError('motor operation interrupted; outputs disabled')
                elapsed = time.monotonic() - started
                if controller:
                    outputs, completed = controller.update(self.encoders.snapshot(), time.monotonic())
                    with self.lock:
                        if self.interrupted.is_set():
                            raise RuntimeError('encoder move interrupted')
                        for side, duty in outputs.items():
                            self.io.set_duty(side, duty)
                    if completed:
                        break
                if elapsed >= handle.request.duration_s:
                    if controller:
                        raise RuntimeError('encoder move timed out before reaching and settling at target')
                    completed = True
                    break
                if time.monotonic() >= next_feedback:
                    feedback = RunMotors.Feedback()
                    feedback.elapsed_s = elapsed
                    handle.publish_feedback(feedback)
                    next_feedback += 0.1
                self.interrupted.wait(min(0.005, handle.request.duration_s - elapsed))
            result.message = ('continuous PWM enabled; use stop service to disable outputs' if continuous
                              else 'encoder target reached and settled; outputs disabled' if controller
                              else 'timed PWM operation completed; outputs disabled') if completed else 'ROS shutdown'
            if completed and controller:
                counts = self.encoders.snapshot()
                result.message += '; ' + '; '.join(
                    f'{side} actual_turns={(counts[side] - controller.start[side]) / controller.cpr[side]:.5f}'
                    for side in handle.request.arms)
        except Exception as exc:
            result.message = str(exc)
        finally:
            # Timed success requires output stop; continuous success acknowledges
            # output setup only, and leaves outputs running until an explicit stop.
            with self.lock:
                try:
                    if self.io and (not continuous or not completed or self.interrupted.is_set()
                                    or handle.is_cancel_requested):
                        self.io.stop()
                except Exception as exc:
                    completed = False
                    result.message = 'failed to stop PWM: ' + str(exc)
                result.success = completed and not self.interrupted.is_set()
                if handle.is_cancel_requested:
                    handle.canceled()
                elif result.success:
                    handle.succeed()
                else:
                    handle.abort()
                self.busy = False
        return result

    def close(self):
        with self.lock:
            self.closed = True
            self.interrupted.set()
            if self.io:
                self.io.close()
            if self.encoders:
                self.encoders.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = PwmMotorNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
        executor.shutdown()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
