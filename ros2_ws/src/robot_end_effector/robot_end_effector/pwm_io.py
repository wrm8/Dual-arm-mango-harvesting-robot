"""Jetson GPIO access is confined to this module. No encoder is used."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MotorPins:
    pwm: int
    in1: int
    in2: int
    direction: int = 1


@dataclass(frozen=True)
class At8236Pins:
    pwm: int
    low: int


class At8236MotorIO:
    """Manual V2 page 9: IN1=PWM, IN2=0, fast-decay forward operation.

    There is no separate enable/PWM pin on this board. To reverse a motor,
    swap its two motor leads (or exchange the IN1/IN2 signal wiring).
    """
    def __init__(self, motors, frequency_hz=20000, gpio=None):
        if set(motors) != {'left', 'right'}:
            raise ValueError('configure both left and right motor channels')
        pins = [pin for p in motors.values() for pin in (p.low, p.pwm)]
        allowed = {7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26,
                   29, 31, 32, 33, 35, 36, 37, 38, 40}
        if len(pins) != len(set(pins)) or not set(pins) <= allowed:
            raise ValueError('BOARD pins must be unique usable GPIO pins')
        if not math.isfinite(frequency_hz) or not 1 <= frequency_hz <= 100000:
            raise ValueError('PWM frequency must be 1..100000 Hz')
        if gpio is None:
            import Jetson.GPIO as gpio
        self.gpio, self.motors = gpio, motors
        self.pwms, self.configured, self.active = {}, [], set()
        try:
            gpio.setmode(gpio.BOARD)
            for pin in pins:
                gpio.setup(pin, gpio.OUT, initial=gpio.LOW)
                self.configured.append(pin)
            for side, p in motors.items():
                self.pwms[side] = gpio.PWM(p.pwm, frequency_hz)
                self.pwms[side].start(0)
        except Exception:
            self.close()
            raise

    def start(self, arms, duty):
        if not arms or len(set(arms)) != len(arms) or not set(arms) <= self.motors.keys():
            raise ValueError('select left, right or both without duplicates')
        if not math.isfinite(duty) or not 0 < duty <= 1:
            raise ValueError('duty_cycle must be in (0, 1]')
        self.stop()
        try:
            for side in arms:
                self.gpio.output(self.motors[side].low, self.gpio.LOW)
                self.pwms[side].ChangeDutyCycle(100.0 * duty)
            self.active = set(arms)
        except Exception:
            self.stop()
            raise

    def stop(self):
        errors = []
        for pwm in self.pwms.values():
            try:
                pwm.ChangeDutyCycle(0)
            except Exception as exc:
                errors.append(str(exc))
        for p in self.motors.values():
            if p.low in self.configured:
                try:
                    self.gpio.output(p.low, self.gpio.LOW)
                except Exception as exc:
                    errors.append(str(exc))
        self.active.clear()
        if errors:
            raise RuntimeError('motor stop failed: ' + '; '.join(errors))

    def set_duty(self, side, duty):
        if side not in self.motors or not math.isfinite(duty) or not 0 <= duty <= 1:
            raise ValueError('invalid motor duty or side')
        self.pwms[side].ChangeDutyCycle(100.0 * duty)
        if duty:
            self.active.add(side)
        else:
            self.active.discard(side)

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        for pwm in self.pwms.values():
            try:
                pwm.stop()
            except Exception:
                pass
        if self.configured:
            self.gpio.cleanup(self.configured)
            self.configured.clear()


class MotorIO:
    """TB6612FNG: dedicated PWM, two direction inputs and shared STBY."""
    def __init__(self, motors, standby_pin, frequency_hz=20000, gpio=None):
        if set(motors) != {'left', 'right'}:
            raise ValueError('configure both left and right motor channels')
        if any(p.direction not in (-1, 1) for p in motors.values()):
            raise ValueError('motor direction must be +1 or -1')
        pins = [standby_pin] + [pin for p in motors.values() for pin in (p.pwm, p.in1, p.in2)]
        # Physical BOARD pins: reject power, ground, I2C-ID, and duplicates.
        allowed = {7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26,
                   29, 31, 32, 33, 35, 36, 37, 38, 40}
        if len(pins) != len(set(pins)) or not set(pins) <= allowed:
            raise ValueError('BOARD pins must be unique usable GPIO pins')
        if not math.isfinite(frequency_hz) or not 1 <= frequency_hz <= 100000:
            raise ValueError('PWM frequency must be 1..100000 Hz')
        if gpio is None:
            import Jetson.GPIO as gpio
        self.gpio, self.motors, self.standby_pin = gpio, motors, standby_pin
        self.pwms, self.configured = {}, []
        self.active = set()
        try:
            gpio.setmode(gpio.BOARD)
            # STBY LOW before any PWM/direction setup; no startup motor pulse.
            for pin in pins:
                gpio.setup(pin, gpio.OUT, initial=gpio.LOW)
                self.configured.append(pin)
            for side, p in motors.items():
                self.pwms[side] = gpio.PWM(p.pwm, frequency_hz)
                self.pwms[side].start(0)
        except Exception:
            self.close()
            raise

    def start(self, arms, duty):
        if not arms or len(set(arms)) != len(arms) or not set(arms) <= self.motors.keys():
            raise ValueError('select left, right or both without duplicates')
        if not math.isfinite(duty) or not 0 < duty <= 1:
            raise ValueError('duty_cycle must be in (0, 1]')
        self.stop()
        try:
            for side in arms:
                pins = self.motors[side]
                forward = pins.direction == 1
                self.gpio.output(pins.in1, self.gpio.HIGH if forward else self.gpio.LOW)
                self.gpio.output(pins.in2, self.gpio.LOW if forward else self.gpio.HIGH)
                self.pwms[side].ChangeDutyCycle(100.0 * duty)
            self.gpio.output(self.standby_pin, self.gpio.HIGH)
            self.active = set(arms)
        except Exception:
            self.stop()
            raise

    def stop(self):
        # STBY is independent of PWM mux and disables BOTH output bridges first.
        errors = []
        if self.standby_pin in self.configured:
            try:
                self.gpio.output(self.standby_pin, self.gpio.LOW)
            except Exception as exc:
                errors.append(str(exc))
        for side, pwm in self.pwms.items():
            try:
                pwm.ChangeDutyCycle(0)
            except Exception as exc:
                errors.append(str(exc))
        self.active.clear()
        if errors:
            raise RuntimeError('motor stop failed: ' + '; '.join(errors))

    def set_duty(self, side, duty):
        if side not in self.motors or not math.isfinite(duty) or not 0 <= duty <= 1:
            raise ValueError('invalid motor duty or side')
        self.pwms[side].ChangeDutyCycle(100.0 * duty)
        if duty:
            self.active.add(side)
        else:
            self.active.discard(side)

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        for pwm in self.pwms.values():
            try:
                pwm.stop()
            except Exception:
                pass
        if self.configured:
            self.gpio.cleanup(self.configured)
            self.configured.clear()


class SimulatedMotorIO:
    """Explicit test-only backend; never imports Jetson.GPIO."""
    def __init__(self):
        self.active = set()

    def start(self, arms, duty):
        self.active = set(arms)

    def stop(self):
        self.active.clear()

    def set_duty(self, side, duty):
        if duty:
            self.active.add(side)
        else:
            self.active.discard(side)

    def close(self):
        self.stop()
