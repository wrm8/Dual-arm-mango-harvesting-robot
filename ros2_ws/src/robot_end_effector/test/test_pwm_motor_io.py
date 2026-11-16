"""Test real GPIO sequencing against an in-memory GPIO object, never real pins."""
import pytest
from robot_end_effector.pwm_io import At8236MotorIO, At8236Pins, MotorIO, MotorPins


class FakeGPIO:
    BOARD, OUT, LOW, HIGH = 'BOARD', 'OUT', 0, 1

    def __init__(self):
        self.events, self.levels, self.duties = [], {}, {}
        self.fail_pwm = None

    def setmode(self, mode):
        assert mode == self.BOARD

    def setup(self, pin, mode, initial):
        self.output(pin, initial)

    def output(self, pin, value):
        self.levels[pin] = value
        self.events.append((pin, value))

    def cleanup(self, pins):
        self.events.append(('cleanup', tuple(pins)))

    def PWM(self, pin, frequency):
        outer = self
        class PWM:
            def start(self, duty):
                self.ChangeDutyCycle(duty)

            def ChangeDutyCycle(self, duty):
                if outer.fail_pwm == pin and duty > 0:
                    raise RuntimeError('PWM write failed')
                outer.duties[pin] = duty
                outer.events.append(('PWM', pin, duty))

            def stop(self):
                outer.duties[pin] = 0
        return PWM()


def make_io(gpio):
    return MotorIO({'left': MotorPins(32, 11, 13), 'right': MotorPins(33, 16, 18, -1)}, 7, gpio=gpio)


def test_two_channels_are_disabled_at_start_and_selected_independently():
    gpio = FakeGPIO()
    io = make_io(gpio)
    assert gpio.events[0] == (7, 0)
    assert gpio.levels[7] == 0 and gpio.duties == {32: 0, 33: 0}
    io.start(['left'], .5)
    assert gpio.duties == {32: 50, 33: 0}
    assert gpio.levels[11] == 1 and gpio.levels[13] == 0
    assert gpio.events[-1] == (7, 1)  # STBY raised only after direction/PWM setup.
    io.start(['left', 'right'], 1.0)
    assert gpio.duties == {32: 100, 33: 100}
    assert gpio.levels[16] == 0 and gpio.levels[18] == 1
    io.stop()
    assert gpio.levels[7] == 0 and gpio.duties == {32: 0, 33: 0}
    io.close()


def test_partial_output_failure_stops_both_channels():
    gpio = FakeGPIO()
    io = make_io(gpio)
    gpio.fail_pwm = 33
    with pytest.raises(RuntimeError, match='PWM write failed'):
        io.start(['left', 'right'], 1.0)
    assert gpio.levels[7] == 0 and gpio.duties == {32: 0, 33: 0}
    io.close()


@pytest.mark.parametrize('arms,duty', [([], .5), (['left', 'left'], .5),
                                    (['bad'], .5), (['left'], 0), (['left'], 1.1),
                                    (['left'], float('nan'))])
def test_invalid_commands_never_enable_outputs(arms, duty):
    gpio = FakeGPIO()
    io = make_io(gpio)
    with pytest.raises(ValueError):
        io.start(arms, duty)
    assert gpio.levels[7] == 0
    io.close()


def test_pin_conflicts_are_rejected_before_any_gpio_access():
    gpio = FakeGPIO()
    with pytest.raises(ValueError):
        MotorIO({'left': MotorPins(32, 11, 13), 'right': MotorPins(32, 16, 18)}, 7, gpio=gpio)
    assert gpio.events == []


def test_at8236_uses_pwm_on_in1_and_keeps_in2_low():
    gpio = FakeGPIO()
    io = At8236MotorIO({'left': At8236Pins(32, 11), 'right': At8236Pins(33, 16)}, gpio=gpio)
    assert gpio.duties == {32: 0, 33: 0}
    io.start(['right'], .35)
    assert gpio.duties == {32: 0, 33: 35}
    assert gpio.levels[11] == gpio.levels[16] == 0
    io.start(['left', 'right'], 1.0)
    assert gpio.duties == {32: 100, 33: 100}
    assert set(gpio.levels) == {11, 16, 32, 33}  # No TB6612 STBY/direction pins.
    io.stop()
    assert gpio.duties == {32: 0, 33: 0}
    io.close()


def test_at8236_partial_start_failure_disables_both_pwm_channels():
    gpio = FakeGPIO()
    io = At8236MotorIO({'left': At8236Pins(32, 11), 'right': At8236Pins(33, 16)}, gpio=gpio)
    gpio.fail_pwm = 33
    with pytest.raises(RuntimeError, match='PWM write failed'):
        io.start(['left', 'right'], .5)
    assert gpio.duties == {32: 0, 33: 0}
    io.close()
