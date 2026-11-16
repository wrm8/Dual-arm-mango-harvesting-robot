import pytest

from robot_end_effector.encoder import EncoderBank, TurnController


def test_a_rising_edge_counts_one_pulse_and_b_sets_direction():
    class GPIO:
        IN, RISING = 'IN', 'RISING'
        def __init__(self):
            self.callbacks, self.levels = {}, {22: 1, 24: 0}
        def setup(self, pin, mode):
            pass
        def input(self, pin):
            return self.levels[pin]
        def add_event_detect(self, pin, edge, callback):
            self.callbacks[pin] = callback
        def remove_event_detect(self, pin):
            del self.callbacks[pin]
        def cleanup(self, pins):
            pass
    gpio = GPIO()
    bank = EncoderBank({'left': (21, 22, 1), 'right': (23, 24, -1)}, gpio)
    gpio.callbacks[21](21)
    gpio.callbacks[23](23)
    assert bank.snapshot() == {'left': 1, 'right': 1}
    gpio.levels[22] = 0
    gpio.callbacks[21](21)
    assert bank.snapshot()['left'] == 0
    bank.close()
    assert gpio.callbacks == {}


def test_encoder_output_pin_conflict_is_rejected_before_setup():
    with pytest.raises(ValueError):
        EncoderBank({'left': (32, 22, 1)}, object(), [32])


def test_ten_revolutions_slowdown_independent_stop_and_settling():
    control = TurnController({'left': 50, 'right': -10}, {'left': 100., 'right': 100.}, 10., 1., 0.)
    outputs, done = control.update({'left': 50, 'right': -10}, 0.)
    assert outputs == {'left': 1., 'right': 1.} and not done
    outputs, done = control.update({'left': 1030, 'right': 700}, .5)
    assert 0 < outputs['left'] < outputs['right'] and not done
    outputs, done = control.update({'left': 1050, 'right': 970}, .6)
    assert outputs['left'] == 0 and outputs['right'] > 0 and not done
    outputs, done = control.update({'left': 1050, 'right': 990}, .7)
    assert outputs == {'left': 0., 'right': 0.} and not done
    assert control.update({'left': 1050, 'right': 990}, .86)[1]


@pytest.mark.parametrize('count,now,error', [(-5, .1, 'backwards'), (1003, .1, 'overshoot'),
                                           (0, 2.1, 'no pulses')])
def test_counting_faults_never_report_complete(count, now, error):
    control = TurnController({'left': 0}, {'left': 100.}, 10., 1., 0.)
    with pytest.raises(RuntimeError, match=error):
        control.update({'left': count}, now)


def test_uncalibrated_pulse_count_cannot_start_controller():
    with pytest.raises(ValueError, match='uncalibrated'):
        TurnController({'left': 0}, {'left': 0.}, 10., 1., 0.)
