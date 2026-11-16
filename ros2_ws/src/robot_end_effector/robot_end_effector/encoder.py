"""A-rising-edge counting with B used for direction (1x decoding).

Counts per OUTPUT revolution must use this same 1x convention. GPIO callbacks
are not a hardware counter; check for missed counts at the actual motor speed.
"""
import math
import threading


class EncoderBank:
    def __init__(self, channels, gpio, output_pins=()):
        self.gpio, self.channels = gpio, channels
        self.lock = threading.Lock()
        self.counts = dict.fromkeys(channels, 0)
        self.fault = None
        self.configured, self.events = [], []
        pins = [p for a, b, sign in channels.values() for p in (a, b)]
        allowed = {7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26,
                   29, 31, 32, 33, 35, 36, 37, 38, 40}
        if (len(pins) != len(set(pins)) or not set(pins) <= allowed or
                set(pins) & set(output_pins) or
                any(sign not in (-1, 1) for a, b, sign in channels.values())):
            raise ValueError('encoder pins must be unique input GPIOs, separate from motor outputs; sign=+/-1')
        try:
            for pin in pins:
                gpio.setup(pin, gpio.IN)
                self.configured.append(pin)
            for side, (a, b, sign) in channels.items():
                gpio.add_event_detect(a, gpio.RISING, callback=self._callback(side, b, sign))
                self.events.append(a)
        except Exception:
            self.close()
            raise

    def _callback(self, side, b, sign):
        def count(channel):
            del channel
            try:
                delta = sign if self.gpio.input(b) else -sign
                with self.lock:
                    self.counts[side] += delta
            except Exception as exc:
                with self.lock:
                    self.fault = str(exc)
        return count

    def snapshot(self):
        with self.lock:
            if self.fault:
                raise RuntimeError('encoder input failed: ' + self.fault)
            return dict(self.counts)

    def close(self):
        for pin in self.events:
            self.gpio.remove_event_detect(pin)
        self.events.clear()
        if self.configured:
            self.gpio.cleanup(self.configured)
            self.configured.clear()


class TurnController:
    """Forward-only move; stop each side separately, verify settling, fail overshoot."""
    def __init__(self, start_counts, counts_per_rev, revolutions, duty, now,
                 slowdown_revolutions=.5, minimum_duty=.15, tolerance_ticks=2,
                 settle_s=.15, no_pulse_timeout_s=2.0):
        values = [revolutions, duty, slowdown_revolutions, minimum_duty, settle_s, no_pulse_timeout_s]
        if (not all(math.isfinite(v) for v in values) or not 0 < revolutions <= 1000 or
                not 0 < duty <= 1 or slowdown_revolutions <= 0 or
                not 0 < minimum_duty <= 1 or tolerance_ticks < 0 or settle_s <= 0 or
                no_pulse_timeout_s <= 0 or not start_counts or
                set(start_counts) != set(counts_per_rev) or
                any(not math.isfinite(cpr) or cpr <= 0 for cpr in counts_per_rev.values())):
            raise ValueError('invalid encoder control settings or uncalibrated counts per output revolution')
        self.start, self.cpr = start_counts, counts_per_rev
        self.turns, self.duty = revolutions, duty
        self.slowdown, self.minimum = slowdown_revolutions, minimum_duty
        self.tolerance, self.settle = tolerance_ticks, settle_s
        self.no_pulse_timeout = no_pulse_timeout_s
        self.last_counts = dict(start_counts)
        self.last_change = dict.fromkeys(start_counts, now)
        self.settling = dict.fromkeys(start_counts, None)

    def update(self, counts, now):
        outputs = {}
        for side, start in self.start.items():
            value = counts[side]
            progress = value - start
            target = self.turns * self.cpr[side]
            remaining = target - progress
            if value != self.last_counts[side]:
                self.last_change[side] = now
                self.last_counts[side] = value
                self.settling[side] = None
            if progress < -max(4, self.tolerance):
                raise RuntimeError(side + ' encoder counts backwards; calibrate direction_sign')
            if remaining < -self.tolerance:
                raise RuntimeError(f'{side} encoder overshoot: actual={progress} target={target:.2f} ticks')
            if remaining <= self.tolerance:
                outputs[side] = 0.0
                if self.settling[side] is None:
                    self.settling[side] = now
            else:
                self.settling[side] = None
                if now - self.last_change[side] > self.no_pulse_timeout:
                    raise RuntimeError(side + ' encoder has no pulses; check wiring or stalled motor')
                fraction = remaining / (self.slowdown * self.cpr[side])
                outputs[side] = min(self.duty, max(min(self.minimum, self.duty), self.duty * fraction))
        done = all(t is not None and now - t >= self.settle for t in self.settling.values())
        return outputs, done
