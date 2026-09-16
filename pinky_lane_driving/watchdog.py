# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Robot-local command lease; transport adapter must call output on its own timer."""

import math


class Watchdog:
    """Accept ordered bounded commands, stop on local timeout or clock faults.

    now is receiver-local monotonic time. source_age is independently validated
    capture age (requires synchronized source/receiver ROS clocks in the adapter).
    Sequence numbers are session-monotonic. Publisher restart requires an explicit
    stopped session restart, not silently accepting a reset counter.
    This cannot protect against its own process dying: motor firmware/driver must
    independently timeout. Defaults to disabled and never accesses an actuator.
    """

    def __init__(self, *, timeout, max_source_age, max_speed, max_omega, enabled=False):
        if any(not math.isfinite(v) or v <= 0
               for v in (timeout, max_source_age, max_speed, max_omega)):
            raise ValueError('Watchdog limits must be finite and positive')
        if type(enabled) is not bool:
            raise ValueError('enabled must be boolean')
        self.timeout = timeout
        self.max_source_age = max_source_age
        self.max_speed = max_speed
        self.max_omega = max_omega
        self.enabled = enabled
        self.sequence = -1
        self.last_time = None
        self.deadline = None
        self.velocity = (0., 0.)

    def stop(self):
        self.deadline = None
        self.velocity = (0., 0.)

    def _time_ok(self, now):
        if (not math.isfinite(now) or now < 0
                or self.last_time is not None and now < self.last_time):
            self.stop()
            return False
        self.last_time = now
        return True

    def receive(self, sequence, speed, omega, *, now, source_age):
        if (not self._time_ok(now) or type(sequence) is not int or sequence <= self.sequence
                or not all(math.isfinite(v) for v in (speed, omega, source_age))
                or not 0 <= speed <= self.max_speed or abs(omega) > self.max_omega
                or not 0 <= source_age < self.max_source_age):
            self.stop()
            return False
        self.sequence = sequence
        self.deadline = now + min(self.timeout, self.max_source_age - source_age)
        self.velocity = (speed, omega)
        return True

    def output(self, now):
        if (not self._time_ok(now) or not self.enabled or self.deadline is None
                or now >= self.deadline):
            self.stop()
        return self.velocity
