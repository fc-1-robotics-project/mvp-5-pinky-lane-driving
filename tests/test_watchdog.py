# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Robot-local monotonic command lease tests, with fake timestamps."""

import math
import unittest

from pinky_lane_driving.watchdog import Watchdog


class WatchdogTest(unittest.TestCase):
    def make(self, enabled=True):
        return Watchdog(timeout=.2, max_source_age=.3, max_speed=.2,
                        max_omega=.8, enabled=enabled)

    def test_default_off_startup_and_expiry(self):
        off = self.make(False)
        off.receive(1, .1, .2, now=1., source_age=.01)
        self.assertEqual(off.output(1.01), (0., 0.))
        dog = self.make()
        self.assertEqual(dog.output(0.), (0., 0.))
        self.assertTrue(dog.receive(1, .1, .2, now=1., source_age=.01))
        self.assertEqual(dog.output(1.1), (.1, .2))
        self.assertEqual(dog.output(1.21), (0., 0.))

    def test_source_age_reduces_lease(self):
        dog = self.make()
        dog.receive(1, .1, .2, now=1., source_age=.25)
        self.assertEqual(dog.output(1.04), (.1, .2))
        self.assertEqual(dog.output(1.06), (0., 0.))

    def test_slower_image_profile_does_not_extend_command_silence_lease(self):
        dog=Watchdog(timeout=.2,max_source_age=.45,max_speed=.2,max_omega=.8,enabled=True)
        self.assertTrue(dog.receive(1,.03,.1,now=1.,source_age=.35))
        self.assertEqual(dog.output(1.09),(.03,.1))
        self.assertEqual(dog.output(1.11),(0.,0.))
        self.assertTrue(dog.receive(2,.03,.1,now=2.,source_age=.1))
        self.assertEqual(dog.output(2.21),(0.,0.))

    def test_six_cm_profile_stops_on_age_or_silence(self):
        dog=Watchdog(timeout=.2,max_source_age=1.1,max_speed=.06,max_omega=.6,enabled=True)
        self.assertTrue(dog.receive(1,.06,.1,now=1.,source_age=.9))
        self.assertEqual(dog.output(1.16),(.06,.1))
        self.assertEqual(dog.output(1.21),(0.,0.))
        self.assertTrue(dog.receive(2,.06,.1,now=2.,source_age=1.05))
        self.assertEqual(dog.output(2.06),(0.,0.))
        self.assertFalse(dog.receive(3,.061,.1,now=3.,source_age=.1))

    def test_duplicate_or_out_of_order_cannot_refresh_lease(self):
        dog = self.make()
        dog.receive(3, .1, .2, now=1., source_age=.01)
        self.assertFalse(dog.receive(2, .1, .2, now=1.1, source_age=.01))
        self.assertEqual(dog.output(1.11), (0., 0.))
        self.assertFalse(dog.receive(3, .1, .2, now=1.12, source_age=.01))

    def test_invalid_and_out_of_bound_inputs_stop(self):
        for override in [dict(speed=math.nan), dict(omega=math.inf),
                         dict(speed=.3), dict(speed=-.1), dict(omega=.9),
                         dict(source_age=-.01), dict(source_age=.31),
                         dict(now=math.nan), dict(sequence=True)]:
            dog = self.make()
            dog.receive(1, .1, .2, now=1., source_age=.01)
            args = dict(sequence=2, speed=.1, omega=.2, now=1.1, source_age=.01)
            args.update(override)
            self.assertFalse(dog.receive(**args))
            self.assertEqual(dog.output(1.15), (0., 0.))

    def test_clock_rewind_and_explicit_stop_revoke(self):
        dog = self.make()
        dog.receive(1, .1, .2, now=1., source_age=.01)
        self.assertEqual(dog.output(.9), (0., 0.))
        self.assertEqual(dog.output(1.1), (0., 0.))
        dog.receive(2, .1, .2, now=1.2, source_age=.01)
        dog.stop()
        self.assertEqual(dog.output(1.21), (0., 0.))
