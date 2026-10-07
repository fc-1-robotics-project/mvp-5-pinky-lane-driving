# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Deterministic fake-clock scenarios; never sleep or command motors."""

import unittest

from pinky_lane_driving.behavior import Behavior, Settings, arbitrate
from pinky_lane_driving.control import Proposal


class BehaviorTest(unittest.TestCase):
    def setUp(self):
        self.fsm = Behavior(Settings(cruise_speed=.2, approach_speed=.1,
                                     stop_distance=.15, stopped_speed=.01,
                                     wait_s=2., clear_s=.5, decel=.3, latency=.1))

    def step(self, now, **kwargs):
        args = dict(sensors_ok=True, estop=False, obstacle=False,
                    crosswalk=None, passed_id=None, measured_speed=0.)
        args.update(kwargs)
        return self.fsm.step(now, **args)

    def ready(self):
        self.step(0.)
        return self.step(.5)

    def test_startup_requires_stable_obstacle_clear(self):
        self.assertEqual(self.step(0.).speed_limit, 0.)
        self.assertGreater(self.step(.5).speed_limit, 0.)

    def test_wait_requires_measured_stop_then_pass_latches(self):
        self.ready()
        self.assertEqual(self.step(1., crosswalk=('a', .7)).state, 'APPROACH')
        decision = self.step(2., crosswalk=('a', .1), measured_speed=.1)
        self.assertEqual(decision.speed_limit, 0.)
        self.assertEqual(decision.state, 'APPROACH')
        self.assertEqual(self.step(3., crosswalk=('a', .1)).state, 'WAIT')
        self.assertEqual(self.step(4., crosswalk=('a', .1)).speed_limit, 0.)
        self.assertEqual(self.step(5., crosswalk=('a', .1)).state, 'PASS')
        self.assertGreater(self.step(6., crosswalk=('a', .1)).speed_limit, 0.)
        self.assertEqual(self.step(7., passed_id='a').state, 'FOLLOW')
        self.assertEqual(self.step(8., crosswalk=('a', .1)).state, 'FOLLOW')
        self.assertEqual(self.step(9., crosswalk=('b', .7)).state, 'APPROACH')

    def test_slow_only_crosswalk_never_stops_and_resumes_after_passage(self):
        self.fsm = Behavior(Settings(cruise_speed=.09, approach_speed=.06,
                                     stop_distance=.15, stopped_speed=.01,
                                     wait_s=2., clear_s=.5, decel=.15, latency=.2,
                                     crosswalk_stop=False))
        self.assertEqual(self.ready().speed_limit, .09)
        for now, distance in ((1., .5), (2., .2), (3., .05), (4., 0.)):
            decision = self.step(now, crosswalk=('a', distance), measured_speed=.06)
            self.assertEqual((decision.state, decision.speed_limit, decision.reason),
                             ('PASS', .06, 'crosswalk'))
        # Out of the camera view while still on the crosswalk: stay slow.
        self.assertEqual(self.step(5.).speed_limit, .06)
        self.assertEqual(self.step(6., passed_id='a').speed_limit, .09)
        self.assertEqual(self.step(7., crosswalk=('a', .1)).speed_limit, .09)
        with self.assertRaises(ValueError):
            Settings(cruise_speed=.09, approach_speed=.06, stop_distance=.15,
                     stopped_speed=.01, wait_s=2., clear_s=.5, decel=.15, latency=.2,
                     crosswalk_stop=1)

    def test_missing_crosswalk_in_approach_is_not_passed(self):
        self.ready()
        self.step(1., crosswalk=('a', .7))
        result = self.step(2.)
        self.assertEqual(result.state, 'APPROACH')
        self.assertEqual(result.speed_limit, 0.)

    def test_priority_and_stable_clear_restart(self):
        self.ready()
        self.step(1., crosswalk=('a', .1))
        self.assertEqual(self.step(2., obstacle=True).reason, 'obstacle')
        self.assertEqual(self.step(3., obstacle=True, estop=True).reason, 'emergency')
        self.assertEqual(self.step(4., obstacle=None).reason, 'sensor_failure')
        self.assertEqual(self.step(5.).speed_limit, 0.)
        self.assertEqual(self.step(5.5).state, 'WAIT')
        self.assertEqual(self.step(6.).state, 'WAIT')
        self.assertEqual(self.step(7.).state, 'PASS')

    def test_clock_rewind_and_nan_stop(self):
        self.ready()
        self.assertEqual(self.step(.4).speed_limit, 0.)
        self.assertEqual(self.step(float('nan')).speed_limit, 0.)

    def test_invalid_crosswalk_does_not_authorize_motion(self):
        self.ready()
        self.assertEqual(self.step(1., crosswalk=('a', float('nan'))).speed_limit, 0.)

    def test_arbiter_preserves_curvature_and_never_overrides_stop(self):
        decision = self.ready()
        proposal = Proposal(.3, .6, 2., (.3, .1), 'tracking')
        result = arbitrate(proposal, decision)
        self.assertEqual((result.speed, result.omega), (.2, .4))
        result = arbitrate(proposal, self.step(1., obstacle=True))
        self.assertEqual((result.speed, result.omega), (0., 0.))
        self.assertEqual(arbitrate(Proposal(), decision).speed, 0.)
