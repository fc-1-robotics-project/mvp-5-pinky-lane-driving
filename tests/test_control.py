# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Synthetic metric paths only; limits here are test inputs, not hardware tuning."""

from dataclasses import replace
import math
import unittest

from pinky_lane_driving.control import Limits, command


class ControlTest(unittest.TestCase):
    def setUp(self):
        self.limits = Limits(max_speed=.3, max_omega=.8, max_accel=.2,
                             max_lateral_accel=.1, braking_decel=.3,
                             latency=.1, stop_margin=.1, timeout=.2,
                             min_lookahead=.2, max_lookahead=.5, lookahead_time=1.)

    def run_control(self, path, **changes):
        values = dict(age=.01, dt=.1, previous_speed=.1, requested_speed=.2,
                      metric_valid=True, limits=self.limits)
        values.update(changes)
        return command(path, **values)

    def test_straight_and_signed_turns(self):
        self.assertEqual(self.run_control([(0., 0.), (2., 0.)]).omega, 0.)
        self.assertGreater(self.run_control([(0., 0.), (1., .3)]).omega, 0.)
        self.assertLess(self.run_control([(0., 0.), (1., -.3)]).omega, 0.)

    def test_steering_gain_preserves_target_and_increases_both_turn_directions(self):
        self.assertEqual(self.limits.steering_gain, 1.)
        for side in (1., -1.):
            with self.subTest(side=side):
                path = [(0., 0.), (1., side * .3)]
                original = self.run_control(path)
                boosted = self.run_control(path, limits=replace(self.limits, steering_gain=1.2))
                self.assertEqual(boosted.target, original.target)
                self.assertAlmostEqual(boosted.curvature, 1.2 * original.curvature)
                self.assertGreater(side * boosted.omega, side * original.omega)
                self.assertAlmostEqual(boosted.omega, boosted.speed * boosted.curvature)

    def test_steering_gain_keeps_far_preview_when_near_point_has_opposite_sign(self):
        path = [(.20, -.007), (.23, -.008), (.26, -.005), (.29, .002), (.33, .011)]
        original = self.run_control(path)
        boosted = self.run_control(path, limits=replace(self.limits, steering_gain=1.2))
        self.assertGreater(original.target[1], 0.)
        self.assertEqual(boosted.target, original.target)
        self.assertGreater(boosted.omega, original.omega)

    def test_steering_gain_respects_angular_and_lateral_limits(self):
        path = [(0., 0.), (.3, .3), (.6, .6)]
        boosted = self.run_control(path, limits=replace(self.limits, steering_gain=2.))
        self.assertLessEqual(abs(boosted.omega), self.limits.max_omega + 1e-12)
        self.assertLessEqual(abs(boosted.speed * boosted.omega),
                             self.limits.max_lateral_accel + 1e-12)
        lateral_limited = self.run_control(
            path, previous_speed=.4, requested_speed=.4,
            limits=replace(self.limits, max_speed=.4, max_omega=10., steering_gain=2.))
        self.assertLessEqual(abs(lateral_limited.speed * lateral_limited.omega),
                             self.limits.max_lateral_accel + 1e-12)

    def test_steering_gain_configuration_bounds(self):
        for gain in (0., -1., 2.001, math.inf, math.nan):
            with self.subTest(gain=gain), self.assertRaises(ValueError):
                replace(self.limits, steering_gain=gain)

    def test_stop_on_invalid_stale_or_unmeasured_path(self):
        for path, changes in [([], {}), ([(0., 0.)], {}),
                              ([(0., 0.), (0., 0.)], {}),
                              ([(math.nan, 0.), (1., 0.)], {}),
                              ([(0., 0.), (-1., 0.)], {}),
                              ([(0., 0.), (1., 0.)], {'age': .21}),
                              ([(0., 0.), (1., 0.)], {'age': -.01}),
                              ([(0., 0.), (1., 0.)], {'metric_valid': False}),
                              ([(0., 0.), (1., 0.)], {'dt': 0.})]:
            with self.subTest(path=path, changes=changes):
                result = self.run_control(path, **changes)
                self.assertEqual((result.speed, result.omega), (0., 0.))
                self.assertNotEqual(result.reason, 'tracking')

    def test_speed_acceleration_angular_and_lateral_bounds(self):
        result = self.run_control([(0., 0.), (.3, .3), (.6, .6)])
        self.assertLessEqual(result.speed, .1 + .2 * .1)
        self.assertLessEqual(abs(result.omega), .8)
        self.assertLessEqual(abs(result.speed * result.omega), .1)
        self.assertAlmostEqual(result.omega, result.speed * result.curvature)

    def test_visible_path_stopping_distance(self):
        result = self.run_control([(0., 0.), (.15, 0.)], previous_speed=.3)
        stopping = (result.speed * self.limits.latency
                    + result.speed**2 / (2 * self.limits.braking_decel)
                    + self.limits.stop_margin)
        self.assertLessEqual(stopping, .15 + 1e-12)
        self.assertEqual(self.run_control([(0., 0.), (.05, 0.)]).speed, 0.)

    def test_zero_requested_speed_stops_without_in_place_rotation(self):
        result = self.run_control([(0., 0.), (.5, .5)], requested_speed=0.)
        self.assertEqual((result.speed, result.omega), (0., 0.))

    def test_lookahead_retains_polyline_bend(self):
        result = self.run_control([(0., 0.), (.1, 0.), (.1, .8)])
        self.assertAlmostEqual(result.target[0], .1)
        self.assertGreater(result.target[1], 0.)

    def test_limits_and_nonfinite_runtime_inputs(self):
        for field in self.limits.__dataclass_fields__:
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    replace(self.limits, **{field: math.nan})
        with self.assertRaises(ValueError):
            replace(self.limits, braking_decel=0.)
        with self.assertRaises(ValueError):
            replace(self.limits, min_lookahead=1.)
        for field in ['age', 'dt', 'previous_speed', 'requested_speed']:
            self.assertEqual(self.run_control([(0., 0.), (1., 0.)],
                                             **{field: math.nan}).speed, 0.)
