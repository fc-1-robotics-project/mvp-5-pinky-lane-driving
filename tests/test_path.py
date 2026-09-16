# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Current-lane geometry from metric near-to-far boundary polylines."""

import math
import unittest

from pinky_lane_driving.path import Boundary, PathSettings, select_path


class PathTest(unittest.TestCase):
    def setUp(self):
        self.config = PathSettings(width=.4, width_tolerance=.08, max_near=.6,
                                   sample_step=.05, fallback_timeout=.3,
                                   fallback_speed=.05, ambiguity_margin=.02)
        self.left = Boundary(1, ((.1, .2), (1., .2)))
        self.right = Boundary(2, ((.1, -.2), (1., -.2)))

    def select(self, boundaries, **kwargs):
        return select_path(boundaries, self.config, **kwargs)

    def test_straight_center_and_opposite_lane_distractor(self):
        other = Boundary(1, ((.1, .6), (3., .6)))
        result = self.select([other, self.left, self.right])
        self.assertTrue(result.valid)
        self.assertEqual(result.selected, (1, 2))
        self.assertTrue(all(abs(y) < 1e-9 for _, y in result.points))
        self.assertFalse(result.degraded)

    def test_normal_sections_follow_120_degree_curve(self):
        angles = [math.radians(i * 3) for i in range(41)]
        left = Boundary(1, tuple((.8 * math.sin(a), 1 - .8 * math.cos(a)) for a in angles))
        right = Boundary(2, tuple((1.2 * math.sin(a), 1 - 1.2 * math.cos(a)) for a in angles))
        result = self.select([left, right])
        self.assertTrue(result.valid)
        self.assertGreater(result.points[-1][1], 1.3)
        self.assertTrue(all(abs(math.hypot(x, y - 1) - 1) < .04 for x, y in result.points))

    def test_missing_boundary_normal_offset_is_time_bounded(self):
        result = self.select([self.left], fallback_age=.1)
        self.assertTrue(result.valid)
        self.assertTrue(result.degraded)
        self.assertEqual(result.speed_limit, .05)
        self.assertTrue(all(abs(y) < 1e-9 for _, y in result.points))
        self.assertFalse(self.select([self.left]).valid)
        self.assertFalse(self.select([self.left], fallback_age=.31).valid)
        self.assertFalse(self.select([], fallback_age=.1).valid)

    def test_wrong_width_crossed_or_disconnected_are_invalid(self):
        for right in [Boundary(2, ((.1, -.8), (1., -.8))),
                      Boundary(2, ((.1, .3), (1., .3))),
                      Boundary(2, ((2., -.2), (3., -.2)))]:
            self.assertFalse(self.select([self.left, right]).valid)

    def test_duplicate_candidates_are_ambiguous_not_arbitrary(self):
        self.assertFalse(self.select([self.left, self.left, self.right]).valid)

    def test_previous_path_must_be_in_current_frame(self):
        self.assertTrue(self.select([self.left, self.right],
                                   previous=((.1, 0.), (.5, 0.))).valid)
        self.assertFalse(self.select([self.left, self.right],
                                    previous=((.1, 2.), (.5, 2.))).valid)

    def test_invalid_geometry_and_settings(self):
        with self.assertRaises(ValueError):
            Boundary(1, ((float('nan'), 0.), (1., 0.)))
        with self.assertRaises(ValueError):
            Boundary(0, ((0., 0.), (1., 0.)))
