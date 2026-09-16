# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Crosswalk overlap/grouping and positive odometry passage evidence."""

import math
import unittest

from pinky_lane_driving.crosswalk import CrosswalkTracker, path_interval


def stripe(x, y=0.):
    return ((x, y - .3), (x + .1, y - .3), (x + .1, y + .3), (x, y + .3))


class CrosswalkTest(unittest.TestCase):
    def setUp(self):
        self.path = ((0., 0.), (2., 0.))
        self.tracker = CrosswalkTracker(group_gap=.2, association_distance=.4, passed_margin=.15)

    def test_off_lane_polygon_ignored_and_entry_distance_exact(self):
        self.assertIsNone(path_interval(stripe(.5, 1.), self.path))
        start, end = path_interval(stripe(.5), self.path)
        self.assertAlmostEqual(start, .5)
        self.assertAlmostEqual(end, .6)

    def test_curved_path_uses_arc_not_straight_line_distance(self):
        polygon = ((.4, .5), (.6, .5), (.6, .6), (.4, .6))
        self.assertAlmostEqual(path_interval(polygon, ((0., 0.), (.5, 0.), (.5, 1.)))[0], 1.)

    def test_multiple_stripes_one_event_and_disappearance_not_passage(self):
        first, passed = self.tracker.update([stripe(.5), stripe(.75)], self.path, (0., 0., 0.))
        self.assertIsNone(passed)
        self.assertAlmostEqual(first[1], .5)
        repeated, _ = self.tracker.update([stripe(.4), stripe(.65)], self.path, (.1, 0., 0.))
        self.assertEqual(first[0], repeated[0])
        observed, passed = self.tracker.update([], self.path, (.1, 0., 0.))
        self.assertIsNone(observed)
        self.assertIsNone(passed)
        _, passed = self.tracker.update([], self.path, (1.1, 0., 0.))
        self.assertEqual(passed, first[0])

    def test_turning_in_place_is_not_positive_passage(self):
        self.tracker.update([stripe(.5)], self.path, (0., 0., 0.))
        _, passed = self.tracker.update([], self.path, (0., 0., math.pi))
        self.assertIsNone(passed)

    def test_next_separate_crosswalk_gets_new_identity(self):
        first, _ = self.tracker.update([stripe(.5), stripe(1.5)], self.path, (0., 0., 0.))
        _, passed = self.tracker.update([], self.path, (.9, 0., 0.))
        self.assertEqual(first[0], passed)
        second, _ = self.tracker.update([stripe(.6)], self.path, (.9, 0., 0.))
        self.assertNotEqual(first[0], second[0])

    def test_missing_pose_is_invalid_not_clear(self):
        with self.assertRaises(ValueError):
            self.tracker.update([stripe(.5)], self.path, None)
