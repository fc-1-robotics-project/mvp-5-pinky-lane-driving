# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Crosswalk overlap/grouping and positive odometry passage evidence."""

import math
import unittest

from pinky_lane_driving.crosswalk import CrosswalkTracker, outline, path_interval


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

    def test_long_approach_keeps_bounded_outline_and_still_detects_passage(self):
        # Runtime flow: each observation is re-projected at every 20 Hz tick
        # with odometry compensation, and every new detection jitters by a
        # centimetre or two. Neither may grow the remembered outline.
        dense = tuple((.8 + .05 * math.cos(a / 50), .25 * math.sin(a / 50))
                      for a in range(314))

        def seen_from(pose, points):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            return tuple((c * (x - pose[0]) + s * (y - pose[1]),
                          -s * (x - pose[0]) + c * (y - pose[1])) for x, y in points)

        sizes = []
        for tick in range(400):
            pose = (tick * .001, tick * .00005, tick * .0003)
            jitter = .015 * math.sin(tick // 8)      # new detection every 8 ticks
            polygon = tuple((x + jitter, y - jitter) for x, y in dense)
            observed, passed = self.tracker.update([seen_from(pose, polygon)], self.path, pose)
            self.assertEqual(observed[0], 'crosswalk-1')
            self.assertIsNone(passed)
            sizes.append(len(self.tracker.world_points))
        self.assertLess(max(sizes), 80)
        _, passed = self.tracker.update([], self.path, (1.3, 0., 0.))
        self.assertEqual(passed, 'crosswalk-1')

    def test_outline_bounds_dense_convex_contour_and_keeps_extent(self):
        dense = tuple((.8 + .05 * math.cos(a / 500), .25 * math.sin(a / 500))
                      for a in range(3142))
        hull = outline(dense)
        self.assertLess(len(hull), 80)
        for axis, pick in ((0, max), (0, min), (1, max), (1, min)):
            self.assertAlmostEqual(pick(p[axis] for p in hull),
                                   pick(p[axis] for p in dense), delta=.005)
        self.assertEqual(outline(()), ())

    def test_degenerate_outline_does_not_divide_by_zero(self):
        speck = ((.5, 0.), (.5004, 0.), (.5004, .0004))
        observed, _ = self.tracker.update([speck], self.path, (0., 0., 0.))
        self.assertEqual(observed[0], 'crosswalk-1')
        self.tracker.update([speck], self.path, (.001, 0., 0.))

    def test_missing_pose_is_invalid_not_clear(self):
        with self.assertRaises(ValueError):
            self.tracker.update([stripe(.5)], self.path, None)
