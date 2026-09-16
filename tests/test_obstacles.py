# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Synthetic laser scans and rigid transforms, in metres/radians."""

import math
import unittest

from pinky_lane_driving.obstacles import scan_collision, transform_points


class ObstacleTest(unittest.TestCase):
    def scan(self, ranges, **overrides):
        args = dict(angle_min=0., angle_increment=.1, range_min=.02, range_max=5.,
                    scan_pose=(0., 0., 0.), path=[(0., 0.), (1., 0.)],
                    radius=.15, age=.01, timeout=.2)
        args.update(overrides)
        return scan_collision(ranges, **args)

    def test_hit_clear_and_footprint_margin(self):
        self.assertTrue(self.scan([.5]))
        self.assertFalse(self.scan([2.]))
        self.assertTrue(self.scan([.5], scan_pose=(0., .14, 0.)))
        self.assertFalse(self.scan([.5], scan_pose=(0., .16, 0.)))

    def test_laser_extrinsic_rotation_and_curved_path(self):
        self.assertFalse(self.scan([.5], scan_pose=(0., 0., math.pi / 2)))
        self.assertTrue(self.scan([.5], scan_pose=(.5, 0., math.pi / 2),
                                  path=[(0., 0.), (.5, 0.), (.5, 1.)]))

    def test_unknown_scan_is_never_clear(self):
        for ranges, overrides in [([], {}), ([math.nan], {}), ([-math.inf], {}),
                                   ([.01], {}), ([6.], {}), ([2.], {'age': .3}),
                                   ([2.], {'age': -.1}), ([2.], {'path': []}),
                                   ([2.], {'scan_pose': None}),
                                   ([2.], {'range_max': .5})]:
            with self.subTest(ranges=ranges, overrides=overrides):
                self.assertIsNone(self.scan(ranges, **overrides))

    def test_infinite_no_return_requires_explicit_driver_contract(self):
        self.assertIsNone(self.scan([math.inf]))
        self.assertFalse(self.scan([math.inf], infinity_is_clear=True))

    def test_rigid_transform_for_current_frame_reuse(self):
        point = transform_points([(1., 0.)], (2., 3., math.pi / 2))[0]
        self.assertAlmostEqual(point[0], 2.)
        self.assertAlmostEqual(point[1], 4.)
        with self.assertRaises(ValueError):
            transform_points([(math.nan, 0.)], (0., 0., 0.))
