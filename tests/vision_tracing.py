# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Run with the replay Python: unittest discover -s tests -p 'vision_*.py'."""

import unittest

from pinky_lane_driving.tracing import trace_polygon
from pinky_lane_driving.calibration import Calibration
from test_calibration import synthetic_config


class TracingTest(unittest.TestCase):
    def test_metric_overlay_roundtrip_with_distortion(self):
        config = synthetic_config()
        config['distortion'] = [.01, -.001, .0001, .0002, 0.]
        cal = Calibration(config)
        original = [(30., 70.), (70., 30.)]
        projected = cal.project(original)
        restored = cal.image_points(projected)
        for a, b in zip(original, restored):
            self.assertAlmostEqual(a[0], b[0], places=4)
            self.assertAlmostEqual(a[1], b[1], places=4)

    def test_vertical_strip_stays_centered_and_near_to_far(self):
        points = trace_polygon([(40, 10), (60, 10), (60, 95), (40, 95)], (100, 100))
        self.assertGreater(len(points), 5)
        self.assertGreater(points[0][1], points[-1][1])
        self.assertTrue(all(abs(x - 50) <= 3 for x, _ in points))

    def test_right_angle_not_replaced_by_centroid(self):
        polygon = [(20, 95), (30, 95), (30, 40), (90, 40), (90, 30), (20, 30)]
        points = trace_polygon(polygon, (100, 100))
        self.assertGreater(points[0][1], 80)
        self.assertGreater(points[-1][0], 75)
        self.assertTrue(any(y < 45 and x < 40 for x, y in points))

    def test_invalid_or_degenerate_mask_fails(self):
        for polygon in [[], [(0, 0)], [(1, 1), (1, 1), (1, 1)]]:
            with self.assertRaises(ValueError):
                trace_polygon(polygon, (100, 100))
