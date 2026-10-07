# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Sensor input contracts and synthetic geometry; no hardware calibration claims."""

import math
import unittest

from pinky_lane_driving.perception import observe, project_ground


POLYGON = ((10., 20.), (30., 20.), (30., 40.), (10., 40.))
# Synthetic rectified pixel -> x forward / y left, metres; NOT robot calibration.
H = ((0., -.01, 1.), (-.01, 0., .5), (0., 0., 1.))


class PerceptionTest(unittest.TestCase):
    def test_preserve_instances_classes_and_capture_time(self):
        result = observe(2., 'camera_optical_frame', (640, 480),
                         [1, 1, 0], [.9, .8, .95], [POLYGON] * 3)
        self.assertEqual(result['capture_time_s'], 2.)
        self.assertEqual([d['class_id'] for d in result['detections']], [1, 1, 0])
        self.assertEqual(result['detections'][0]['polygon_px'], POLYGON)
        self.assertFalse(result['metric_valid'])

    def test_empty_is_observation_not_drivable_path(self):
        result = observe(0., 'camera', (640, 480), [], [], [])
        self.assertEqual(result['detections'], [])
        self.assertFalse(result['metric_valid'])

    def test_reject_invalid_input(self):
        for stamp, frame, size, ids, scores, polygons in [
            (math.nan, 'camera', (640, 480), [], [], []),
            (-1., 'camera', (640, 480), [], [], []),
            (0., '', (640, 480), [], [], []),
            (0., 'camera', (0, 480), [], [], []),
            (0., 'camera', (640, 480), [3], [.9], [POLYGON]),
            (0., 'camera', (640, 480), [1], [math.nan], [POLYGON]),
            (0., 'camera', (640, 480), [1], [.9], []),
            (0., 'camera', (640, 480), [1], [.9], [[(0., 0.)] * 3]),
            (0., 'camera', (640, 480), [1], [.9], [[(0., 0.), (700., 1.), (1., 2.)]]),
        ]:
            with self.subTest(stamp=stamp, ids=ids, polygons=polygons):
                with self.assertRaises(ValueError):
                    observe(stamp, frame, size, ids, scores, polygons)

    def test_projection_requires_rectification_and_calibration(self):
        for matrix, rectified in [(None, True), (H, False), (((0.,) * 3,) * 3, True)]:
            with self.assertRaises(ValueError):
                project_ground(POLYGON, matrix, rectified=rectified)

    def test_metric_axes_and_projective_division(self):
        result = project_ground(((10., 20.), (30., 40.)), H, rectified=True)
        self.assertAlmostEqual(result[0][0], .8)
        self.assertAlmostEqual(result[0][1], .4)
        scaled = tuple(tuple(v * 2 for v in row) for row in H)
        self.assertEqual(result, project_ground(((10., 20.), (30., 40.)), scaled,
                                                rectified=True))

    def test_horizon_nonfinite_and_behind_robot_rejected(self):
        horizon = ((1., 0., 0.), (0., 1., 0.), (0., 1., -20.))
        for points, matrix in [(((10., 20.),), horizon),
                               (((math.nan, 0.),), H), (((0., 200.),), H)]:
            with self.assertRaises(ValueError):
                project_ground(points, matrix, rectified=True)
