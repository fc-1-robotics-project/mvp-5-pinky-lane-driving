# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Temporal fallback and odometry contracts on synthetic camera geometry."""

import unittest

from pinky_lane_driving.calibration import Calibration
from pinky_lane_driving.path import PathSettings
from pinky_lane_driving.tracking import LaneTracker, relative_pose
from test_calibration import synthetic_config


def observation(stamp, ids=(1, 2)):
    return dict(capture_time_s=stamp, image_size=(100, 100), frame_id='camera_optical_frame',
                detections=[dict(class_id=i, boundary_px=((30. if i == 1 else 70., 90.),
                                                         (30. if i == 1 else 70., 10.)))
                            for i in ids])


class TrackingTest(unittest.TestCase):
    def setUp(self):
        config = PathSettings(.4, .08, .6, .05, .3, .05, .02)
        self.tracker = LaneTracker(Calibration(synthetic_config()), config,
                                   mounting_id='synthetic_fixture', timeout=.2)

    def test_pixel_to_metric_pair_and_bounded_fallback(self):
        self.assertTrue(self.tracker.update(observation(1.), now=1.01).valid)
        self.assertTrue(self.tracker.update(observation(1.1, (1,)), now=1.11).degraded)
        self.assertTrue(self.tracker.update(observation(1.2, (1,)), now=1.21).valid)
        self.assertFalse(self.tracker.update(observation(1.31, (1,)), now=1.32).valid)
        self.assertTrue(self.tracker.update(observation(1.4), now=1.41).valid)

    def test_starting_single_boundary_cannot_invent_a_lease(self):
        self.assertFalse(self.tracker.update(observation(1., (1,)), now=1.01).valid)

    def test_stale_out_of_order_missing_and_mismatched_inputs(self):
        self.assertFalse(self.tracker.update(observation(1.), now=2.).valid)
        self.assertTrue(self.tracker.update(observation(3.), now=3.).valid)
        self.assertFalse(self.tracker.update(observation(2.), now=3.).valid)
        invalid = observation(4.)
        invalid['image_size'] = (200, 200)
        self.assertFalse(self.tracker.update(invalid, now=4.).valid)
        self.assertFalse(self.tracker.update({}, now=4.).valid)

    def test_relative_pose_transforms_previous_frame_not_just_reuses_points(self):
        result = relative_pose((1., 2., 0.), (1.2, 2., 0.))
        self.assertAlmostEqual(result[0], -.2)
        self.assertEqual(result[1:], (0., 0.))
        self.assertTrue(self.tracker.update(observation(1.), now=1., pose=(0., 0., 0.)).valid)
        self.assertTrue(self.tracker.update(observation(1.1), now=1.1, pose=(.1, 0., 0.)).valid)

    def test_no_calibration_cannot_produce_metric_path(self):
        self.tracker.calibration = None
        self.assertFalse(self.tracker.update(observation(1.), now=1.).valid)
