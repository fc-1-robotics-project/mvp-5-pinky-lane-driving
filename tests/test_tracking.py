# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Temporal fallback and odometry contracts on synthetic camera geometry."""

import unittest
from dataclasses import replace

from pinky_lane_driving.calibration import Calibration
from pinky_lane_driving.path import PathSettings
from pinky_lane_driving.tracking import LaneTracker, arc_length, relative_pose
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

    def test_fresh_one_sided_sequence_continues_with_odometry_but_loss_stops(self):
        self.assertTrue(self.tracker.update(observation(1.), now=1., pose=(0., 0., 0.)).valid)
        for index in range(1, 31):
            stamp = 1. + index * .1
            lane = self.tracker.update(observation(stamp, (1,)), now=stamp,
                                       pose=(index * .002, 0., 0.))
            self.assertTrue(lane.valid)
            self.assertTrue(lane.degraded)
        self.assertFalse(self.tracker.update(observation(4.1, ()), now=4.1,
                                             pose=(.062, 0., 0.)).valid)

    def test_empty_images_follow_only_recent_visible_path_with_odometry(self):
        self.tracker.settings = replace(self.tracker.settings, blind_timeout=.6,
                                        blind_distance=.05, blind_speed=.02)
        visible = self.tracker.update(observation(1.), now=1., pose=(0., 0., 0.))
        self.assertTrue(visible.valid)
        hidden = self.tracker.update(observation(1.1, ()), now=1.1,
                                     pose=(.01, 0., 0.))
        self.assertTrue(hidden.valid)
        self.assertTrue(hidden.degraded)
        self.assertEqual(hidden.reason, 'recent_path_no_boundaries')
        self.assertEqual(hidden.speed_limit, .02)
        self.assertAlmostEqual(hidden.points[0][0], visible.points[0][0] - .01)
        self.assertFalse(self.tracker.update(observation(1.2, ()), now=1.2,
                                             pose=(.06, 0., 0.)).valid)

    def test_empty_observations_do_not_renew_blind_lease(self):
        self.tracker.settings = replace(self.tracker.settings, blind_timeout=.3,
                                        blind_distance=.05, blind_speed=.02)
        self.assertTrue(self.tracker.update(observation(1.), now=1.,
                                            pose=(0., 0., 0.)).valid)
        self.assertTrue(self.tracker.update(observation(1.1, ()), now=1.1,
                                            pose=(0., 0., 0.)).valid)
        self.assertTrue(self.tracker.update(observation(1.2, ()), now=1.2,
                                            pose=(0., 0., 0.)).valid)
        self.assertFalse(self.tracker.update(observation(1.4, ()), now=1.4,
                                             pose=(0., 0., 0.)).valid)
        self.assertTrue(self.tracker.update(observation(1.5), now=1.5,
                                            pose=(0., 0., 0.)).valid)

    def test_empty_images_without_capture_pose_do_not_drive(self):
        self.tracker.settings = replace(self.tracker.settings, blind_timeout=.6,
                                        blind_distance=.05, blind_speed=.02)
        self.assertTrue(self.tracker.update(observation(1.), now=1.,
                                            pose=(0., 0., 0.)).valid)
        self.assertFalse(self.tracker.update(observation(1.1, ()), now=1.1,
                                             pose=None).valid)

    def test_short_current_pair_borrows_only_recent_matching_measured_extent(self):
        self.tracker.settings = replace(self.tracker.settings, blind_timeout=.4,
                                        blind_distance=.05, blind_speed=.02)
        self.assertTrue(self.tracker.update(observation(1.), now=1.,
                                            pose=(0., 0., 0.),
                                            min_path_length=.1).valid)
        short = observation(1.1)
        for detection in short['detections']:
            u = detection['boundary_px'][0][0]
            detection['boundary_px'] = ((u, 90.), (u, 84.))
        recovered = self.tracker.update(short, now=1.1, pose=(.01, 0., 0.),
                                        min_path_length=.1)
        self.assertTrue(recovered.valid)
        self.assertEqual(recovered.reason, 'recent_path_short_observation')
        self.assertLessEqual(recovered.speed_limit, .02)
        self.assertGreater(arc_length(recovered.points), .1)
        self.assertEqual(self.tracker.last_visible_stamp, 1.)
        short['capture_time_s'] = 1.41
        expired = self.tracker.update(short, now=1.41, pose=(.02, 0., 0.),
                                      min_path_length=.1)
        self.assertNotEqual(expired.reason, 'recent_path_short_observation')
        self.assertEqual(self.tracker.last_visible_stamp, 1.)

    def test_tiny_fragments_recover_but_substantial_conflict_does_not(self):
        self.tracker.settings = replace(self.tracker.settings, blind_timeout=.4,
                                        blind_distance=.05, blind_speed=.02)
        self.tracker.update(observation(1.), now=1., pose=(0., 0., 0.),
                            min_path_length=.1)
        tiny = observation(1.1, (1,))
        tiny['detections'][0]['boundary_px'] = ((30., 90.), (30., 86.))
        recovered = self.tracker.update(tiny, now=1.1, pose=(0., 0., 0.),
                                        min_path_length=.1)
        self.assertEqual(recovered.reason, 'recent_path_unusable_boundaries')
        tiny_conflict = observation(1.15, (1,))
        tiny_conflict['detections'][0]['boundary_px'] = ((0., 90.), (0., 86.))
        rejected_tiny = self.tracker.update(tiny_conflict, now=1.15,
                                            pose=(0., 0., 0.), min_path_length=.1)
        self.assertFalse(rejected_tiny.valid)
        self.assertEqual(rejected_tiny.reason, 'no_current_lane')
        conflict = observation(1.2, (1,))
        conflict['detections'][0]['boundary_px'] = ((0., 90.), (0., 10.))
        rejected = self.tracker.update(conflict, now=1.2, pose=(0., 0., 0.),
                                       min_path_length=.1)
        self.assertFalse(rejected.valid)
        self.assertEqual(rejected.reason, 'no_current_lane')

    def test_recovered_path_trims_passed_prefix_without_extending_endpoint(self):
        self.tracker.settings = replace(self.tracker.settings, blind_timeout=.4,
                                        blind_distance=.2, blind_speed=.02)
        visible = self.tracker.update(observation(1.), now=1.,
                                      pose=(0., 0., 0.), min_path_length=.1)
        recovered = self.tracker.update(observation(1.1, ()), now=1.1,
                                        pose=(.15, 0., 0.), min_path_length=.1)
        self.assertEqual(recovered.reason, 'recent_path_no_boundaries')
        self.assertAlmostEqual(recovered.points[0][0], 0.)
        self.assertTrue(all(point[0] >= 0 for point in recovered.points))
        self.assertAlmostEqual(recovered.points[-1][0], visible.points[-1][0] - .15)
        self.assertEqual(self.tracker.last_visible_points, visible.points)

    def test_out_of_roi_instance_does_not_discard_visible_boundary(self):
        self.assertTrue(self.tracker.update(observation(1.), now=1., pose=(0., 0., 0.)).valid)
        obs = observation(1.1, (2,))
        obs['detections'].append(dict(class_id=1, boundary_px=((150., 150.), (160., 160.))))
        result = self.tracker.update(obs, now=1.1, pose=(0., 0., 0.))
        self.assertTrue(result.valid)
        self.assertTrue(result.degraded)

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
