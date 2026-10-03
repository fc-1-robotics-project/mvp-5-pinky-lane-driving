# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Connect real algorithm components without ROS or motors."""

import unittest
from dataclasses import replace

from pinky_lane_driving.behavior import Settings
from pinky_lane_driving.calibration import Calibration
from pinky_lane_driving.control import Limits
from pinky_lane_driving.crosswalk import CrosswalkTracker
from pinky_lane_driving.path import PathSettings
from pinky_lane_driving.runtime import DriveCore
from pinky_lane_driving.tracking import LaneTracker
from test_calibration import synthetic_config
from test_tracking import observation


class RuntimeTest(unittest.TestCase):
    def setUp(self):
        tracker = LaneTracker(Calibration(synthetic_config()),
                              PathSettings(.4, .08, .6, .05, .3, .05, .02),
                              mounting_id='synthetic_fixture', timeout=.2)
        self.core = DriveCore(tracker,
                              Limits(.2, .8, 1., .1, .3, .1, .1, .2, .2, .5, 1.),
                              Settings(.2, .1, .15, .01, .2, .05, .3, .1),
                              CrosswalkTracker(group_gap=.2, association_distance=.4,
                                               passed_margin=.15), scan_timeout=.2)

    def tick(self, time, **overrides):
        args = dict(pose=(0., 0., 0.), measured_speed=0., scan_hit=False,
                    scan_age=.01, coverage_ok=True, estop=False)
        args.update(overrides)
        return self.core.tick(time, **args)

    def update(self, time, ids=(1, 2)):
        self.core.observe(observation(time, ids), now=time, pose=(0., 0., 0.))

    def test_full_lane_to_control_chain_and_sensor_priority(self):
        self.update(1.)
        self.assertEqual(self.tick(1.).speed, 0.)
        self.update(1.06)
        self.assertGreater(self.tick(1.06).speed, 0.)
        self.assertEqual(self.tick(1.07, scan_hit=True).speed, 0.)
        self.assertEqual(self.tick(1.08, coverage_ok=False).speed, 0.)
        self.assertEqual(self.tick(1.09, estop=True).speed, 0.)
        self.assertEqual(self.tick(1.5).speed, 0.)

    def test_one_sided_cap_survives_final_arbitration(self):
        self.update(1.)
        self.tick(1.)
        self.update(1.1, (1,))
        result = self.tick(1.1)
        self.assertGreater(result.speed, 0.)
        self.assertLessEqual(result.speed, .05)
        self.update(1.4, (1,))
        self.assertEqual(self.tick(1.4).speed, 0.)

    def disable_lidar_obstacle_stop(self):
        self.core = DriveCore(self.core.tracker, self.core.limits,
                              self.core.behavior.settings, self.core.crosswalks,
                              scan_timeout=self.core.scan_timeout,
                              lidar_obstacle_stop_enabled=False)

    def test_disabled_lidar_obstacle_stop_follows_despite_valid_hits(self):
        self.disable_lidar_obstacle_stop()
        self.update(1.)
        self.tick(1., scan_hit=True)
        for i in range(1, 11):
            stamp = 1. + i * .06
            self.update(stamp)
            result = self.tick(stamp, scan_hit=True)
            self.assertGreater(result.speed, 0.)
            self.assertLessEqual(result.speed, self.core.limits.max_speed)
            self.assertEqual(result.reason, 'follow')
        self.assertFalse(self.core.obstacle_for_stop(True))
        self.assertIsNone(self.core.obstacle_for_stop(None))

    def test_disabled_obstacle_stop_preserves_emergency_and_sensor_faults(self):
        for override in (dict(estop=True), dict(scan_hit=None), dict(scan_age=.3),
                         dict(coverage_ok=False), dict(pose=None)):
            with self.subTest(override=override):
                self.setUp()
                self.disable_lidar_obstacle_stop()
                self.update(1.)
                self.tick(1., scan_hit=True)
                self.assertGreater(self.tick(1.06, scan_hit=True).speed, 0.)
                self.assertEqual(self.tick(1.07, **override).speed, 0.)
        self.setUp()
        self.disable_lidar_obstacle_stop()
        self.update(1.)
        self.tick(1.)
        self.assertEqual(self.tick(1.4, scan_hit=True).speed, 0.)
        self.core.observe({}, now=1.5, pose=(0.,0.,0.))
        self.assertEqual(self.tick(1.5, scan_hit=True).speed, 0.)

    def test_lidar_obstacle_stop_defaults_on_and_requires_boolean(self):
        self.assertTrue(self.core.lidar_obstacle_stop_enabled)
        self.assertTrue(self.core.obstacle_for_stop(True))
        for value in (0, 1, None, 'false'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'lidar_obstacle_stop_enabled'):
                DriveCore(self.core.tracker, self.core.limits, self.core.behavior.settings,
                          self.core.crosswalks, scan_timeout=self.core.scan_timeout,
                          lidar_obstacle_stop_enabled=value)

    def test_continuous_fresh_single_boundary_moves_beyond_last_pair_timeout(self):
        self.update(1.)
        self.tick(1.)
        for index in range(1, 31):
            stamp = 1. + index * .1
            self.update(stamp, (2,))
            result = self.tick(stamp)
            self.assertGreater(result.speed, 0.)
            self.assertLessEqual(result.speed, .05)
        self.update(4.1, ())
        self.assertEqual(self.tick(4.1).speed, 0.)

    def test_bounded_missing_lane_uses_known_path_but_sensor_priority_remains(self):
        self.core.tracker.settings = replace(self.core.tracker.settings,
                                             blind_timeout=.4,
                                             blind_distance=.05,
                                             blind_speed=.02)
        self.update(1.)
        self.tick(1.)
        self.update(1.1, ())
        self.assertEqual(self.core.lane.reason, 'recent_path_no_boundaries')
        self.assertLessEqual(self.tick(1.1).speed, .02)
        self.assertEqual(self.tick(1.11, scan_hit=True).speed, 0.)
        self.assertEqual(self.tick(1.12, coverage_ok=False).speed, 0.)
        self.assertEqual(self.tick(1.13, estop=True).speed, 0.)
        self.update(1.5, ())
        self.assertEqual(self.tick(1.5).speed, 0.)

    def test_short_measurement_uses_recent_path_without_renewing_lease(self):
        self.core.tracker.settings = replace(self.core.tracker.settings,
                                             blind_timeout=.4,
                                             blind_distance=.05,
                                             blind_speed=.02)
        self.update(1.)
        self.tick(1.)
        short = observation(1.1)
        for detection in short['detections']:
            u = detection['boundary_px'][0][0]
            detection['boundary_px'] = ((u, 90.), (u, 84.))
        self.core.observe(short, now=1.1, pose=(.01, 0., 0.))
        self.assertEqual(self.core.lane.reason, 'recent_path_short_observation')
        result = self.tick(1.1, pose=(.01, 0., 0.))
        self.assertGreater(result.speed, 0.)
        self.assertLessEqual(result.speed, .02)
        self.assertEqual(self.tick(1.11, pose=(.01, 0., 0.), scan_hit=True).speed, 0.)
        self.assertEqual(self.tick(1.12, pose=(.01, 0., 0.), estop=True).speed, 0.)
        short['capture_time_s'] = 1.41
        self.core.observe(short, now=1.41, pose=(.02, 0., 0.))
        self.assertNotEqual(self.core.lane.reason, 'recent_path_short_observation')
        self.assertEqual(self.tick(1.41, pose=(.02, 0., 0.)).speed, 0.)

    def test_short_visible_terminal_fragment_moves_without_renewing_blind_lease(self):
        self.core.tracker.settings = replace(self.core.tracker.settings,
                                             blind_timeout=.05,
                                             blind_distance=.05,
                                             blind_speed=.02)
        limits = replace(self.core.limits, stop_margin=.02)
        self.core = DriveCore(self.core.tracker, limits, self.core.behavior.settings,
                              self.core.crosswalks, scan_timeout=self.core.scan_timeout,
                              recovery_min_path_length_m=.06)
        self.update(1.)
        self.tick(1.)
        self.assertEqual(self.core.tracker.last_visible_stamp, 1.)

        # The old blind lease has expired. The current 5.5 cm measured pair
        # remains driveable with a 2 cm endpoint reserve, but cannot refresh
        # the separate 6 cm last-visible threshold.
        short = observation(1.21)
        for detection in short['detections']:
            u = detection['boundary_px'][0][0]
            detection['boundary_px'] = ((u, 90.), (u, 84.5))
        self.core.observe(short, now=1.21, pose=(0., 0., 0.))
        self.assertEqual(self.core.lane.reason, 'paired')
        self.assertEqual(self.core.tracker.last_visible_stamp, 1.)
        self.assertGreater(self.tick(1.21).speed, 0.)

        self.update(1.3, ())
        self.assertFalse(self.core.lane.valid)
        self.assertEqual(self.tick(1.3).speed, 0.)

    def test_recovery_threshold_defaults_to_controller_margin_and_rejects_invalid(self):
        self.assertEqual(self.core.recovery_min_path_length_m, self.core.limits.stop_margin)
        for value in (-.01, float('nan'), True, '0.06'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                DriveCore(self.core.tracker, self.core.limits, self.core.behavior.settings,
                          self.core.crosswalks, scan_timeout=self.core.scan_timeout,
                          recovery_min_path_length_m=value)

    def test_projection_or_pose_failure_stops_not_reuses_old_path(self):
        self.update(1.)
        self.tick(1.)
        self.core.observe({}, now=1.1, pose=(0., 0., 0.))
        self.assertEqual(self.tick(1.1).speed, 0.)
        self.update(1.2)
        self.assertEqual(self.tick(1.2, pose=None).speed, 0.)

    def test_crosswalk_input_enters_behavior_chain(self):
        obs = observation(1.)
        obs['detections'].append(dict(class_id=0, polygon_px=((20., 80.), (80., 80.),
                                                             (80., 90.), (20., 90.))))
        self.core.observe(obs, now=1., pose=(0., 0., 0.))
        self.assertEqual(self.tick(1.).speed, 0.)
        self.assertEqual(self.core.behavior.state, 'WAIT')

    def test_crosswalk_bypass_preserves_lane_and_safety_controls(self):
        self.core = DriveCore(self.core.tracker, self.core.limits,
                              self.core.behavior.settings, self.core.crosswalks,
                              scan_timeout=self.core.scan_timeout,
                              crosswalk_control_enabled=False)
        obs = observation(1.)
        # This class-0 shape would fail ground projection if the bypass still
        # processed crosswalk polygons. Lane boundaries remain valid.
        obs['detections'].append(dict(class_id=0, polygon_px=((float('nan'), 80.),)))
        self.core.observe(obs, now=1., pose=(0., 0., 0.))
        self.assertTrue(self.core.lane.valid)
        self.assertEqual(self.core.polygons, ())
        self.tick(1.)
        self.assertGreater(self.tick(1.06).speed, 0.)
        self.assertEqual(self.core.behavior.state, 'FOLLOW')
        self.assertEqual(self.tick(1.07, scan_hit=True).speed, 0.)
        self.assertEqual(self.tick(1.08, coverage_ok=False).speed, 0.)
        self.assertEqual(self.tick(1.09, estop=True).speed, 0.)

    def test_crosswalk_bypass_rejects_non_boolean_configuration(self):
        with self.assertRaisesRegex(ValueError, 'crosswalk_control_enabled'):
            DriveCore(self.core.tracker, self.core.limits, self.core.behavior.settings,
                      self.core.crosswalks, scan_timeout=self.core.scan_timeout,
                      crosswalk_control_enabled=0)

    def test_crosswalk_outside_roi_does_not_invalidate_fresh_lane(self):
        self.core.tracker.calibration.roi = (0., 0., 100., 95.)
        obs = observation(1.)
        obs['detections'].append(dict(class_id=0, polygon_px=((10., 96.), (90., 96.),
                                                             (90., 99.), (10., 99.))))
        self.core.observe(obs, now=1., pose=(0., 0., 0.))
        self.assertTrue(self.core.lane.valid)
        self.assertEqual(self.core.polygons, ())
