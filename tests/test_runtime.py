# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Connect real algorithm components without ROS or motors."""

import unittest

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
