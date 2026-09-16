# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Synthetic calibration configuration; never use these values on hardware."""

import copy
import unittest

from pinky_lane_driving.calibration import Calibration


def synthetic_config():
    return dict(image_size=[100, 100], camera_frame='camera_optical_frame',
                ground_frame='base_footprint', mounting_id='synthetic_fixture',
                source='synthetic test only', lane_width_m=.4,
                camera_matrix=[[100., 0., 50.], [0., 100., 50.], [0., 0., 1.]],
                distortion=[0.] * 5,
                homography=[[0., -.01, 1.], [-.01, 0., .5], [0., 0., 1.]],
                rectified_roi=[0., 0., 100., 100.])


class CalibrationTest(unittest.TestCase):
    def test_synthetic_projection_and_source_contract(self):
        cal = Calibration(synthetic_config())
        cal.validate_source((100, 100), 'camera_optical_frame', 'synthetic_fixture')
        point = cal.project([(50., 50.)])[0]
        self.assertAlmostEqual(point[0], .5)
        self.assertAlmostEqual(point[1], 0.)

    def test_resolution_frame_or_mount_changes_require_new_calibration(self):
        cal = Calibration(synthetic_config())
        for args in [((200, 100), 'camera_optical_frame', 'synthetic_fixture'),
                     ((100, 100), 'other_camera', 'synthetic_fixture'),
                     ((100, 100), 'camera_optical_frame', 'moved_camera')]:
            with self.assertRaises(ValueError):
                cal.validate_source(*args)

    def test_missing_invalid_or_unmeasured_configuration_fails(self):
        for changes in [dict(source=''), dict(lane_width_m=0.),
                        dict(ground_frame='map'), dict(mounting_id=''),
                        dict(image_size=[100., 100]), dict(distortion=[float('nan')] * 5),
                        dict(camera_matrix=[[0.] * 3] * 3),
                        dict(homography=[[0.] * 3] * 3),
                        dict(rectified_roi=[50., 0., 40., 100.])]:
            config = copy.deepcopy(synthetic_config())
            config.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                Calibration(config)
        with self.assertRaises(ValueError):
            Calibration({})

    def test_projection_outside_measured_domain_fails(self):
        config = synthetic_config()
        config['rectified_roi'] = [10., 10., 90., 90.]
        cal = Calibration(config)
        with self.assertRaises(ValueError):
            cal.project([(5., 50.)])
