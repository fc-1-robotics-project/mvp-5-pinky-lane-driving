# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Measured ground-calibration configuration generation."""

import unittest

from pinky_lane_driving.ground_calibration import build_control_config


class GroundCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.camera = {
            'image_width': 100,
            'image_height': 100,
            'distortion_model': 'plumb_bob',
            'camera_matrix': {
                'data': [100., 0., 50., 0., 100., 50., 0., 0., 1.],
            },
            'distortion_coefficients': {'data': [0., 0., 0., 0., 0.]},
        }
        pixels = ((10., 10.), (90., 10.), (90., 90.), (10., 90.), (50., 50.))
        self.measurements = {
            'mounting_id': 'fixture-v1',
            'camera_frame': 'camera_optical_frame',
            'source': 'measured-fixture',
            'lane_width_m': .4,
            'rectified_roi': [10., 10., 90., 90.],
            'max_reprojection_error_m': .001,
            'correspondences': [
                {'pixel': list((u, v)), 'ground': [v / 100., .5 - u / 100.]}
                for u, v in pixels
            ],
            'path': dict(width_tolerance=.08, max_near=.6, sample_step=.03,
                         fallback_timeout=.3, fallback_speed=.03,
                         ambiguity_margin=.08, min_coverage=.7),
            'control': dict(max_speed=.08, max_omega=.6, max_accel=.1,
                            max_lateral_accel=.12, braking_decel=.15, latency=.2,
                            stop_margin=.15, timeout=.3, min_lookahead=.12,
                            max_lookahead=.35, lookahead_time=1.),
            'behavior': dict(cruise_speed=.06, approach_speed=.03,
                             stop_distance=.15, stopped_speed=.01, wait_s=2.,
                             clear_s=.5, decel=.15, latency=.2),
            'crosswalk': dict(group_gap=.2, association_distance=.4,
                              passed_margin=.15),
            'sensors': dict(scan_timeout_s=.3, odom_timeout_s=.3,
                            estop_timeout_s=.3, max_scan_gap_rad=.03,
                            footprint_radius_m=.15, infinity_is_clear=False),
        }

    def test_measured_points_generate_valid_control_configuration(self):
        config, report = build_control_config(self.camera, self.measurements)
        self.assertEqual(config['path']['width'], .4)
        self.assertEqual(config['calibration']['ground_frame'], 'base_footprint')
        self.assertLess(report['max_error_m'], 1e-7)

    def test_error_limit_and_missing_measurement_fail(self):
        self.measurements['correspondences'][-1]['ground'][0] += .1
        with self.assertRaises(ValueError):
            build_control_config(self.camera, self.measurements)
        self.measurements['lane_width_m'] = None
        with self.assertRaises((TypeError, ValueError)):
            build_control_config(self.camera, self.measurements)


if __name__ == '__main__':
    unittest.main()
