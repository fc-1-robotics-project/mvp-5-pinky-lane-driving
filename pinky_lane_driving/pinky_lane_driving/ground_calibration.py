# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Generate the measured lane-control JSON from camera and floor measurements."""

import argparse
import json
import math
from pathlib import Path

from .behavior import Settings
from .calibration import Calibration
from .control import Limits
from .crosswalk import CrosswalkTracker
from .path import PathSettings


def _finite_points(values, *, count, name):
    points = tuple(tuple(point) for point in values)
    if (len(points) < count or any(len(point) != 2 for point in points)
            or not all(math.isfinite(float(value)) for point in points for value in point)):
        raise ValueError(f'{name} requires at least {count} finite 2D points')
    return points


def build_control_config(camera, measurements):
    """Return a validated control configuration and reprojection statistics."""
    import cv2
    import numpy as np

    if camera.get('distortion_model') != 'plumb_bob':
        raise ValueError('Only plumb_bob camera calibration is supported')
    width, height = int(camera['image_width']), int(camera['image_height'])
    matrix = np.asarray(camera['camera_matrix']['data'], dtype=float).reshape(3, 3)
    distortion = np.asarray(
        camera['distortion_coefficients']['data'], dtype=float
    ).reshape(-1)
    if (width <= 0 or height <= 0 or not np.isfinite(matrix).all()
            or not np.isfinite(distortion).all()):
        raise ValueError('Invalid camera calibration values')

    pairs = measurements['correspondences']
    raw_pixels = _finite_points(
        (pair['pixel'] for pair in pairs), count=4, name='pixel correspondences'
    )
    ground = _finite_points(
        (pair['ground'] for pair in pairs), count=4, name='ground correspondences'
    )
    if len(raw_pixels) != len(ground):
        raise ValueError('Pixel and ground correspondence counts differ')
    pixels_array = np.asarray(raw_pixels, dtype=float).reshape(-1, 1, 2)
    rectified = cv2.undistortPoints(
        pixels_array, matrix, distortion, P=matrix
    ).reshape(-1, 2)
    homography, _ = cv2.findHomography(
        rectified, np.asarray(ground, dtype=float), method=0
    )
    if homography is None or not np.isfinite(homography).all():
        raise ValueError('Ground homography could not be solved')
    estimate = cv2.perspectiveTransform(
        rectified.reshape(-1, 1, 2), homography
    ).reshape(-1, 2)
    errors = np.linalg.norm(estimate - np.asarray(ground, dtype=float), axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))
    maximum = float(np.max(errors))
    error_limit = float(measurements['max_reprojection_error_m'])
    if not math.isfinite(error_limit) or error_limit <= 0 or maximum > error_limit:
        raise ValueError(
            f'Ground reprojection max error {maximum:.4f} m exceeds '
            f'{error_limit:.4f} m'
        )

    mounting_id = measurements['mounting_id']
    lane_width = float(measurements['lane_width_m'])
    calibration = {
        'image_size': [width, height],
        'camera_frame': measurements['camera_frame'],
        'ground_frame': 'base_footprint',
        'mounting_id': mounting_id,
        'source': measurements['source'],
        'lane_width_m': lane_width,
        'camera_matrix': matrix.tolist(),
        'distortion': distortion.tolist(),
        'homography': homography.tolist(),
        'rectified_roi': measurements['rectified_roi'],
    }
    Calibration(calibration).validate_source(
        (width, height), measurements['camera_frame'], mounting_id
    )

    path = dict(measurements['path'])
    path['width'] = lane_width
    control = dict(measurements['control'])
    behavior = dict(measurements['behavior'])
    crosswalk = dict(measurements['crosswalk'])
    sensors = dict(measurements['sensors'])
    PathSettings(**path)
    Limits(**control)
    Settings(**behavior)
    CrosswalkTracker(**crosswalk)
    for key in ('scan_timeout_s', 'odom_timeout_s', 'estop_timeout_s',
                'max_scan_gap_rad', 'footprint_radius_m'):
        value = sensors[key]
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if type(sensors['infinity_is_clear']) is not bool:
        raise ValueError('infinity_is_clear must be a measured driver contract')

    return ({
        'calibration': calibration,
        'mounting_id': mounting_id,
        'path': path,
        'control': control,
        'behavior': behavior,
        'crosswalk': crosswalk,
        'sensors': sensors,
    }, {'rmse_m': rmse, 'max_error_m': maximum})


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Build lane control JSON from measured floor correspondences.'
    )
    parser.add_argument('--camera', required=True, help='ROS camera calibration YAML')
    parser.add_argument('--measurements', required=True,
                        help='Measured ground-correspondence YAML')
    parser.add_argument('--output', required=True, help='Output control JSON')
    options = parser.parse_args(args)

    import yaml
    camera = yaml.safe_load(Path(options.camera).read_text())
    measurements = yaml.safe_load(Path(options.measurements).read_text())
    config, report = build_control_config(camera, measurements)
    output = Path(options.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, indent=2, allow_nan=False) + '\n')
    print(f'Wrote {output}')
    print(f"Ground reprojection RMSE: {report['rmse_m']:.4f} m")
    print(f"Ground reprojection max:  {report['max_error_m']:.4f} m")


if __name__ == '__main__':
    main()
