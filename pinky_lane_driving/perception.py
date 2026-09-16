# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Preserve segmentation instances and explicitly gate metric projection."""

import math


def observe(capture_time_s, frame_id, image_size, class_ids, scores, polygons):
    """Capture pixels (u right, v down), not a selected lane or metric path.

    Timestamp is the source capture clock, not inference completion time.
    Crosswalk and repeated boundary instances stay separate for later association.
    """
    if not math.isfinite(capture_time_s) or capture_time_s < 0 or not frame_id:
        raise ValueError('A finite nonnegative capture time and frame_id are required')
    if len(image_size) != 2 or any(type(v) is not int or v <= 0 for v in image_size):
        raise ValueError('Image size must be positive integer width, height')
    if not len(class_ids) == len(scores) == len(polygons):
        raise ValueError('Class, confidence and polygon counts must match')
    width, height = image_size
    detections = []
    for class_id, score, polygon in zip(class_ids, scores, polygons):
        if type(class_id) is not int or class_id not in (0, 1, 2):
            raise ValueError('Expected crosswalk=0, left line=1, right line=2')
        if not math.isfinite(score) or not 0 <= score <= 1:
            raise ValueError('Confidence must be finite and within 0..1')
        points = tuple(tuple(p) for p in polygon)
        if len(points) < 3 or any(len(p) != 2 for p in points):
            raise ValueError('A polygon needs at least three 2D vertices')
        if any(not math.isfinite(u) or not math.isfinite(v)
               or not 0 <= u <= width or not 0 <= v <= height for u, v in points):
            raise ValueError('Polygon must be finite and inside image bounds')
        area2 = sum(a[0] * b[1] - b[0] * a[1]
                    for a, b in zip(points, points[1:] + points[:1]))
        if abs(area2) < 1e-9:
            raise ValueError('Degenerate polygon')
        detections.append({'class_id': class_id, 'confidence': score, 'polygon_px': points})
    return {'capture_time_s': capture_time_s, 'frame_id': frame_id,
            'image_size': image_size, 'detections': detections, 'metric_valid': False}


def project_ground(points_px, homography, *, rectified):
    """Project already-undistorted pixels to base_footprint x/y in metres.

    Caller must supply a measured homography for this camera pose and resolution.
    This primitive does not estimate calibration, select a lane, or authorize motion.
    Entire input fails if it crosses the horizon or contains points behind the robot.
    """
    if rectified is not True or homography is None:
        raise ValueError('Measured calibration and rectified pixels are required')
    if len(homography) != 3 or any(len(row) != 3 for row in homography):
        raise ValueError('Homography must be 3x3')
    if not all(math.isfinite(v) for row in homography for v in row):
        raise ValueError('Homography must be finite')
    scale = max(abs(v) for row in homography for v in row)
    if scale == 0:
        raise ValueError('Singular homography')
    a, b, c = (tuple(v / scale for v in row) for row in homography)
    det = (a[0] * (b[1] * c[2] - b[2] * c[1])
           - a[1] * (b[0] * c[2] - b[2] * c[0])
           + a[2] * (b[0] * c[1] - b[1] * c[0]))
    if abs(det) < 1e-12:
        raise ValueError('Singular or ill-conditioned homography')
    result = []
    denominator_sign = None
    for point in points_px:
        if len(point) != 2 or not all(math.isfinite(v) for v in point):
            raise ValueError('Pixels must be finite 2D points')
        u, v = point
        denominator = c[0] * u + c[1] * v + c[2]
        sign = denominator > 0
        if (abs(denominator) < 1e-9
                or denominator_sign is not None and sign != denominator_sign):
            raise ValueError('Projection touches or crosses the ground horizon')
        denominator_sign = sign
        x = (a[0] * u + a[1] * v + a[2]) / denominator
        y = (b[0] * u + b[1] * v + b[2]) / denominator
        if not math.isfinite(x) or not math.isfinite(y) or x < 0:
            raise ValueError('Projection must be finite and in front of the robot')
        result.append((x, y))
    return tuple(result)
