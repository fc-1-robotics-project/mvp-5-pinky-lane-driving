# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Planar scan-return collision primitives; coverage validation belongs to adapter."""

import math


def transform_points(points, pose):
    """Apply target_from_source planar rigid transform (x_m, y_m, yaw_rad)."""
    if pose is None or len(pose) != 3 or not all(math.isfinite(v) for v in pose):
        raise ValueError('A finite planar transform is required')
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    result = []
    for point in points:
        if len(point) != 2 or not all(math.isfinite(v) for v in point):
            raise ValueError('Points must be finite 2D coordinates')
        u, v = point
        result.append((x + c * u - s * v, y + s * u + c * v))
    return tuple(result)


def point_segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return math.dist(point, a)
    t = max(0., min(1., ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def scan_collision(ranges, *, angle_min, angle_increment, range_min, range_max,
                   scan_pose, path, radius, age, timeout, infinity_is_clear=False):
    """Return hit/no-hit/unknown for available laser returns in a swept corridor.

    radius is measured robot circumscribed footprint radius plus safety margin.
    A disk over-approximates footprint for every heading (conservative at corners).
    False means no sampled return collides, NOT proof of free unseen space: the
    ROS adapter must independently validate FOV/angular coverage of the corridor.
    Infinity is unknown unless the driver explicitly documents no-return as clear.
    This cannot detect objects below the scan plane; it never commands avoidance.
    """
    values = (angle_min, angle_increment, range_min, range_max, radius, age, timeout)
    if (not all(math.isfinite(v) for v in values) or radius <= 0 or timeout <= 0
            or not 0 <= age <= timeout or not 0 <= range_min < range_max
            or angle_increment == 0 or len(ranges) == 0 or len(path) < 2):
        return None
    try:
        path = transform_points(path, (0., 0., 0.))
        transform_points([], scan_pose)
    except (ValueError, TypeError):
        return None
    if max(math.dist(scan_pose[:2], p) for p in path) + radius > range_max:
        return None
    laser_points = []
    for index, value in enumerate(ranges):
        if value == math.inf and infinity_is_clear is True:
            continue
        if not math.isfinite(value) or not range_min <= value <= range_max:
            return None
        angle = angle_min + index * angle_increment
        laser_points.append((value * math.cos(angle), value * math.sin(angle)))
    points = transform_points(laser_points, scan_pose)
    swept = ((0., 0.),) + path
    return any(point_segment_distance(p, a, b) <= radius
               for p in points for a, b in zip(swept, swept[1:]))
