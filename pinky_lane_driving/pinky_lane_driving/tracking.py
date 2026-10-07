# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Join calibrated observations to current-frame lane geometry; no stale path reuse."""

import math

from .obstacles import point_segment_distance, transform_points
from .path import Boundary, LanePath, sections, select_path
from .perception import project_ground


def relative_pose(previous, current):
    """current_base_from_previous_base, given two odom_from_base SE(2) poses."""
    transform_points([], previous)
    transform_points([], current)
    dx, dy = previous[0] - current[0], previous[1] - current[1]
    c, s = math.cos(current[2]), math.sin(current[2])
    return (c * dx + s * dy, -s * dx + c * dy, previous[2] - current[2])


def arc_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def forward_measured_path(points):
    """Discard already-passed samples without extending the measured endpoint."""
    forward = []
    for a, b in zip(points, points[1:]):
        if not forward:
            if a[0] >= 0:
                forward.append(a)
            elif b[0] >= 0:
                fraction = -a[0] / (b[0] - a[0])
                forward.append((0., a[1] + fraction * (b[1] - a[1])))
            else:
                continue
        if b[0] < 0:
            break
        if math.dist(forward[-1], b) > 1e-9:
            forward.append(b)
    return tuple(forward) if len(forward) >= 2 else ()


def agrees_with_path(candidate, previous, tolerance):
    """A short current measurement may borrow extent only from the same route."""
    if len(candidate) < 2 or len(previous) < 2:
        return False
    segments = tuple(zip(previous, previous[1:]))
    if any(min(point_segment_distance(point, a, b) for a, b in segments) > tolerance
           for point in candidate):
        return False
    near_segment = min(segments, key=lambda pair:
                       point_segment_distance(candidate[0], *pair))
    a, b = near_segment
    current = (candidate[-1][0] - candidate[0][0],
               candidate[-1][1] - candidate[0][1])
    old = (b[0] - a[0], b[1] - a[1])
    scale = math.hypot(*current) * math.hypot(*old)
    return scale > 1e-12 and sum(x * y for x, y in zip(current, old)) / scale >= .5


def fragments_agree_with_path(boundaries, previous, settings):
    """Even too-short edge fragments must support the remembered lane side."""
    for boundary in boundaries:
        direction = 1 if boundary.class_id == 1 else -1
        centers = tuple((point[0] + direction * settings.width / 2 * normal[0],
                         point[1] + direction * settings.width / 2 * normal[1])
                        for point, normal in sections(boundary.points, settings.sample_step))
        if not agrees_with_path(centers, previous, settings.width_tolerance / 2):
            return False
    return True


class LaneTracker:
    def __init__(self, calibration, settings, *, mounting_id, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Capture timeout must be finite and positive')
        if calibration is not None and abs(calibration.lane_width - settings.width) > 1e-9:
            raise ValueError('Path width must match measured calibration width')
        self.calibration = calibration
        self.settings = settings
        self.mounting_id = mounting_id
        self.timeout = timeout
        self.last_stamp = None
        self.last_pair_stamp = None
        self.previous = ()
        self.previous_pose = None
        self.last_visible_stamp = None
        self.last_visible_pose = None
        self.last_visible_points = ()

    def reset(self):
        self.last_stamp = self.last_pair_stamp = None
        self.previous = ()
        self.previous_pose = None
        self.last_visible_stamp = None
        self.last_visible_pose = None
        self.last_visible_points = ()

    def update(self, observation, *, now, pose=None, min_path_length=0.):
        """Use fresh capture-clock observations and optional pose AT capture time.

        Pixel traces must be near-to-far. Trim to the first contiguous calibrated
        ROI segment, never bridge gaps. No pose means no temporal geometry hint.
        The controller must separately transform capture-frame path to timer-time
        base frame and enforce capture age again before issuing any proposal.
        """
        if self.calibration is None:
            self.reset()
            return LanePath(reason='uncalibrated')
        try:
            if not math.isfinite(min_path_length) or min_path_length < 0:
                raise ValueError('Minimum path length must be finite and nonnegative')
            stamp = observation['capture_time_s']
            if (not math.isfinite(now) or not math.isfinite(stamp)
                    or not 0 <= now - stamp <= self.timeout
                    or self.last_stamp is not None and stamp <= self.last_stamp):
                raise ValueError('Stale, future or out-of-order capture')
            self.calibration.validate_source(observation['image_size'], observation['frame_id'],
                                             self.mounting_id)
            if pose is not None:
                transform_points([], pose)
            boundaries = []
            for detection in observation['detections']:
                if detection['class_id'] == 0:
                    continue
                # A distant/out-of-ROI or untraceable instance is not evidence
                # that the other currently visible boundary is invalid.
                raw = detection.get('boundary_px', ())
                if len(raw) < 2:
                    continue
                pixels = self.calibration.rectify(raw)
                x0, y0, x1, y1 = self.calibration.roi
                clipped = []
                for u, v in pixels:
                    if x0 <= u <= x1 and y0 <= v <= y1:
                        clipped.append((u, v))
                    elif clipped:
                        break
                if len(clipped) < 2:
                    continue
                metric = project_ground(clipped, self.calibration.homography, rectified=True)
                boundaries.append(Boundary(detection['class_id'], metric))
            previous = ()
            if (self.previous and pose is not None and self.previous_pose is not None
                    and self.last_stamp is not None
                    and stamp - self.last_stamp <= min(self.timeout, self.settings.fallback_timeout)):
                previous = transform_points(self.previous, relative_pose(self.previous_pose, pose))
            fallback_age = None if self.last_pair_stamp is None else stamp - self.last_pair_stamp
            result = select_path(boundaries, self.settings, fallback_age=fallback_age,
                                 previous=previous)
            short_measured = (result.valid and min_path_length > 0
                              and arc_length(result.points) <= min_path_length)
            weak_unusable = (not result.valid and result.reason == 'no_current_lane'
                             and all(arc_length(b.points) < 3 * self.settings.sample_step
                                     for b in boundaries))
            if ((short_measured or weak_unusable) and self.settings.blind_timeout > 0
                    and self.last_visible_points and pose is not None
                    and self.last_visible_pose is not None
                    and self.last_visible_stamp is not None
                    and 0 <= stamp - self.last_visible_stamp <= self.settings.blind_timeout
                    and math.dist(pose[:2], self.last_visible_pose[:2])
                    <= self.settings.blind_distance
                    and abs(math.remainder(pose[2] - self.last_visible_pose[2],
                                           2 * math.pi)) <= .5):
                # Follow only the already measured path, transformed by
                # odometry. Never extrapolate its endpoint or renew the
                # blind time/distance lease with another empty image.
                points = forward_measured_path(transform_points(
                    self.last_visible_points,
                    relative_pose(self.last_visible_pose, pose)))
                if (arc_length(points) > min_path_length
                        and (not short_measured or agrees_with_path(
                            result.points, points, self.settings.width_tolerance / 2))
                        and (not weak_unusable or fragments_agree_with_path(
                            boundaries, points, self.settings))):
                    reason = ('recent_path_short_observation' if short_measured else
                              'recent_path_no_boundaries' if not boundaries else
                              'recent_path_unusable_boundaries')
                    result = LanePath(tuple(points), (), True, True,
                                      self.settings.blind_speed, reason)
            self.last_stamp = stamp
            self.previous = result.points if result.valid else ()
            self.previous_pose = pose if result.valid else None
            if (result.valid and boundaries and result.reason in ('paired', 'single_boundary')
                    and arc_length(result.points) > min_path_length):
                self.last_visible_stamp = stamp
                self.last_visible_pose = pose
                self.last_visible_points = result.points
            if result.valid and not result.degraded:
                self.last_pair_stamp = stamp
            return result
        except (KeyError, ValueError, TypeError, IndexError):
            self.reset()
            return LanePath(reason='invalid_observation_or_calibration')
