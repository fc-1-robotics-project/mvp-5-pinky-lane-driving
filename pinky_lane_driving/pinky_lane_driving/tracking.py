# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Join calibrated observations to current-frame lane geometry; no stale path reuse."""

import math

from .obstacles import transform_points
from .path import Boundary, LanePath, select_path
from .perception import project_ground


def relative_pose(previous, current):
    """current_base_from_previous_base, given two odom_from_base SE(2) poses."""
    transform_points([], previous)
    transform_points([], current)
    dx, dy = previous[0] - current[0], previous[1] - current[1]
    c, s = math.cos(current[2]), math.sin(current[2])
    return (c * dx + s * dy, -s * dx + c * dy, previous[2] - current[2])


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

    def reset(self):
        self.last_stamp = self.last_pair_stamp = None
        self.previous = ()
        self.previous_pose = None

    def update(self, observation, *, now, pose=None):
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
                raw = detection['boundary_px']
                pixels = self.calibration.rectify(raw)
                x0, y0, x1, y1 = self.calibration.roi
                clipped = []
                for u, v in pixels:
                    if x0 <= u <= x1 and y0 <= v <= y1:
                        clipped.append((u, v))
                    elif clipped:
                        break
                if len(clipped) < 2:
                    raise ValueError('Boundary not visible in calibrated domain')
                metric = project_ground(clipped, self.calibration.homography, rectified=True)
                boundaries.append(Boundary(detection['class_id'], metric))
            previous = ()
            if (self.previous and pose is not None and self.previous_pose is not None
                    and self.last_stamp is not None and stamp - self.last_stamp <= self.timeout):
                previous = transform_points(self.previous, relative_pose(self.previous_pose, pose))
            fallback_age = None if self.last_pair_stamp is None else stamp - self.last_pair_stamp
            result = select_path(boundaries, self.settings, fallback_age=fallback_age,
                                 previous=previous)
            self.last_stamp = stamp
            self.previous = result.points if result.valid else ()
            self.previous_pose = pose if result.valid else None
            if result.valid and not result.degraded:
                self.last_pair_stamp = stamp
            return result
        except (KeyError, ValueError, TypeError, IndexError):
            self.reset()
            return LanePath(reason='invalid_observation_or_calibration')
