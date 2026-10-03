# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""ROS-free end-to-end driving proposals; adapters own clocks, TF and publishing."""

import math

from .behavior import Behavior, arbitrate
from .control import Proposal, command
from .obstacles import transform_points
from .path import LanePath
from .tracking import relative_pose


class DriveCore:
    def __init__(self, tracker, limits, behavior_settings, crosswalk_tracker, *, scan_timeout,
                 crosswalk_control_enabled=True, recovery_min_path_length_m=None,
                 lidar_obstacle_stop_enabled=True):
        if not math.isfinite(scan_timeout) or scan_timeout <= 0:
            raise ValueError('Scan timeout must be finite and positive')
        if type(crosswalk_control_enabled) is not bool:
            raise ValueError('crosswalk_control_enabled must be a boolean')
        if type(lidar_obstacle_stop_enabled) is not bool:
            raise ValueError('lidar_obstacle_stop_enabled must be a boolean')
        if recovery_min_path_length_m is None:
            recovery_min_path_length_m = limits.stop_margin
        if (isinstance(recovery_min_path_length_m, bool)
                or not isinstance(recovery_min_path_length_m, (int, float))
                or not math.isfinite(recovery_min_path_length_m)
                or recovery_min_path_length_m < 0):
            raise ValueError('recovery_min_path_length_m must be finite and nonnegative')
        self.tracker = tracker
        self.limits = limits
        self.recovery_min_path_length_m = float(recovery_min_path_length_m)
        self.behavior = Behavior(behavior_settings)
        self.crosswalks = crosswalk_tracker
        self.crosswalk_control_enabled = crosswalk_control_enabled
        self.lidar_obstacle_stop_enabled = lidar_obstacle_stop_enabled
        self.scan_timeout = scan_timeout
        self.lane = LanePath()
        self.polygons = ()
        self.capture_pose = None
        self.capture_stamp = None
        self.last_tick = None
        self.previous_speed = 0.

    def obstacle_for_stop(self, raw_hit):
        """Operator may disable object-triggered stops; unknown stays unknown.

        Scan freshness/coverage, odometry, camera, central permit, emergency
        stop and the independent output watchdog remain separate requirements.
        """
        if type(raw_hit) is bool and not self.lidar_obstacle_stop_enabled:
            return False
        return raw_hit

    def observe(self, observation, *, now, pose):
        """Called on completed inference; pose is odom_from_base at capture time."""
        self.lane = LanePath(reason='invalid_observation')
        self.polygons = ()
        self.capture_pose = self.capture_stamp = None
        try:
            transform_points([], pose)
            lane = self.tracker.update(observation, now=now, pose=pose,
                                       min_path_length=self.recovery_min_path_length_m)
            if not lane.valid:
                self.lane = lane
                return
            polygons = ()
            if self.crosswalk_control_enabled:
                polygons = tuple(polygon for d in observation['detections'] if d['class_id'] == 0
                                 if (polygon := self.tracker.calibration.project_visible_polygon(d['polygon_px'])))
            self.lane, self.polygons = lane, polygons
            self.capture_pose, self.capture_stamp = pose, observation['capture_time_s']
        except (KeyError, ValueError, TypeError, IndexError):
            self.tracker.reset()

    def tick(self, now, *, pose, measured_speed, scan_hit, scan_age, coverage_ok, estop):
        """Separate control timer; all supplied ages/poses must be validated by adapter.

        scan_hit is True/False/None from collision checking; coverage_ok must be
        explicitly True before a no-hit result may count as clear. Current pose
        is distinct from capture pose. Outputs still require robot-side watchdog.
        """
        try:
            age = math.inf if self.capture_stamp is None else now - self.capture_stamp
            dt = 0. if self.last_tick is None else now - self.last_tick
            valid = (self.lane.valid and math.isfinite(now) and 0 <= age <= self.limits.timeout
                     and math.isfinite(scan_age) and 0 <= scan_age <= self.scan_timeout
                     and coverage_ok is True and type(scan_hit) is bool)
            event, passed = None, None
            current_path = ()
            if valid:
                transform = relative_pose(self.capture_pose, pose)
                current_path = transform_points(self.lane.points, transform)
                if self.crosswalk_control_enabled:
                    polygons = [transform_points(p, transform) for p in self.polygons]
                    event, passed = self.crosswalks.update(polygons, current_path, pose)
            decision = self.behavior.step(now, sensors_ok=valid, estop=estop,
                                          obstacle=self.obstacle_for_stop(scan_hit) if valid else None,
                                          crosswalk=event, passed_id=passed,
                                          measured_speed=measured_speed)
            requested = decision.speed_limit
            if self.lane.degraded:
                requested = min(requested, self.lane.speed_limit)
            proposal = command(current_path, age=age, dt=dt, previous_speed=self.previous_speed,
                               requested_speed=requested, metric_valid=valid, limits=self.limits)
            result = arbitrate(proposal, decision)
        except (ValueError, TypeError, IndexError, KeyError, OverflowError):
            # Reset safety-clear timers too; a failed frame must not advance restart.
            self.behavior.clear_since = self.behavior.wait_since = None
            result = Proposal(reason='invalid_runtime_input')
        self.last_tick = now if isinstance(now, (int, float)) and math.isfinite(now) else None
        self.previous_speed = result.speed
        return result
