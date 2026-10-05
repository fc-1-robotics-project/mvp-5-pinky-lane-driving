# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""ROS sensor/TF adapter; publishes proposals only, never motor Twist commands."""

import json
import math
from pathlib import Path
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .behavior import Settings
from .calibration import Calibration
from .control import Limits, Proposal, command
from .crosswalk import CrosswalkTracker
from .obstacles import convex_footprint, scan_collision, steering_corridor, stopping_corridor, transform_points, validate_self_filter, validate_self_radius
from .path import PathSettings
from .runtime import DriveCore
from .tracking import LaneTracker, relative_pose


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


def nonnegative_margin(value, name):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0):
        raise ValueError(f'{name} must be finite and nonnegative')
    return float(value)


def planar_pose(position, orientation):
    values = (position.x, position.y, position.z, orientation.x, orientation.y,
              orientation.z, orientation.w)
    if (not all(math.isfinite(v) for v in values)
            or abs(sum(v * v for v in values[3:]) - 1.) > 1e-3
            or abs(orientation.x) > 1e-3 or abs(orientation.y) > 1e-3):
        raise ValueError('Nonfinite, nonunit or nonplanar transform')
    return (position.x, position.y,
            math.atan2(2 * orientation.w * orientation.z, 1 - 2 * orientation.z**2))


class ControlNode(Node):
    def __init__(self, config=None, **kwargs):
        super().__init__('lane_control', **kwargs)
        config_path = self.declare_parameter('config_path', '',
                                             ParameterDescriptor(read_only=True)).value
        if config is None:
            if not config_path:
                raise ValueError('Measured control configuration is required')
            config = json.loads(Path(config_path).read_text())
        calibration = Calibration(config['calibration'])
        limits = Limits(**config['control'])
        settings = PathSettings(**config['path'])
        recovery_min_path_length_m = nonnegative_margin(
            config.get('recovery_min_path_length_m', limits.stop_margin),
            'recovery_min_path_length_m')
        tracker = LaneTracker(calibration, settings, mounting_id=config['mounting_id'],
                              timeout=limits.timeout)
        self.sensors = config['sensors']
        self.obstacle_stop_margin_m = nonnegative_margin(
            self.sensors.get('obstacle_stop_margin_m', limits.stop_margin),
            'obstacle_stop_margin_m')
        # Heading-straight reaction check only counts returns inside the lane.
        self.straight_check_half_width_m = settings.width / 2 + nonnegative_margin(
            self.sensors.get('straight_check_lane_margin_m', 0.), 'straight_check_lane_margin_m')
        for key in ('scan_timeout_s', 'odom_timeout_s', 'estop_timeout_s',
                    'max_scan_gap_rad', 'footprint_radius_m'):
            if not math.isfinite(self.sensors[key]) or self.sensors[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if type(self.sensors['infinity_is_clear']) is not bool:
            raise ValueError('infinity_is_clear requires verified driver boolean contract')
        footprint = self.sensors.get('footprint_polygon_m')
        if footprint is not None:
            self.sensors['footprint_polygon_m'] = convex_footprint(footprint)
            padding = self.sensors.get('footprint_padding_m', 0.)
            if not math.isfinite(padding) or padding < 0:
                raise ValueError('Footprint padding must be finite and nonnegative')
        self_filter = self.sensors.get('self_filter_bounds_m')
        if self_filter is not None:
            if footprint is None:
                raise ValueError('Self-return filter requires a physical polygon footprint')
            validate_self_filter(self_filter, self.sensors['footprint_polygon_m'])
        self_radius = self.sensors.get('self_filter_radius_m')
        if self_radius is not None:
            if footprint is None:
                raise ValueError('Self-return radius requires a physical footprint envelope')
            validate_self_radius(self_radius, self.sensors['footprint_polygon_m'])
        self.core = DriveCore(tracker, limits, Settings(**config['behavior']),
                              CrosswalkTracker(**config['crosswalk']),
                              scan_timeout=self.sensors['scan_timeout_s'],
                              crosswalk_control_enabled=config.get('crosswalk_control_enabled', True),
                              lidar_obstacle_stop_enabled=config.get('lidar_obstacle_stop_enabled', True),
                              recovery_min_path_length_m=recovery_min_path_length_m)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.odom = self.scan = self.estop = None
        self.odom_history = deque(maxlen=8)
        self.scan_history = deque(maxlen=3)
        self.sequence = 0
        self.last_reason = None
        self.last_diagnostic_time = -math.inf
        self.observation_error = 'No observation received'
        self.publisher = self.create_publisher(String, 'lane/command', 1)
        self.diagnostic_publisher = self.create_publisher(String, 'lane/diagnostics', 1)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.receive_odom, sensor_qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.receive_scan, sensor_qos)
        self.stop_sub = self.create_subscription(Bool, 'lane/estop', self.receive_stop, 1)
        self.mission_id = ''
        self.mission_sub = self.create_subscription(
            String, 'lane/mission_status', self.receive_mission, 10)
        self.observation_sub = self.create_subscription(String, 'lane/observation', self.observe, 1)
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive_odom(self, message):
        if self.odom and stamp_seconds(message) <= stamp_seconds(self.odom[0]):
            self.odom = None
            self.odom_history.clear()
        else:
            self.odom = (message, time.monotonic())
            self.odom_history.append(self.odom)

    def receive_scan(self, message):
        if self.scan and stamp_seconds(message) <= stamp_seconds(self.scan[0]):
            self.scan = None
            self.scan_history.clear()
        else:
            self.scan = (message, time.monotonic())
            self.scan_history.append(self.scan)

    def receive_stop(self, message):
        self.estop = (message.data, time.monotonic())

    def receive_mission(self, message):
        """Reset mission-scoped state once per accepted FollowLane goal."""
        try:
            status = json.loads(message.data)
            mission_id = status['mission_id']
            active = status['active'] is True
        except (ValueError, TypeError, KeyError):
            return
        if active and isinstance(mission_id, str) and mission_id and mission_id != self.mission_id:
            self.mission_id = mission_id
            self.core.start_mission()

    def tf_pose(self, target, source, stamp):
        transform = self.buffer.lookup_transform(target, source,
                                                 Time(seconds=stamp, clock_type=ClockType.ROS_TIME))
        return planar_pose(transform.transform.translation, transform.transform.rotation)

    def observe(self, message):
        now = self.get_clock().now().nanoseconds / 1e9
        try:
            if len(message.data) > 8_000_000:
                raise ValueError('Oversized observation')
            observation = json.loads(message.data)
            stamp = observation['capture_time_s']
            if not math.isfinite(stamp) or not 0 <= now - stamp <= self.core.limits.timeout:
                raise ValueError('Invalid capture age')
            pose = self.tf_pose('odom', 'base_footprint', stamp)
            self.core.observe(observation, now=now, pose=pose)
            self.observation_error = None
        except (ValueError, TypeError, KeyError, TransformException, OverflowError) as error:
            self.observation_error = str(error)
            self.core.observe({}, now=now, pose=None)

    def fresh(self, item, now, timeout):
        if item is None:
            return False
        return (0 <= now - stamp_seconds(item[0]) <= timeout
                and 0 <= time.monotonic() - item[1] <= timeout)

    def scan_transform_with_odom(self, now):
        """Find a fresh scan/odometry pair whose TF has reached the buffer.

        Both dynamic TF and their corresponding messages arrive independently.
        Use only a bounded recent history; returned scan and odometry must both
        supply the geometry used for this tick.
        """
        latest_stamp = stamp_seconds(self.odom[0])
        first_error = None
        for scan_item in reversed(self.scan_history):
            if not self.fresh(scan_item, now, self.sensors['scan_timeout_s']):
                continue
            scan = scan_item[0]
            for odom_item in reversed(self.odom_history):
                if latest_stamp - stamp_seconds(odom_item[0]) > .18:
                    break
                if not self.fresh(odom_item, now, self.sensors['odom_timeout_s']):
                    continue
                odom = odom_item[0]
                if odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_footprint':
                    raise ValueError('Unexpected odometry frames')
                try:
                    transform = self.buffer.lookup_transform_full(
                        'base_footprint', Time.from_msg(odom.header.stamp),
                        scan.header.frame_id, Time.from_msg(scan.header.stamp), 'odom')
                    return odom, scan, transform
                except TransformException as error:
                    if first_error is None:
                        first_error = error
        if first_error is None:
            raise ValueError('No fresh scan/odometry for transform')
        raise first_error

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        pose, measured_speed, hit, scan_age, coverage = None, math.nan, None, math.inf, False
        selected_odom = selected_scan = None
        sensor_error = None
        collision_horizon = None
        forward_stop_horizon = None
        emergency = (self.estop is None or self.estop[0]
                     or time.monotonic() - self.estop[1] > self.sensors['estop_timeout_s'])
        try:
            if not self.fresh(self.odom, now, self.sensors['odom_timeout_s']):
                raise ValueError('Odometry unavailable or stale')
            if not self.fresh(self.scan, now, self.sensors['scan_timeout_s']):
                raise ValueError('Scan unavailable or stale')
            odom, scan, transform = self.scan_transform_with_odom(now)
            selected_odom, selected_scan = odom, scan
            scan_age = now - stamp_seconds(scan)
            pose = planar_pose(odom.pose.pose.position, odom.pose.pose.orientation)
            measured_speed = odom.twist.twist.linear.x
            scan_pose = planar_pose(transform.transform.translation, transform.transform.rotation)
            self_pose = None
            if (self.sensors.get('self_filter_bounds_m') is not None
                    or self.sensors.get('self_filter_radius_m') is not None):
                static_tf = self.buffer.lookup_transform(
                    'base_footprint', scan.header.frame_id, Time.from_msg(scan.header.stamp))
                self_pose = planar_pose(static_tf.transform.translation, static_tf.transform.rotation)
            if self.core.lane.valid:
                path = transform_points(self.core.lane.points, relative_pose(self.core.capture_pose, pose))
                geometry = command(path, age=now-self.core.capture_stamp, dt=.05,
                                   previous_speed=self.core.previous_speed,
                                   requested_speed=self.core.limits.max_speed, metric_valid=True,
                                   limits=self.core.limits)
                stopping_limits = dict(
                    max_speed=self.core.limits.max_speed, measured_speed=measured_speed,
                    decel=self.core.limits.braking_decel, latency=self.core.limits.latency,
                    scan_timeout=self.sensors['scan_timeout_s'], timer_period=.05)
                path_for_lane = path
                path, collision_horizon = stopping_corridor(
                    path, **stopping_limits, observation_timeout=self.core.limits.timeout,
                    stop_margin=self.obstacle_stop_margin_m)
                steering_path, steering_guard = None, 0.
                if geometry.reason == 'tracking':
                    footprint = self.sensors.get('footprint_polygon_m')
                    reach = (max(math.hypot(*v) for v in footprint) if footprint is not None
                             else self.sensors['footprint_radius_m'])
                    steering_path, steering_guard = steering_corridor(
                        collision_horizon, geometry.curvature, reach)
                    # The command arc retains the full preview and stop margin.
                    # Also protect forward reaction/braking travel in case the
                    # steering response lags. The independently refreshed lidar
                    # can stop this motion without waiting for another camera
                    # frame. Extending this straight alternative through the
                    # camera lease + preview margin falsely blocks side walls
                    # that the commanded turn clears.
                    path, forward_stop_horizon = stopping_corridor(
                        ((0., 0.), (collision_horizon, 0.)), **stopping_limits,
                        observation_timeout=0., stop_margin=0.)
                hit = scan_collision(scan.ranges, angle_min=scan.angle_min,
                                     angle_increment=scan.angle_increment, range_min=scan.range_min,
                                     range_max=scan.range_max, scan_pose=scan_pose, path=path,
                                     radius=self.sensors['footprint_radius_m'], age=scan_age,
                                     timeout=self.sensors['scan_timeout_s'],
                                     infinity_is_clear=self.sensors['infinity_is_clear'],
                                     footprint=self.sensors.get('footprint_polygon_m'),
                                     padding=self.sensors.get('footprint_padding_m', 0.),
                                     self_filter_bounds=self.sensors.get('self_filter_bounds_m'),
                                     self_filter_radius=self.sensors.get('self_filter_radius_m'),
                                     self_filter_pose=self_pose, steering_path=steering_path,
                                     steering_guard=steering_guard, lane_path=path_for_lane,
                                     lane_half_width=self.straight_check_half_width_m)
            increment = abs(scan.angle_increment)
            span = increment * (len(scan.ranges) - 1)
            # Conservative full-circle gate. Partial-FOV hardware needs a separate
            # validated swept-corridor visibility model, never an assumed clear flag.
            coverage = (0 < increment <= self.sensors['max_scan_gap_rad']
                        and 2 * math.pi - 2 * increment <= span <= 2 * math.pi + increment
                        and math.isfinite(scan.angle_max)
                        and abs(scan.angle_max - scan.angle_min
                                - scan.angle_increment * (len(scan.ranges) - 1)) < 1e-4)
        except (ValueError, TypeError, IndexError, TransformException, OverflowError) as error:
            sensor_error = str(error)
            coverage = False
        result = self.core.tick(now, pose=pose, measured_speed=measured_speed,
                                scan_hit=hit, scan_age=scan_age, coverage_ok=coverage, estop=emergency)
        reason_changed = result.reason != self.last_reason
        self.publish(result, now)
        if reason_changed or time.monotonic() - self.last_diagnostic_time >= .5:
            def age(item):
                return None if item is None else now - stamp_seconds(item[0])

            capture_age = None if self.core.capture_stamp is None else now - self.core.capture_stamp
            diagnostic = dict(reason=result.reason, lane_valid=self.core.lane.valid,
                              lane_degraded=self.core.lane.degraded,
                              path_points_m=self.core.lane.points,
                              lane_reason=self.core.lane.reason, capture_age_s=capture_age,
                              observation_error=self.observation_error, sensor_error=sensor_error,
                              odom_age_s=(now - stamp_seconds(selected_odom)
                                          if selected_odom is not None else age(self.odom)),
                              scan_age_s=(now - stamp_seconds(selected_scan)
                                          if selected_scan is not None else age(self.scan)),
                              scan_coverage_ok=coverage,
                              scan_hit=self.core.obstacle_for_stop(hit),
                              raw_scan_hit=hit,
                              lidar_obstacle_stop_enabled=self.core.lidar_obstacle_stop_enabled,
                              emergency=emergency)
            diagnostic['tf_odom_rewind_s'] = (stamp_seconds(self.odom[0]) - stamp_seconds(selected_odom)
                                              if self.odom and selected_odom is not None else None)
            diagnostic['tf_scan_rewind_s'] = (stamp_seconds(self.scan[0]) - stamp_seconds(selected_scan)
                                              if self.scan and selected_scan is not None else None)
            diagnostic['collision_horizon_m'] = collision_horizon
            diagnostic['forward_stop_horizon_m'] = forward_stop_horizon
            diagnostic['obstacle_stop_margin_m'] = self.obstacle_stop_margin_m
            diagnostic['recovery_min_path_length_m'] = self.core.recovery_min_path_length_m
            diagnostic['self_filter_radius_m'] = self.sensors.get('self_filter_radius_m')
            if result.reason == 'sensor_failure':
                if not self.core.lane.valid:
                    diagnostic['fault'] = 'lane:' + self.core.lane.reason
                elif capture_age is None or not 0 <= capture_age <= self.core.limits.timeout:
                    diagnostic['fault'] = 'lane_observation_stale'
                elif sensor_error:
                    diagnostic['fault'] = sensor_error
                elif not coverage:
                    diagnostic['fault'] = 'insufficient_scan_coverage'
                elif hit is None:
                    diagnostic['fault'] = 'unknown_scan_collision_result'
                else:
                    diagnostic['fault'] = 'invalid_measured_speed_or_runtime_input'
            if self.scan:
                scan = selected_scan if selected_scan is not None else self.scan[0]
                diagnostic['invalid_scan_returns'] = sum(
                    not (scan.range_min <= value <= scan.range_max
                         or value == math.inf and self.sensors['infinity_is_clear'])
                    for value in scan.ranges)
            self.diagnostic_publisher.publish(String(data=json.dumps(diagnostic, allow_nan=False)))
            self.last_diagnostic_time = time.monotonic()

    def publish(self, result, now):
        self.sequence += 1
        capture_stamp = self.core.capture_stamp if result.speed > 0 else now
        payload = dict(sequence=self.sequence, capture_stamp=capture_stamp,
                       speed=result.speed, omega=result.omega,
                       reason=result.reason)
        self.publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
        if result.reason != self.last_reason:
            self.get_logger().info(f'Control: {result.reason}; v={result.speed:.3f}, w={result.omega:.3f}')
            self.last_reason = result.reason

    def stop(self):
        self.publish(Proposal(reason='shutdown'), self.get_clock().now().nanoseconds / 1e9)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
