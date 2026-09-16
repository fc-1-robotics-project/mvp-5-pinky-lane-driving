# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""ROS sensor/TF adapter; publishes proposals only, never motor Twist commands."""

import json
import math
from pathlib import Path
import time

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
from .control import Limits, Proposal
from .crosswalk import CrosswalkTracker
from .obstacles import scan_collision, transform_points
from .path import PathSettings
from .runtime import DriveCore
from .tracking import LaneTracker, relative_pose


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


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
        tracker = LaneTracker(calibration, settings, mounting_id=config['mounting_id'],
                              timeout=limits.timeout)
        self.sensors = config['sensors']
        for key in ('scan_timeout_s', 'odom_timeout_s', 'estop_timeout_s',
                    'max_scan_gap_rad', 'footprint_radius_m'):
            if not math.isfinite(self.sensors[key]) or self.sensors[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if type(self.sensors['infinity_is_clear']) is not bool:
            raise ValueError('infinity_is_clear requires verified driver boolean contract')
        self.core = DriveCore(tracker, limits, Settings(**config['behavior']),
                              CrosswalkTracker(**config['crosswalk']),
                              scan_timeout=self.sensors['scan_timeout_s'])
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.odom = self.scan = self.estop = None
        self.sequence = 0
        self.last_reason = None
        self.publisher = self.create_publisher(String, 'lane/command', 1)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.receive_odom, sensor_qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.receive_scan, sensor_qos)
        self.stop_sub = self.create_subscription(Bool, 'lane/estop', self.receive_stop, 1)
        self.observation_sub = self.create_subscription(String, 'lane/observation', self.observe, 1)
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive_odom(self, message):
        if self.odom and stamp_seconds(message) <= stamp_seconds(self.odom[0]):
            self.odom = None
        else:
            self.odom = (message, time.monotonic())

    def receive_scan(self, message):
        if self.scan and stamp_seconds(message) <= stamp_seconds(self.scan[0]):
            self.scan = None
        else:
            self.scan = (message, time.monotonic())

    def receive_stop(self, message):
        self.estop = (message.data, time.monotonic())

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
        except (ValueError, TypeError, KeyError, TransformException, OverflowError):
            self.core.observe({}, now=now, pose=None)

    def fresh(self, item, now, timeout):
        if item is None:
            return False
        return (0 <= now - stamp_seconds(item[0]) <= timeout
                and 0 <= time.monotonic() - item[1] <= timeout)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        pose, measured_speed, hit, scan_age, coverage = None, math.nan, None, math.inf, False
        emergency = (self.estop is None or self.estop[0]
                     or time.monotonic() - self.estop[1] > self.sensors['estop_timeout_s'])
        try:
            if not self.fresh(self.odom, now, self.sensors['odom_timeout_s']):
                raise ValueError('Odometry unavailable or stale')
            odom = self.odom[0]
            if odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_footprint':
                raise ValueError('Unexpected odometry frames')
            pose = planar_pose(odom.pose.pose.position, odom.pose.pose.orientation)
            measured_speed = odom.twist.twist.linear.x
            if not self.fresh(self.scan, now, self.sensors['scan_timeout_s']):
                raise ValueError('Scan unavailable or stale')
            scan = self.scan[0]
            scan_age = now - stamp_seconds(scan)
            transform = self.buffer.lookup_transform_full(
                'base_footprint', Time.from_msg(odom.header.stamp),
                scan.header.frame_id, Time.from_msg(scan.header.stamp), 'odom')
            scan_pose = planar_pose(transform.transform.translation, transform.transform.rotation)
            path = transform_points(self.core.lane.points, relative_pose(self.core.capture_pose, pose))
            hit = scan_collision(scan.ranges, angle_min=scan.angle_min,
                                 angle_increment=scan.angle_increment, range_min=scan.range_min,
                                 range_max=scan.range_max, scan_pose=scan_pose, path=path,
                                 radius=self.sensors['footprint_radius_m'], age=scan_age,
                                 timeout=self.sensors['scan_timeout_s'],
                                 infinity_is_clear=self.sensors['infinity_is_clear'])
            increment = abs(scan.angle_increment)
            span = increment * (len(scan.ranges) - 1)
            # Conservative full-circle gate. Partial-FOV hardware needs a separate
            # validated swept-corridor visibility model, never an assumed clear flag.
            coverage = (0 < increment <= self.sensors['max_scan_gap_rad']
                        and 2 * math.pi - 2 * increment <= span <= 2 * math.pi + increment
                        and math.isfinite(scan.angle_max)
                        and abs(scan.angle_max - scan.angle_min
                                - scan.angle_increment * (len(scan.ranges) - 1)) < 1e-4)
        except (ValueError, TypeError, IndexError, TransformException, OverflowError):
            coverage = False
        result = self.core.tick(now, pose=pose, measured_speed=measured_speed,
                                scan_hit=hit, scan_age=scan_age, coverage_ok=coverage, estop=emergency)
        self.publish(result, now)

    def publish(self, result, now):
        self.sequence += 1
        capture_stamp = self.core.capture_stamp if result.speed > 0 else now
        payload = dict(sequence=self.sequence, capture_stamp=capture_stamp,
                       speed=result.speed, omega=result.omega)
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
