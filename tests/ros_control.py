# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Synthetic sensor graph -> controller -> independent dry-run watchdog."""

from dataclasses import asdict
import json
import math
import os
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster

from pinky_lane_driving.ros_control import ControlNode
from pinky_lane_driving.ros_watchdog import WatchdogNode
from test_calibration import synthetic_config
import test_runtime
from test_tracking import observation


class RosControlTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        helper = test_runtime.RuntimeTest()
        helper.setUp()
        config = dict(calibration=synthetic_config(), mounting_id='synthetic_fixture',
                      path=asdict(helper.core.tracker.settings), control=asdict(helper.core.limits),
                      behavior=asdict(helper.core.behavior.settings),
                      crosswalk=dict(group_gap=.2, association_distance=.4, passed_margin=.15),
                      sensors=dict(scan_timeout_s=.2, odom_timeout_s=.2, estop_timeout_s=.2,
                                   max_scan_gap_rad=.03, footprint_radius_m=.15,
                                   infinity_is_clear=False))
        namespace = f'/control_test_{os.getpid()}'
        self.controller = ControlNode(config=config, namespace=namespace)
        self.watchdog = WatchdogNode(namespace=namespace)
        self.probe = rclpy.create_node('synthetic_sensors', namespace=namespace)
        self.executor = SingleThreadedExecutor()
        for node in (self.controller, self.watchdog, self.probe):
            self.executor.add_node(node)
        self.obs_pub = self.probe.create_publisher(String, 'lane/observation', 1)
        self.scan_pub = self.probe.create_publisher(LaserScan, 'scan', 1)
        self.odom_pub = self.probe.create_publisher(Odometry, 'odom', 1)
        self.estop_pub = self.probe.create_publisher(Bool, 'lane/estop', 1)
        self.messages = []
        self.sub = self.probe.create_subscription(Twist, 'lane/dry_run_cmd_vel',
                                                 lambda msg: self.messages.append(msg), 10)
        self.tf = StaticTransformBroadcaster(self.probe)
        transforms = []
        for parent, child in [('odom', 'base_footprint'), ('base_footprint', 'laser')]:
            tf = TransformStamped()
            tf.header.stamp = self.probe.get_clock().now().to_msg()
            tf.header.frame_id, tf.child_frame_id = parent, child
            tf.transform.rotation.w = 1.
            transforms.append(tf)
        self.tf.sendTransform(transforms)  # Stationary synthetic robot, not physical TF evidence.
        self.send_scan = True
        self.partial_scan = False
        self.emergency = False
        self.timer = self.probe.create_timer(.03, self.publish_sensors)

    def tearDown(self):
        self.executor.shutdown()
        for node in (self.controller, self.watchdog, self.probe):
            node.destroy_node()
        rclpy.shutdown()

    def publish_sensors(self):
        stamp = self.probe.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
        odom.pose.pose.orientation.w = 1.
        self.odom_pub.publish(odom)
        self.estop_pub.publish(Bool(data=self.emergency))
        if self.send_scan:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'laser'
            scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 360
            scan.range_min, scan.range_max = .02, 3.
            scan.ranges = [2.] * (90 if self.partial_scan else 360)
            scan.angle_max = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
            self.scan_pub.publish(scan)
        self.obs_pub.publish(String(data=json.dumps(observation(stamp.sec + stamp.nanosec / 1e9))))

    def wait(self, predicate, timeout=4.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        self.fail('Controller graph condition timed out')

    def wait_duration(self, seconds):
        end = time.monotonic() + seconds
        self.wait(lambda: time.monotonic() >= end)

    def test_valid_graph_moves_dry_run_then_scan_loss_stops(self):
        self.wait(lambda: any(m.linear.x > 0 for m in self.messages))
        self.send_scan = False
        self.wait_duration(.4)
        self.assertEqual(self.messages[-1].linear.x, 0.)

    def test_partial_scan_and_emergency_never_count_as_clear(self):
        self.partial_scan = True
        self.wait_duration(.3)
        self.assertTrue(self.messages)
        self.assertTrue(all(m.linear.x == 0. for m in self.messages))
        self.partial_scan = False
        self.wait(lambda: self.messages[-1].linear.x > 0.)
        self.emergency = True
        self.wait_duration(.15)
        self.assertEqual(self.messages[-1].linear.x, 0.)
