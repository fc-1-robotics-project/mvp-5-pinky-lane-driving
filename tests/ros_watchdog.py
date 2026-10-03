# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""ROS graph test: isolated namespace, exclusively dry-run velocity topic."""

import json
import os
import time
import unittest

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String

from pinky_lane_driving.ros_watchdog import WatchdogNode


class RosWatchdogTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        namespace = f'/lane_test_{os.getpid()}'
        self.watchdog = WatchdogNode(namespace=namespace)
        self.probe = rclpy.create_node('probe', namespace=namespace)
        self.messages = []
        self.sub = self.probe.create_subscription(Twist, 'lane/dry_run_cmd_vel',
                                                 lambda msg: self.messages.append(msg), 10)
        self.pub = self.probe.create_publisher(String, 'lane/command', 1)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.watchdog)
        self.executor.add_node(self.probe)
        self.wait(lambda: self.pub.get_subscription_count() == 1)

    def tearDown(self):
        self.executor.shutdown()
        self.watchdog.destroy_node()
        self.probe.destroy_node()

    def wait(self, predicate, timeout=3.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        self.fail('ROS condition timed out')

    def send(self, sequence, **extra):
        payload = dict(sequence=sequence, capture_stamp=self.probe.get_clock().now().nanoseconds / 1e9,
                       speed=.1, omega=.2)
        payload.update(extra)
        self.pub.publish(String(data=json.dumps(payload)))

    def test_command_and_publisher_silence_stops_on_dry_topic(self):
        self.send(1)
        self.wait(lambda: any(m.linear.x == .1 for m in self.messages))
        end = time.monotonic() + .3
        self.wait(lambda: time.monotonic() >= end)
        self.assertEqual((self.messages[-1].linear.x, self.messages[-1].angular.z), (0., 0.))
        topics = self.watchdog.get_publisher_names_and_types_by_node(
            self.watchdog.get_name(), self.watchdog.get_namespace())
        self.assertFalse(any(name.endswith('/cmd_vel') for name, _ in topics))

    def test_stale_malformed_and_parameter_mutation_rejected(self):
        self.send(1, capture_stamp=0.)
        self.wait(lambda: len(self.messages) >= 3)
        self.assertTrue(all(m.linear.x == 0. for m in self.messages))
        self.pub.publish(String(data='{broken'))
        before = len(self.messages)
        self.wait(lambda: len(self.messages) > before + 2)
        self.assertEqual(self.messages[-1].linear.x, 0.)
        result = self.watchdog.set_parameters([Parameter('dry_run', value=False)])
        self.assertFalse(result[0].successful)
