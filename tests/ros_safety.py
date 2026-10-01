# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""ROS graph check for the fail-closed local motion latch."""

import os
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from pinky_lane_driving.ros_safety import SafetyNode


class RosSafetyTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        namespace = f'/safety_test_{os.getpid()}'
        self.safety = SafetyNode(namespace=namespace)
        self.probe = rclpy.create_node('safety_probe', namespace=namespace)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.safety)
        self.executor.add_node(self.probe)
        self.messages = []
        self.subscription = self.probe.create_subscription(
            Bool, 'lane/estop', lambda message: self.messages.append(message.data), 10
        )
        self.client = self.probe.create_client(SetBool, 'lane/set_enabled')

    def tearDown(self):
        self.executor.shutdown()
        self.safety.destroy_node()
        self.probe.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=2.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        self.fail('Safety graph condition timed out')

    def set_enabled(self, enabled):
        self.spin_until(self.client.service_is_ready)
        future = self.client.call_async(SetBool.Request(data=enabled))
        self.spin_until(future.done)
        self.assertTrue(future.result().success)

    def test_starts_stopped_and_requires_explicit_enable(self):
        self.spin_until(lambda: self.messages and self.messages[-1] is True)
        self.set_enabled(True)
        self.spin_until(lambda: self.messages[-1] is False)
        self.set_enabled(False)
        self.spin_until(lambda: self.messages[-1] is True)


if __name__ == '__main__':
    unittest.main()
