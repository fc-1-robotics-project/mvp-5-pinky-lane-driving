# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""ROS callback/worker integration; fake inference isolates scheduling behavior."""

import json
import os
import threading
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String

from pinky_lane_driving.ros_perception import PerceptionNode


class RosPerceptionTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.release = threading.Event()
        self.started = threading.Event()
        self.processed = []

        def predictor(message):
            self.processed.append(message.width)
            if len(self.processed) == 1:
                self.started.set()
                if not self.release.wait(2.):
                    raise RuntimeError('test worker timed out')
            return dict(capture_time_s=message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                        frame_id=message.header.frame_id, image_size=[message.width, message.height],
                        detections=[], metric_valid=False)

        namespace = f'/perception_test_{os.getpid()}'
        self.node = PerceptionNode(predictor=predictor, namespace=namespace,
                                  parameter_overrides=[Parameter('max_age_s', value=.5)])
        self.probe = rclpy.create_node('probe', namespace=namespace)
        self.messages = []
        self.sub = self.probe.create_subscription(String, 'lane/observation',
                                                 lambda m: self.messages.append(json.loads(m.data)), 1)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.probe)
        self.wait(lambda: self.node.publisher.get_subscription_count() == 1)

    def tearDown(self):
        self.release.set()
        self.executor.shutdown()
        self.node.destroy_node()
        self.probe.destroy_node()
        rclpy.shutdown()

    def wait(self, condition, timeout=3.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)
            if condition():
                return
        self.fail('ROS perception condition timed out')

    def image(self, index):
        msg = Image()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        msg.width, msg.height = index, 2
        return msg

    def test_latest_frame_replaces_pending_and_capture_time_survives(self):
        first = self.image(1)
        self.node.receive(first)
        self.wait(self.started.is_set)
        self.node.receive(self.image(2))
        last = self.image(3)
        self.node.receive(last)
        self.release.set()
        self.wait(lambda: len(self.processed) == 2 and len(self.messages) >= 2)
        self.assertEqual(self.processed, [1, 3])
        self.assertEqual(self.messages[-1]['capture_time_s'],
                         last.header.stamp.sec + last.header.stamp.nanosec / 1e9)

    def test_slow_inference_is_not_published_as_fresh(self):
        self.node.receive(self.image(1))
        self.wait(self.started.is_set)
        end = time.monotonic() + .55
        self.wait(lambda: time.monotonic() >= end)
        self.release.set()
        self.wait(lambda: self.node.future is None)
        self.assertEqual(self.messages, [])
