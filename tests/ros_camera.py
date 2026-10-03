# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""ROS graph test for calibrated, current-stamped camera publication."""

from pathlib import Path
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pinky_bringup'))

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from pinky_bringup.camera import CameraNode


class FakeCapture:
    def __init__(self):
        self.released = False

    def isOpened(self):
        return True

    def get(self, prop):
        return 640 if prop == 3 else 480

    def read(self):
        return True, np.zeros((480, 640, 3), dtype=np.uint8)

    def release(self):
        self.released = True


class RosCameraTest(unittest.TestCase):
    def test_frame_and_calibration_share_a_current_stamp(self):
        rclpy.init()
        capture = FakeCapture()
        namespace = f'/camera_test_{id(self)}'
        node = CameraNode(
            capture=capture,
            namespace=namespace,
            parameter_overrides=[Parameter(
                'calibration_file',
                value=str(ROOT / 'pinky_bringup' / 'config' /
                          'pinky_camera.yaml'),
            )],
        )
        probe = rclpy.create_node('probe', namespace=namespace)
        images = []
        infos = []
        probe.create_subscription(
            Image, 'image_raw', images.append, qos_profile_sensor_data
        )
        probe.create_subscription(
            CameraInfo, 'camera_info', infos.append, qos_profile_sensor_data
        )
        try:
            end = time.monotonic() + 2.0
            while (node.image_publisher.get_subscription_count() < 1
                   or node.info_publisher.get_subscription_count() < 1):
                self.assertLess(time.monotonic(), end)
                rclpy.spin_once(probe, timeout_sec=.01)
            node.capture_once()
            while not images or not infos:
                self.assertLess(time.monotonic(), end)
                rclpy.spin_once(probe, timeout_sec=.01)
            image, info = images[-1], infos[-1]
            self.assertEqual(image.header.stamp, info.header.stamp)
            self.assertEqual(image.header.frame_id, 'front_camera_link')
            self.assertEqual(image.encoding, 'bgr8')
            self.assertEqual((image.width, image.height), (640, 480))
            self.assertEqual(len(info.k), 9)
            stamp = image.header.stamp.sec + image.header.stamp.nanosec / 1e9
            now = node.get_clock().now().nanoseconds / 1e9
            self.assertGreaterEqual(now - stamp, 0.)
            self.assertLess(now - stamp, .2)
        finally:
            node.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()
        self.assertTrue(capture.released)


if __name__ == '__main__':
    unittest.main()
