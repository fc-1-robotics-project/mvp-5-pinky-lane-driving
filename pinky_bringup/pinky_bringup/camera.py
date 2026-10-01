#!/usr/bin/env python3
"""Low-latency V4L2 camera publisher with calibrated ROS CameraInfo."""

import math
from pathlib import Path
import re

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml


def load_camera_info(path, width, height):
    """Load and validate a standard ROS camera-calibration YAML file."""
    calibration = yaml.safe_load(Path(path).read_text())
    if (int(calibration['image_width']) != width
            or int(calibration['image_height']) != height):
        raise ValueError('Camera calibration resolution does not match capture')

    def values(name, count):
        data = [float(value) for value in calibration[name]['data']]
        if len(data) != count or not all(math.isfinite(value) for value in data):
            raise ValueError(f'Invalid {name} in camera calibration')
        return data

    message = CameraInfo()
    message.width = width
    message.height = height
    message.distortion_model = str(calibration['distortion_model'])
    if message.distortion_model != 'plumb_bob':
        raise ValueError('Only plumb_bob camera calibration is supported')
    message.d = values('distortion_coefficients', 5)
    message.k = values('camera_matrix', 9)
    message.r = values('rectification_matrix', 9)
    message.p = values('projection_matrix', 12)
    return message


class CameraNode(Node):
    """Capture one fresh V4L2 frame per timer tick and publish it as bgr8."""

    def __init__(self, capture=None, **kwargs):
        super().__init__('front_camera', **kwargs)

        def parameter(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True),
            ).value

        device = parameter('video_device', '/dev/video0')
        self.frame_id = parameter('frame_id', 'front_camera_link')
        self.width = parameter('image_width', 640)
        self.height = parameter('image_height', 480)
        rate = parameter('framerate', 30.0)
        pixel_format = parameter('pixel_format', 'mjpeg2rgb').lower()
        calibration_file = parameter('calibration_file', '')
        if (not device or not self.frame_id or self.width <= 0
                or self.height <= 0 or not math.isfinite(rate) or rate <= 0
                or not calibration_file):
            raise ValueError(
                'Invalid camera device, frame, size, rate or calibration',
            )
        formats = {
            'mjpeg': 'MJPG',
            'mjpeg2rgb': 'MJPG',
            'yuyv': 'YUYV',
            'yuyv2rgb': 'YUYV',
        }
        if pixel_format not in formats:
            raise ValueError(f'Unsupported camera pixel_format: {pixel_format}')

        self.info = load_camera_info(calibration_file, self.width, self.height)
        if capture is None:
            import cv2
            match = re.fullmatch(r'/dev/video([0-9]+)', device)
            if match is None:
                raise ValueError('video_device must be /dev/videoN')
            capture = cv2.VideoCapture(int(match.group(1)), cv2.CAP_V4L2)
            capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*formats[pixel_format]),
            )
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            capture.set(cv2.CAP_PROP_FPS, rate)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture = capture
        if not self.capture.isOpened():
            raise RuntimeError(f'Cannot open camera device: {device}')
        actual_width = round(self.capture.get(3))
        actual_height = round(self.capture.get(4))
        if (actual_width, actual_height) != (self.width, self.height):
            self.capture.release()
            raise RuntimeError(
                f'Camera returned {actual_width}x{actual_height}, expected '
                f'{self.width}x{self.height}',
            )

        self.image_publisher = self.create_publisher(
            Image, 'image_raw', qos_profile_sensor_data,
        )
        self.info_publisher = self.create_publisher(
            CameraInfo, 'camera_info', qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / rate, self.capture_once)
        self.get_logger().info(
            f'Camera {device}: {self.width}x{self.height} at {rate:.1f} Hz; '
            f'format={formats[pixel_format]}, frame={self.frame_id}',
        )

    def capture_once(self):
        """Publish only successfully decoded frames with a current ROS stamp."""
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().warning(
                'Camera frame capture failed', throttle_duration_sec=2.0,
            )
            return
        if frame.shape[:2] != (self.height, self.width) or frame.ndim != 3:
            self.get_logger().error(
                f'Unexpected camera frame shape: {frame.shape}',
                throttle_duration_sec=2.0,
            )
            return
        stamp = self.get_clock().now().to_msg()
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.height = self.height
        message.width = self.width
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = self.width * 3
        message.data = frame.tobytes()
        self.info.header.stamp = stamp
        self.info.header.frame_id = self.frame_id
        self.image_publisher.publish(message)
        self.info_publisher.publish(self.info)

    def destroy_node(self):
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if hasattr(self, 'capture'):
            self.capture.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
