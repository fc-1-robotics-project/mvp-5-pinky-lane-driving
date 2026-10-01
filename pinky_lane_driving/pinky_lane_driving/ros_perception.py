# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Latest-image ROS perception, isolated from the independent control timer."""

from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .vision import decode_image, observe_result, trace_observation, validate_model


class PerceptionNode(Node):
    def __init__(self, predictor=None, **kwargs):
        super().__init__('lane_perception', **kwargs)

        def parameter(name, default):
            return self.declare_parameter(name, default,
                                          ParameterDescriptor(read_only=True)).value

        self.max_age = parameter('max_age_s', .3)
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_age_s must be finite and positive')
        topic = parameter('image_topic', 'camera/image_raw')
        model_path = parameter('model_path', '')
        device = parameter('device', 'cpu')
        cpu_threads = parameter('cpu_threads', 1)
        if type(cpu_threads) is not int or not 1 <= cpu_threads <= 4:
            raise ValueError('cpu_threads must be an integer within 1..4')
        imgsz, conf = parameter('imgsz', 640), parameter('confidence', .25)
        if imgsz <= 0 or imgsz % 32 or not math.isfinite(conf) or not 0 <= conf <= 1:
            raise ValueError('Invalid image size or confidence')
        if predictor is None:
            if not Path(model_path).is_file():
                raise ValueError('Provide a trusted local model_path; no automatic download')
            import torch
            import cv2
            from ultralytics import YOLO
            cv2.setNumThreads(1)
            torch.set_num_threads(cpu_threads)
            model = YOLO(model_path)
            validate_model(model.task, model.names)

            def predictor(message):
                started = time.monotonic()
                frame = decode_image(message)
                # Ultralytics' initial device setup resets PyTorch's thread
                # count. Reassert the budget on every subsequent frame so CPU
                # inference cannot monopolize camera/control/watchdog cores.
                torch.set_num_threads(cpu_threads)
                result = model.predict(frame, imgsz=imgsz, conf=conf, device=device,
                                       retina_masks=True, verbose=False)[0]
                observation = trace_observation(observe_result(
                    result, self.stamp(message), message.header.frame_id))
                observation['processing_time_s'] = time.monotonic() - started
                observation['cpu_threads'] = torch.get_num_threads()
                return observation

        self.predictor = predictor
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='lane_inference')
        self.future = self.inflight = self.latest = None
        self.last_received = -1.
        self.publisher = self.create_publisher(String, 'lane/observation', 1)
        self.subscription = self.create_subscription(
            Image, topic, self.receive, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(.02, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def receive(self, message):
        stamp = self.stamp(message)
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if (not message.header.frame_id or not 0 <= age <= self.max_age
                or stamp <= self.last_received or message.width <= 0 or message.height <= 0):
            self.get_logger().warning('Dropped stale, unordered or invalid camera frame')
            return
        self.last_received = stamp
        self.latest = (message, time.monotonic())

    def fresh(self, item):
        message, received = item
        age = self.get_clock().now().nanoseconds / 1e9 - self.stamp(message)
        return 0 <= age <= self.max_age and 0 <= time.monotonic() - received <= self.max_age

    def poll(self):
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                if self.fresh(self.inflight):
                    if result['capture_time_s'] != self.stamp(self.inflight[0]):
                        raise ValueError('Inference changed capture timestamp')
                    self.publisher.publish(String(data=json.dumps(result, allow_nan=False)))
                else:
                    self.get_logger().warning('Dropped expired inference result')
            except Exception as error:
                self.get_logger().error(f'Inference failed; no observation published: {error}')
            self.future = self.inflight = None
        if self.future is None and self.latest is not None:
            item, self.latest = self.latest, None
            if self.fresh(item):
                self.inflight = item
                self.future = self.pool.submit(self.predictor, item[0])

    def destroy_node(self):
        self.timer.cancel()
        self.pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionNode()
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
