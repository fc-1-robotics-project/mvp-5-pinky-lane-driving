#!/usr/bin/env python3
# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Offline real-weight/video -> ROS observation check, NEVER motor publication."""

import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'pinky_lane_driving'))

import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from pinky_lane_driving.ros_perception import PerceptionNode


def main():
    config_path = ROOT / '.agents/replay.local.json'
    config = json.loads(config_path.read_text())
    weights = (config_path.parent / Path(config['model']).expanduser()).resolve()
    cases = list({c['video']: c for c in config['cases']}.values())
    if len(cases) < 2:
        raise ValueError('Validation requires both source drive videos')
    rclpy.init()
    namespace = f'/model_validation_{os.getpid()}'
    # This generous age is ONLY an offline test allowance for CPU/model warmup.
    node = PerceptionNode(namespace=namespace, parameter_overrides=[
        Parameter('model_path', value=str(weights)), Parameter('max_age_s', value=5.)])
    probe = rclpy.create_node('video_source', namespace=namespace)
    publisher = probe.create_publisher(Image, 'camera/image_raw',
                                       QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    received = []
    subscription = probe.create_subscription(String, 'lane/observation',
                                             lambda m: received.append(json.loads(m.data)), 1)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)

    def wait(predicate):
        end = time.monotonic() + 15.
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        raise RuntimeError('Real model ROS observation timeout')

    report = {'scope': 'offline observation transport only; no driving or live-rate acceptance',
              'validation_max_age_s': 5., 'cases': []}
    try:
        wait(lambda: publisher.get_subscription_count() == 1
             and node.publisher.get_subscription_count() == 1)
        for case in cases:
            video = (config_path.parent / Path(case['video']).expanduser()).resolve()
            cap = cv2.VideoCapture(str(video))
            try:
                cap.set(cv2.CAP_PROP_POS_MSEC, case['start_s'] * 1000)
                ok, frame = cap.read()
            finally:
                cap.release()
            if not ok:
                raise RuntimeError(f'Cannot read {video}')
            message = Image()
            message.header.stamp = probe.get_clock().now().to_msg()
            message.header.frame_id = 'camera_optical_frame'
            message.height, message.width = frame.shape[:2]
            message.encoding = 'bgr8'
            message.step = message.width * 3
            message.data = frame.tobytes()
            count = len(received)
            started = time.monotonic()
            publisher.publish(message)
            wait(lambda: len(received) > count)
            result = received[-1]
            assert result['capture_time_s'] == node.stamp(message)
            assert result['metric_valid'] is False
            report['cases'].append({'video': str(video), 'video_time_s': case['start_s'],
                                    'elapsed_s': time.monotonic() - started,
                                    'detections': len(result['detections']),
                                    'capture_time_preserved': True})
        with weights.open('rb') as stream:
            report['model_sha256'] = hashlib.file_digest(stream, 'sha256').hexdigest()
        report['status'] = 'execution_passed'
        base = ROOT / '.agents/output/ros_model'
        base.mkdir(parents=True, exist_ok=True)
        output = Path(tempfile.mkdtemp(prefix='run-', dir=base)) / 'report.json'
        output.write_text(json.dumps(report, indent=2) + '\n')
        print(output)
        print(json.dumps(report, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
