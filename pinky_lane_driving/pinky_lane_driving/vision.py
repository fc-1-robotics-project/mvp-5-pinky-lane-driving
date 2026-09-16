# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Shared YOLO observation adapter for offline replay and live ROS images."""

from .perception import observe
from .tracing import trace_polygon

CLASSES = {0: 'crosswalk', 1: 'left line', 2: 'right line'}


def validate_model(task, names):
    if task != 'segment' or names != CLASSES:
        raise ValueError(f'Expected segment with {CLASSES}; got {task}: {names}')


def observe_result(result, capture_time_s, frame_id='camera_optical_frame'):
    ids = [int(i) for i in result.boxes.cls.tolist()]
    scores = result.boxes.conf.tolist()
    polygons = [] if result.masks is None else [p.tolist() for p in result.masks.xy]
    height, width = result.orig_shape
    return observe(capture_time_s, frame_id, (width, height), ids, scores, polygons)


def trace_observation(observation):
    for detection in observation['detections']:
        if detection['class_id'] == 0:
            continue
        try:
            detection['boundary_px'] = trace_polygon(detection['polygon_px'], observation['image_size'])
            detection['trace_status'] = 'pixel_only'
        except ValueError as error:
            detection['trace_status'] = 'invalid'
            detection['trace_error'] = str(error)
    return observation


def decode_image(message):
    """Decode uint8 ROS images with row stride; no cv_bridge binary ABI dependency."""
    import numpy as np

    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(message.encoding)
    if (channels is None or message.width <= 0 or message.height <= 0
            or message.width * message.height > 16_777_216
            or message.step < message.width * channels
            or len(message.data) != message.height * message.step):
        raise ValueError('Unsupported encoding, size or row stride')
    image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    image = image[:, :message.width * channels].reshape(message.height, message.width, channels)
    if message.encoding == 'rgb8':
        image = image[:, :, ::-1]
    if channels == 1:
        image = np.repeat(image, 3, axis=2)
    return np.ascontiguousarray(image)
