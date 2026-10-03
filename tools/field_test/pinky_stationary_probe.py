#!/usr/bin/env python3
"""Read-only 20-second Pinky ROS_DOMAIN_ID=21 stationary readiness probe.

Run on the robot after sourcing ROS and the robot install/setup.bash.  This
node creates subscriptions and GetParameters clients only.  It never publishes
or calls a motion-enabling service.  Output is one JSON object on stdout.
"""

import argparse
from collections import Counter
import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String


def finite_round(value, places=4):
    if isinstance(value, (int, float)) and math.isfinite(value):
        return round(float(value), places)
    return None


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def parse_json(message):
    try:
        value = json.loads(message.data)
        return value if isinstance(value, dict) else {}
    except (ValueError, TypeError):
        return {}


def parameter_value(value):
    # rcl_interfaces/msg/ParameterType numeric constants.  Unknown types stay
    # visible as an explicit type instead of accidentally becoming 0 or False.
    kind = value.type
    if kind == 1:
        return value.bool_value
    if kind == 2:
        return value.integer_value
    if kind == 3:
        return value.double_value
    if kind == 4:
        return value.string_value
    if kind == 5:
        return list(value.byte_array_value)
    if kind == 6:
        return list(value.bool_array_value)
    if kind == 7:
        return list(value.integer_array_value)
    if kind == 8:
        return list(value.double_array_value)
    if kind == 9:
        return list(value.string_array_value)
    return {'type': kind}


class StationaryProbe(Node):
    PARAMETER_REQUESTS = {
        'lane_perception': ['imgsz', 'confidence', 'max_age_s', 'cpu_threads'],
        'lane_watchdog': ['dry_run', 'hardware_watchdog_confirmed',
                          'command_timeout_s', 'max_source_age_s',
                          'max_speed_mps', 'output_topic'],
        'lane_safety': ['start_enabled'],
        'lane_control': ['config_path'],
    }

    def __init__(self):
        super().__init__('pinky_stationary_probe')
        self.started = time.monotonic()
        self.topics = {}
        self.latest = {}
        self.observation_classes = Counter()
        self.observation_trace_status = Counter()
        self.lane_reasons = Counter()
        self.diagnostic_reasons = Counter()
        self.diagnostic_faults = Counter()
        self.mode_counts = Counter()
        self.command_reasons = Counter()
        self.nonzero = Counter()
        self.max_abs_cmd_vel = 0.0
        self.max_abs_lane_command = 0.0
        self.max_abs_odom_speed = 0.0
        self.parameters = {}
        self.parameter_clients = {}
        self.parameter_futures = {}
        self.parameter_sent = False
        self.create_subscription(Image, '/camera/image_raw', self.on_camera,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/lane/observation', self.on_observation,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/lane/diagnostics', self.on_diagnostic,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/drive/mode_status', self.on_mode,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/lane/command', self.on_command,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, '/lane/estop', self.on_estop,
                                 qos_profile_sensor_data)
        for name in self.PARAMETER_REQUESTS:
            self.parameter_clients[name] = self.create_client(
                GetParameters, f'/{name}/get_parameters')

    def mark(self, name, stamp=None):
        now = time.monotonic()
        item = self.topics.setdefault(name, {
            'count': 0, 'first_rx': now, 'last_rx': now,
            'stamp_s': None,
        })
        item['count'] += 1
        item['last_rx'] = now
        if stamp is not None:
            item['stamp_s'] = stamp

    def on_camera(self, message):
        self.mark('camera', stamp_seconds(message.header.stamp))
        self.latest['camera'] = {
            'width': message.width, 'height': message.height,
            'encoding': message.encoding, 'step': message.step,
            'frame_id': message.header.frame_id,
        }

    def on_observation(self, message):
        payload = parse_json(message)
        self.mark('observation', payload.get('capture_time_s'))
        detections = payload.get('detections', [])
        if not isinstance(detections, list):
            detections = []
        counts = Counter()
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            class_id = detection.get('class_id')
            counts[str(class_id)] += 1
            self.observation_classes[str(class_id)] += 1
            self.observation_trace_status[str(detection.get('trace_status'))] += 1
        self.latest['observation'] = {
            'image_size': payload.get('image_size'),
            'frame_id': payload.get('frame_id'),
            'metric_valid': payload.get('metric_valid'),
            'detection_counts_by_class_id': dict(counts),
            'processing_time_s': finite_round(payload.get('processing_time_s')),
            'cpu_threads': payload.get('cpu_threads'),
        }

    def on_diagnostic(self, message):
        payload = parse_json(message)
        self.mark('diagnostics')
        reason = str(payload.get('reason', 'missing'))
        lane_reason = str(payload.get('lane_reason', 'missing'))
        self.diagnostic_reasons[reason] += 1
        self.lane_reasons[lane_reason] += 1
        if payload.get('fault'):
            self.diagnostic_faults[str(payload['fault'])] += 1
        path = payload.get('path_points_m', [])
        if not isinstance(path, list):
            path = []
        path_length = 0.0
        for first, second in zip(path, path[1:]):
            try:
                path_length += math.dist(first, second)
            except (TypeError, ValueError):
                path_length = math.nan
                break
        self.latest['diagnostics'] = {
            'reason': reason,
            'lane_reason': lane_reason,
            'lane_valid': payload.get('lane_valid'),
            'lane_degraded': payload.get('lane_degraded'),
            'path_points': len(path),
            'path_length_m': finite_round(path_length),
            'capture_age_s': finite_round(payload.get('capture_age_s')),
            'observation_error': payload.get('observation_error'),
            'sensor_error': payload.get('sensor_error'),
            'odom_age_s': finite_round(payload.get('odom_age_s')),
            'scan_age_s': finite_round(payload.get('scan_age_s')),
            'scan_coverage_ok': payload.get('scan_coverage_ok'),
            'scan_hit': payload.get('scan_hit'),
            'emergency': payload.get('emergency'),
            'fault': payload.get('fault'),
        }

    def on_scan(self, message):
        self.mark('scan', stamp_seconds(message.header.stamp))
        n = len(message.ranges)
        inc = abs(message.angle_increment)
        span = inc * (n - 1)
        coverage = (
            n > 1 and 0 < inc <= 0.03
            and 2 * math.pi - 2 * inc <= span <= 2 * math.pi + inc
            and math.isfinite(message.angle_max)
            and abs(message.angle_max - message.angle_min
                    - message.angle_increment * (n - 1)) < 1e-4
        )
        self.latest['scan'] = {
            'frame_id': message.header.frame_id,
            'samples': n,
            'angle_span_rad': finite_round(span),
            'angle_increment_rad': finite_round(message.angle_increment, 6),
            'coverage_ok_at_config_0_03_rad': coverage,
            'finite_in_range_returns': sum(
                math.isfinite(x) and message.range_min <= x <= message.range_max
                for x in message.ranges),
            'infinity_returns': sum(x == math.inf for x in message.ranges),
        }

    def on_odom(self, message):
        self.mark('odom', stamp_seconds(message.header.stamp))
        speed = float(message.twist.twist.linear.x)
        if math.isfinite(speed):
            self.max_abs_odom_speed = max(self.max_abs_odom_speed, abs(speed))
        self.latest['odom'] = {
            'frame_id': message.header.frame_id,
            'child_frame_id': message.child_frame_id,
            'speed_mps': finite_round(speed),
            'x_m': finite_round(message.pose.pose.position.x),
            'y_m': finite_round(message.pose.pose.position.y),
        }

    def on_mode(self, message):
        payload = parse_json(message)
        self.mark('mode_status')
        mode = str(payload.get('mode', 'missing'))
        self.mode_counts[mode] += 1
        self.latest['mode_status'] = payload

    def on_cmd_vel(self, message):
        self.mark('cmd_vel')
        values = [message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z]
        magnitude = max(abs(float(x)) for x in values)
        if math.isfinite(magnitude):
            self.max_abs_cmd_vel = max(self.max_abs_cmd_vel, magnitude)
        if not math.isfinite(magnitude) or magnitude > 1e-4:
            self.nonzero['cmd_vel'] += 1
        self.latest['cmd_vel'] = {
            'linear_x': finite_round(message.linear.x),
            'angular_z': finite_round(message.angular.z),
        }

    def on_command(self, message):
        payload = parse_json(message)
        self.mark('lane_command', payload.get('capture_stamp'))
        reason = str(payload.get('reason', 'missing'))
        self.command_reasons[reason] += 1
        speed, omega = payload.get('speed'), payload.get('omega')
        try:
            magnitude = max(abs(float(speed)), abs(float(omega)))
        except (TypeError, ValueError):
            magnitude = math.nan
        if math.isfinite(magnitude):
            self.max_abs_lane_command = max(self.max_abs_lane_command, magnitude)
        if not math.isfinite(magnitude) or magnitude > 1e-4:
            self.nonzero['lane_command'] += 1
        self.latest['lane_command'] = {
            'speed_mps': finite_round(speed),
            'omega_radps': finite_round(omega),
            'reason': reason,
        }

    def on_estop(self, message):
        self.mark('lane_estop')
        if message.data is not True:
            self.nonzero['estop_false'] += 1
        self.latest['lane_estop'] = message.data

    def request_parameters(self):
        self.parameter_sent = True
        for name, names in self.PARAMETER_REQUESTS.items():
            client = self.parameter_clients[name]
            if not client.service_is_ready():
                self.parameters[name] = {'error': 'service unavailable'}
                continue
            self.parameter_futures[name] = client.call_async(
                GetParameters.Request(names=names))

    def collect_parameters(self):
        for name, future in list(self.parameter_futures.items()):
            if not future.done():
                continue
            try:
                values = future.result().values
                names = self.PARAMETER_REQUESTS[name]
                self.parameters[name] = {
                    key: parameter_value(value)
                    for key, value in zip(names, values)
                }
            except Exception as error:
                self.parameters[name] = {'error': str(error)}
            del self.parameter_futures[name]

    def summarize(self, duration_s):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds / 1e9
        topics = {}
        for name, item in self.topics.items():
            received_span = item['last_rx'] - item['first_rx']
            topics[name] = {
                'messages': item['count'],
                'rate_hz': finite_round((item['count'] - 1) / received_span, 2)
                if received_span > 0 else None,
                'last_rx_age_s': finite_round(now - item['last_rx']),
                'last_stamp_age_s': finite_round(ros_now - item['stamp_s'])
                if isinstance(item['stamp_s'], (int, float)) else None,
            }
        expected = ('camera', 'observation', 'diagnostics', 'scan', 'odom',
                    'mode_status', 'cmd_vel', 'lane_command', 'lane_estop')
        missing = [name for name in expected if name not in topics]
        camera = self.latest.get('camera', {})
        observation = self.latest.get('observation', {})
        diagnostic = self.latest.get('diagnostics', {})
        scan = self.latest.get('scan', {})
        odom = self.latest.get('odom', {})
        stop_checks = {
            'all_mode_samples_STOP': bool(self.mode_counts)
            and set(self.mode_counts) == {'STOP'},
            'all_cmd_vel_samples_zero': topics.get('cmd_vel', {}).get('messages', 0) > 0
            and self.nonzero['cmd_vel'] == 0,
            'all_lane_commands_zero': topics.get('lane_command', {}).get('messages', 0) > 0
            and self.nonzero['lane_command'] == 0,
            'all_local_estop_samples_true': topics.get('lane_estop', {}).get('messages', 0) > 0
            and self.nonzero['estop_false'] == 0,
        }
        readiness = {
            'camera_640x480': camera.get('width') == 640 and camera.get('height') == 480,
            'perception_imgsz_448': self.parameters.get('lane_perception', {}).get('imgsz') == 448,
            'observation_640x480': observation.get('image_size') == [640, 480],
            'lane_valid': diagnostic.get('lane_valid') is True,
            'scan_coverage': scan.get('coverage_ok_at_config_0_03_rad') is True
            and diagnostic.get('scan_coverage_ok') is True,
            'odom_frames': odom.get('frame_id') == 'odom'
            and odom.get('child_frame_id') == 'base_footprint',
            'watchdog_max_speed_0_03': self.parameters.get('lane_watchdog', {}).get('max_speed_mps') == 0.03,
            'watchdog_command_timeout_0_2': self.parameters.get('lane_watchdog', {}).get('command_timeout_s') == 0.2,
            'safety_start_disabled': self.parameters.get('lane_safety', {}).get('start_enabled') is False,
        }
        return {
            'probe': 'read_only_stationary',
            'domain_id': 21,
            'duration_s': finite_round(duration_s, 2),
            'missing_topics': missing,
            'topics': topics,
            'camera': camera,
            'observation': observation,
            'observation_classes_total': dict(self.observation_classes),
            'observation_trace_status_total': dict(self.observation_trace_status),
            'diagnostics': diagnostic,
            'diagnostic_reasons': dict(self.diagnostic_reasons),
            'lane_reasons': dict(self.lane_reasons),
            'diagnostic_faults': dict(self.diagnostic_faults),
            'scan': scan,
            'odom': odom,
            'mode_status': self.latest.get('mode_status'),
            'mode_counts': dict(self.mode_counts),
            'cmd_vel': self.latest.get('cmd_vel'),
            'lane_command': self.latest.get('lane_command'),
            'lane_estop': self.latest.get('lane_estop'),
            'max_abs_cmd_vel_component': finite_round(self.max_abs_cmd_vel),
            'max_abs_lane_command_component': finite_round(self.max_abs_lane_command),
            'max_abs_odom_speed_mps': finite_round(self.max_abs_odom_speed),
            'nonzero_or_invalid_samples': dict(self.nonzero),
            'stop_checks': stop_checks,
            'readiness_checks': readiness,
            'parameters': self.parameters,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=20.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration_s) or not 1 <= args.duration_s <= 120:
        parser.error('--duration-s must be within 1..120')
    rclpy.init(domain_id=21)
    node = StationaryProbe()
    try:
        deadline = time.monotonic() + args.duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.parameter_sent and time.monotonic() - node.started >= 1.0:
                node.request_parameters()
            node.collect_parameters()
        node.collect_parameters()
        for name in node.parameter_futures:
            node.parameters[name] = {'error': 'GetParameters timeout'}
        print(json.dumps(node.summarize(time.monotonic() - node.started),
                         allow_nan=False, separators=(',', ':')), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
