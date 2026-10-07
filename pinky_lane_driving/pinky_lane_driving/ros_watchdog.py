# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Independent robot-side watchdog node; default output cannot reach cmd_vel."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String

from .watchdog import Watchdog


class WatchdogNode(Node):
    def __init__(self, **kwargs):
        super().__init__('lane_watchdog', **kwargs)

        def parameter(name, default):
            return self.declare_parameter(name, default,
                                          ParameterDescriptor(read_only=True)).value

        dry_run = parameter('dry_run', True)
        confirmed = parameter('hardware_watchdog_confirmed', False)
        if not dry_run and not confirmed:
            raise ValueError('Hardware mode requires verified lower-level motor watchdog')
        self.guard = Watchdog(timeout=parameter('command_timeout_s', .2),
                              max_source_age=parameter('max_source_age_s', .3),
                              max_speed=parameter('max_speed_mps', .2),
                              max_omega=parameter('max_omega_radps', .8), enabled=True)
        # Hardware mode is an explicit deployment choice, never enabled by a
        # message. It still feeds the robot-local mode mux rather than /cmd_vel.
        output_topic = parameter('output_topic', 'cmd_vel_lane_candidate')
        if not isinstance(output_topic, str) or not output_topic:
            raise ValueError('output_topic must be a non-empty string')
        topic = 'lane/dry_run_cmd_vel' if dry_run else output_topic
        self.publisher = self.create_publisher(Twist, topic, 1)
        self.subscription = self.create_subscription(String, 'lane/command', self.receive, 1)
        self.timer = self.create_timer(.02, self.publish,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Watchdog output: {topic}; dry_run={dry_run}')

    def receive(self, message):
        try:
            if len(message.data) > 4096:
                raise ValueError('Oversized command')
            payload = json.loads(message.data)
            required = {'sequence', 'capture_stamp', 'speed', 'omega'}
            if set(payload) not in (required, required | {'reason'}):
                raise ValueError('Unexpected command fields')
            if 'reason' in payload and not isinstance(payload['reason'], str):
                raise ValueError('Invalid command reason')
            source_age = self.get_clock().now().nanoseconds / 1e9 - payload['capture_stamp']
            if not self.guard.receive(payload['sequence'], payload['speed'], payload['omega'],
                                      now=time.monotonic(), source_age=source_age):
                self.get_logger().warning('Rejected command: age, sequence or bounds')
        except (ValueError, TypeError, KeyError, OverflowError):
            self.guard.stop()
            self.get_logger().warning('Rejected malformed command')

    def publish(self):
        speed, omega = self.guard.output(time.monotonic())
        message = Twist()
        message.linear.x = speed
        message.angular.z = omega
        self.publisher.publish(message)

    def stop(self):
        self.guard.stop()
        self.publish()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WatchdogNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
