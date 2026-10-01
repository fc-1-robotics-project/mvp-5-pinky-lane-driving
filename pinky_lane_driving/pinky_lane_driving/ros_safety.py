# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Local operator motion latch; starts and fails in the stopped state."""

import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class SafetyNode(Node):
    """Publish a fresh E-STOP heartbeat controlled by an explicit service call."""

    def __init__(self, **kwargs):
        super().__init__('lane_safety', **kwargs)
        rate = self.declare_parameter(
            'publish_rate_hz', 10.0, ParameterDescriptor(read_only=True)
        ).value
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz must be finite and positive')
        start_enabled = self.declare_parameter(
            'start_enabled', False, ParameterDescriptor(read_only=True)
        ).value
        if type(start_enabled) is not bool:
            raise ValueError('start_enabled must be boolean')
        self.enabled = start_enabled
        self.publisher = self.create_publisher(Bool, 'lane/estop', 1)
        self.service = self.create_service(
            SetBool, 'lane/set_enabled', self.set_enabled
        )
        self.timer = self.create_timer(
            1.0 / rate, self.publish,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        state = 'enabled' if self.enabled else 'disabled'
        self.get_logger().warning(
            f'Lane motion starts {state}; enable it only after verifying '
            'dry-run output and the physical emergency stop.'
        )

    def set_enabled(self, request, response):
        self.enabled = request.data is True
        self.publish()
        response.success = True
        response.message = (
            'lane motion enabled' if self.enabled else 'lane motion stopped'
        )
        if self.enabled:
            self.get_logger().warning('Lane motion ENABLED by local operator')
        else:
            self.get_logger().warning('Lane motion stopped by local operator')
        return response

    def publish(self):
        self.publisher.publish(Bool(data=not self.enabled))

    def stop(self):
        self.enabled = False
        self.publish()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyNode()
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
