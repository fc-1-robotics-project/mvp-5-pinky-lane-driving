"""ROS node that owns the final cmd_vel output on a fleet robot."""

import time

from geometry_msgs.msg import Twist
from pinky_interfaces.msg import FleetPermit, RobotHeartbeat
from pinky_interfaces.srv import SetLed
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy

from .gate_policy import LED_HOLD, LED_RUN, LED_STOP
from .gate_policy import led_state_for_gate, VelocityGatePolicy


class VelocityGateNode(Node):
    """Pass velocity commands only while a fresh RUN permit exists."""

    def __init__(self) -> None:
        super().__init__('fleet_velocity_gate')
        self.declare_parameter('robot_id', 'robot')
        self.declare_parameter('input_cmd_vel_topic', 'cmd_vel_candidate')
        self.declare_parameter('output_cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('permit_topic', 'fleet/permit')
        self.declare_parameter('heartbeat_topic', 'fleet/heartbeat')
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('max_permit_ttl_sec', 1.0)
        self.declare_parameter('output_rate_hz', 20.0)
        self.declare_parameter('heartbeat_rate_hz', 5.0)
        self.declare_parameter('led_enabled', True)
        self.declare_parameter('led_service_topic', '/set_led')
        self.declare_parameter('led_run_rgb', [0, 255, 0])
        self.declare_parameter('led_hold_rgb', [255, 180, 0])
        self.declare_parameter('led_stop_rgb', [255, 0, 0])

        robot_id = self._string_parameter('robot_id')
        input_topic = self._string_parameter('input_cmd_vel_topic')
        output_topic = self._string_parameter('output_cmd_vel_topic')
        permit_topic = self._string_parameter('permit_topic')
        heartbeat_topic = self._string_parameter('heartbeat_topic')
        command_timeout = self._positive_parameter('command_timeout_sec')
        permit_ttl = self._positive_parameter('max_permit_ttl_sec')
        output_rate = self._positive_parameter('output_rate_hz')
        heartbeat_rate = self._positive_parameter('heartbeat_rate_hz')
        self.led_enabled = bool(self.get_parameter('led_enabled').value)
        led_service_topic = self._string_parameter('led_service_topic')
        self.led_colours = {
            LED_RUN: self._rgb_parameter('led_run_rgb'),
            LED_HOLD: self._rgb_parameter('led_hold_rgb'),
            LED_STOP: self._rgb_parameter('led_stop_rgb'),
        }

        self.policy = VelocityGatePolicy(
            robot_id,
            command_timeout,
            permit_ttl,
        )
        self.latest_command = Twist()
        self.last_status = 'NO_PERMIT'
        self.desired_led_state = LED_STOP
        self.applied_led_state = None
        self.led_request_in_flight = False
        self.led_service_wait_logged = False

        permit_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_publisher = self.create_publisher(Twist, output_topic, 10)
        self.heartbeat_publisher = self.create_publisher(
            RobotHeartbeat,
            heartbeat_topic,
            10,
        )
        self.command_subscription = self.create_subscription(
            Twist,
            input_topic,
            self._command_callback,
            10,
        )
        self.permit_subscription = self.create_subscription(
            FleetPermit,
            permit_topic,
            self._permit_callback,
            permit_qos,
        )
        self.led_client = None
        if self.led_enabled:
            self.led_client = self.create_client(SetLed, led_service_topic)
        self.output_timer = self.create_timer(
            1.0 / output_rate,
            self._publish_output,
        )
        self.heartbeat_timer = self.create_timer(
            1.0 / heartbeat_rate,
            self._publish_heartbeat,
        )
        self.get_logger().warning(
            f'Velocity gate started fail-closed for {robot_id}; '
            f'{input_topic} -> {output_topic}',
        )

    def publish_stop(self) -> None:
        """Publish an explicit zero velocity command."""
        self.command_publisher.publish(Twist())

    def _command_callback(self, message: Twist) -> None:
        self.latest_command = message
        self.policy.note_command(time.monotonic())

    def _permit_callback(self, message: FleetPermit) -> None:
        ttl_sec = message.ttl.sec + message.ttl.nanosec / 1e9
        accepted = self.policy.accept_permit(
            robot_id=message.robot_id,
            controller_id=message.controller_id,
            sequence=message.sequence,
            mode=message.mode,
            ttl_sec=ttl_sec,
            lease_id=message.lease_id,
            now=time.monotonic(),
        )
        if not accepted:
            self.get_logger().debug('Ignored an invalid or stale fleet permit.')

    def _publish_output(self) -> None:
        decision = self.policy.evaluate(time.monotonic())
        if decision.output_enabled:
            self.command_publisher.publish(self.latest_command)
        else:
            self.publish_stop()

        if decision.status != self.last_status:
            if decision.output_enabled:
                self.get_logger().info(
                    f'Velocity gate state: {decision.status}',
                )
            else:
                self.get_logger().warning(
                    f'Velocity gate state: {decision.status}',
                )
            self.last_status = decision.status
        self._update_led(decision.permit_fresh)

    def _publish_heartbeat(self) -> None:
        decision = self.policy.evaluate(time.monotonic())
        message = RobotHeartbeat()
        message.header.stamp = self.get_clock().now().to_msg()
        message.robot_id = self.policy.robot_id
        message.gate_mode = self.policy.mode
        message.permit_fresh = decision.permit_fresh
        message.command_fresh = decision.command_fresh
        message.output_enabled = decision.output_enabled
        message.status = decision.status
        message.controller_id = self.policy.controller_id
        message.last_permit_sequence = self.policy.last_permit_sequence
        message.active_lease_id = self.policy.active_lease_id
        self.heartbeat_publisher.publish(message)

    def _update_led(self, permit_fresh: bool) -> None:
        """Request an LED colour only when the gate state changes."""
        if not self.led_enabled or self.led_client is None:
            return
        self.desired_led_state = led_state_for_gate(
            self.policy.mode,
            permit_fresh,
        )
        if (
            self.desired_led_state == self.applied_led_state
            or self.led_request_in_flight
        ):
            return
        if not self.led_client.service_is_ready():
            if not self.led_service_wait_logged:
                self.get_logger().warning(
                    'Waiting for LED service; gate operation is unaffected.',
                )
                self.led_service_wait_logged = True
            return

        self.led_service_wait_logged = False
        requested_state = self.desired_led_state
        red, green, blue = self.led_colours[requested_state]
        request = SetLed.Request()
        request.command = 'fill'
        request.r = red
        request.g = green
        request.b = blue
        self.led_request_in_flight = True
        future = self.led_client.call_async(request)
        future.add_done_callback(
            lambda result: self._led_response_callback(
                result,
                requested_state,
            ),
        )

    def _led_response_callback(self, future, requested_state: str) -> None:
        """Record a successful LED request and leave failures retryable."""
        self.led_request_in_flight = False
        try:
            response = future.result()
        except Exception as error:  # ROS future exception types vary.
            self.get_logger().warning(f'LED service call failed: {error}')
            return
        if not response.success:
            self.get_logger().warning(
                f'LED service rejected state {requested_state}: '
                f'{response.message}',
            )
            return
        self.applied_led_state = requested_state
        colour = self.led_colours[requested_state]
        self.get_logger().info(
            f'Fleet LED state: {requested_state} rgb={colour}',
        )

    def _string_parameter(self, name: str) -> str:
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value:
            raise ValueError(f'{name} must be a non-empty string')
        return value

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
        return value

    def _rgb_parameter(self, name: str):
        values = self.get_parameter(name).value
        if len(values) != 3:
            raise ValueError(f'{name} must contain exactly three values')
        colour = tuple(int(value) for value in values)
        if any(value < 0 or value > 255 for value in colour):
            raise ValueError(f'{name} values must be between 0 and 255')
        return colour


def main(args=None) -> None:
    """Run the fail-closed robot velocity gate."""
    rclpy.init(args=args)
    node = VelocityGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.context.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
