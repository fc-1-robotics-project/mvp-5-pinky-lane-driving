"""Robot-local velocity-source mux placed before the fleet safety gate."""

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .mode_policy import DriveModePolicy, MODE_LANE


def _twist_values(message: Twist) -> tuple:
    return (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    )


def _make_twist(values: tuple) -> Twist:
    message = Twist()
    (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    ) = values
    return message


class DriveModeMuxNode(Node):
    """Select Nav2, lane, or manual velocity without bypassing fleet safety."""

    def __init__(self) -> None:
        super().__init__('drive_mode_mux')
        self.declare_parameter('nav_input_topic', 'cmd_vel_nav_candidate')
        self.declare_parameter('lane_input_topic', 'cmd_vel_lane_candidate')
        self.declare_parameter('manual_input_topic', 'cmd_vel_manual_candidate')
        self.declare_parameter('output_topic', 'cmd_vel_candidate')
        self.declare_parameter('mode_request_topic', 'drive/mode_request')
        self.declare_parameter('mode_status_topic', 'drive/mode_status')
        self.declare_parameter('lane_finish_topic', 'lane/finish')
        self.declare_parameter('lane_completed_topic', 'lane/completed')
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('output_rate_hz', 20.0)
        self.declare_parameter('status_rate_hz', 5.0)

        timeout = self._positive_parameter('command_timeout_sec')
        output_rate = self._positive_parameter('output_rate_hz')
        status_rate = self._positive_parameter('status_rate_hz')
        self.policy = DriveModePolicy(timeout)
        self.last_finish = False
        self.last_status = None

        self.output_publisher = self.create_publisher(
            Twist, self._topic('output_topic'), 10,
        )
        self.status_publisher = self.create_publisher(
            String, self._topic('mode_status_topic'), 10,
        )
        self.lane_completed_publisher = self.create_publisher(
            Bool, self._topic('lane_completed_topic'), 10,
        )
        self.source_subscriptions = [
            self.create_subscription(
                Twist,
                self._topic('nav_input_topic'),
                lambda message: self._receive_command('NAV2', message),
                10,
            ),
            self.create_subscription(
                Twist,
                self._topic('lane_input_topic'),
                lambda message: self._receive_command('LANE', message),
                10,
            ),
            self.create_subscription(
                Twist,
                self._topic('manual_input_topic'),
                lambda message: self._receive_command('MANUAL', message),
                10,
            ),
            self.create_subscription(
                String,
                self._topic('mode_request_topic'),
                self._receive_mode,
                10,
            ),
            self.create_subscription(
                Bool,
                self._topic('lane_finish_topic'),
                self._receive_lane_finish,
                10,
            ),
        ]
        self.toggle_service = self.create_service(
            Trigger, 'drive/toggle_manual', self._toggle_manual,
        )
        self.output_timer = self.create_timer(
            1.0 / output_rate, self._publish_output,
        )
        self.status_timer = self.create_timer(
            1.0 / status_rate, self._publish_status,
        )
        self.get_logger().warning(
            'Drive mode mux started in STOP; select NAV2, LANE, or MANUAL.',
        )

    def _topic(self, parameter_name: str) -> str:
        value = self.get_parameter(parameter_name).value
        if not isinstance(value, str) or not value:
            raise ValueError(f'{parameter_name} must be a non-empty string')
        return value

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
        return value

    def _receive_command(self, source: str, message: Twist) -> None:
        if not self.policy.note_command(
            source, _twist_values(message), time.monotonic(),
        ):
            self.get_logger().warning(f'Rejected invalid {source} command')

    def _receive_mode(self, message: String) -> None:
        try:
            previous = self.policy.mode
            active = self.policy.request_mode(message.data, time.monotonic())
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        if active != previous:
            self.output_publisher.publish(Twist())
            if active == MODE_LANE:
                self.lane_completed_publisher.publish(Bool(data=False))
            self.get_logger().warning(f'Drive mode: {previous} -> {active}')

    def _receive_lane_finish(self, message: Bool) -> None:
        rising = message.data and not self.last_finish
        self.last_finish = message.data
        if not rising or self.policy.mode != MODE_LANE:
            return
        self.policy.request_mode('STOP', time.monotonic())
        self.output_publisher.publish(Twist())
        self.lane_completed_publisher.publish(Bool(data=True))
        self.get_logger().warning('Lane finish edge received; drive mode is STOP')

    def _toggle_manual(self, _request, response):
        previous = self.policy.mode
        active = self.policy.request_mode('TOGGLE_MANUAL', time.monotonic())
        self.output_publisher.publish(Twist())
        response.success = True
        response.message = f'{previous} -> {active}'
        return response

    def _publish_output(self) -> None:
        decision = self.policy.evaluate(time.monotonic())
        self.output_publisher.publish(_make_twist(decision.velocity))
        if decision.status != self.last_status:
            message = (
                f'Drive mux: mode={decision.mode} status={decision.status}'
            )
            if decision.output_enabled:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
            self.last_status = decision.status

    def _publish_status(self) -> None:
        decision = self.policy.evaluate(time.monotonic())
        payload = {
            'mode': decision.mode,
            'source': decision.source,
            'output_enabled': decision.output_enabled,
            'status': decision.status,
            'previous_autonomous_mode': self.policy.previous_autonomous_mode,
        }
        self.status_publisher.publish(
            String(data=json.dumps(payload, separators=(',', ':'))),
        )

    def stop(self) -> None:
        self.policy.request_mode('STOP', time.monotonic())
        self.output_publisher.publish(Twist())


def main(args=None) -> None:
    """Run the robot-local drive-mode mux."""
    rclpy.init(args=args)
    node = DriveModeMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.context.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
