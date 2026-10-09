"""Publish a fleet pose preserving the robot-local localization TF time."""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def pose_from_transform(
    transform: TransformStamped,
    covariance,
) -> PoseWithCovarianceStamped:
    """Build a map-frame pose using a TF transform and AMCL covariance."""
    message = PoseWithCovarianceStamped()
    message.header.stamp = transform.header.stamp
    message.header.frame_id = transform.header.frame_id
    message.pose.pose.position.x = transform.transform.translation.x
    message.pose.pose.position.y = transform.transform.translation.y
    message.pose.pose.position.z = transform.transform.translation.z
    message.pose.pose.orientation = transform.transform.rotation
    message.pose.covariance = list(covariance)
    return message


def source_nanoseconds(stamp):
    """Reject malformed or unset source times before comparing clock ages."""
    if stamp is None or stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        return None
    value = stamp.sec * 1000000000 + stamp.nanosec
    return value if value > 0 else None


def valid_covariance(covariance):
    return (len(covariance) == 36 and all(math.isfinite(v) for v in covariance)
            and all(covariance[i] >= 0 for i in range(0, 36, 7)))


class FleetPoseReporter(Node):
    """Republish current localization TF at a fixed rate for fleet control."""

    def __init__(self) -> None:
        super().__init__('fleet_pose_reporter')
        self.declare_parameter('source_pose_topic', 'amcl_pose')
        self.declare_parameter('output_pose_topic', 'fleet/pose')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('transform_timeout_sec', 0.05)
        self.declare_parameter('transform_stale_sec', 2.0)

        source_topic = self._string_parameter('source_pose_topic')
        output_topic = self._string_parameter('output_pose_topic')
        self.global_frame = self._string_parameter('global_frame')
        self.robot_base_frame = self._string_parameter('robot_base_frame')
        publish_rate = self._positive_parameter('publish_rate_hz')
        timeout = self._positive_parameter('transform_timeout_sec')
        self.transform_stale_sec = self._positive_parameter(
            'transform_stale_sec',
        )

        self.transform_timeout = Duration(seconds=timeout)
        self.latest_covariance = None
        self.latest_covariance_stamp = None
        self.last_status = ''
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            output_topic,
            10,
        )
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            source_topic,
            self._pose_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_pose)
        self.get_logger().info(
            f'Fleet pose reporter started: {self.global_frame} -> '
            f'{self.robot_base_frame} to {output_topic} at {publish_rate:.1f} Hz',
        )

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        stamp = source_nanoseconds(message.header.stamp)
        previous = source_nanoseconds(self.latest_covariance_stamp)
        if (message.header.frame_id != self.global_frame or stamp is None
                or stamp > self.get_clock().now().nanoseconds
                or previous is not None and stamp < previous
                or not valid_covariance(message.pose.covariance)):
            self.latest_covariance = self.latest_covariance_stamp = None
            self._report_status('AMCL_POSE_INVALID')
            return
        self.latest_covariance = list(message.pose.covariance)
        self.latest_covariance_stamp = message.header.stamp

    def _publish_pose(self) -> None:
        if self.latest_covariance is None:
            self._report_status('WAITING_FOR_AMCL_POSE')
            return
        now = self.get_clock().now().nanoseconds
        covariance_stamp = source_nanoseconds(self.latest_covariance_stamp)
        if (covariance_stamp is None or covariance_stamp > now
                or not valid_covariance(self.latest_covariance)):
            self._report_status('AMCL_POSE_INVALID')
            return
        # AMCL covariance is the last filter measurement, not a 5 Hz source.
        # A stationary robot may keep it until motion crosses update_min_*.
        # Its source time is validated, but no fixed receive lease is imposed.
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
                timeout=self.transform_timeout,
            )
        except TransformException:
            self._report_status('LOCALIZATION_TF_UNAVAILABLE')
            return

        now = self.get_clock().now().nanoseconds
        transform_stamp = source_nanoseconds(transform.header.stamp)
        if (transform_stamp is None
                or not 0 <= (now - transform_stamp) / 1e9 <= self.transform_stale_sec):
            self._report_status('LOCALIZATION_TF_STALE')
            return

        message = pose_from_transform(
            transform,
            self.latest_covariance,
        )
        self.pose_publisher.publish(message)
        self._report_status('REPORTING')

    def _report_status(self, status: str) -> None:
        if status == self.last_status:
            return
        if status == 'REPORTING':
            self.get_logger().info('Fleet pose reporting is active.')
        else:
            self.get_logger().warning(f'Fleet pose reporter: {status}')
        self.last_status = status

    def _string_parameter(self, name: str) -> str:
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value:
            raise ValueError(f'{name} must be a non-empty string')
        return value

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
        return value


def main(args=None) -> None:
    """Run the fleet pose reporter."""
    rclpy.init(args=args)
    node = FleetPoseReporter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
