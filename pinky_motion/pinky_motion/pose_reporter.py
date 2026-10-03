"""Republish AMCL's latest Pinky pose at a fixed interval."""

import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class PoseReporter(Node):
    """Relay the latest AMCL pose for the control-room display."""

    def __init__(self):
        super().__init__('pinky_pose_reporter')

        self.declare_parameter('amcl_topic', 'amcl_pose')
        self.declare_parameter('pose_topic', 'pinky/pose')
        self.declare_parameter('report_period_s', 0.5)

        amcl_topic = str(self.get_parameter('amcl_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        period = float(self.get_parameter('report_period_s').value)
        if period <= 0.0:
            raise ValueError('report_period_s must be > 0')

        self.latest_pose = None
        self.last_pose_at = None
        self.last_warning_at = 0.0
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        # AMCL의 마지막 pose를 늦게 시작한 reporter도 받을 수 있도록 한다.
        amcl_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self.pose_callback,
            amcl_qos,
        )
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Republishing {amcl_topic} to {pose_topic} every {period:.2f} s')

    def pose_callback(self, message: PoseWithCovarianceStamped):
        """Store the newest AMCL pose."""
        # AMCL이 map frame 기준으로 계산한 최신 위치를 저장한다.
        self.latest_pose = message
        self.last_pose_at = time.monotonic()

    def timer_callback(self):
        """Publish the newest pose at the configured interval."""
        if self.latest_pose is None:
            now = time.monotonic()
            if now - self.last_warning_at >= 2.0:
                self.get_logger().warning(
                    'No /amcl_pose received yet; set the initial pose first')
                self.last_warning_at = now
            return

        # PoseWithCovarianceStamped에서 화면 표시용 PoseStamped로 변환한다.
        pose = PoseStamped()
        pose.header = self.latest_pose.header
        pose.pose = self.latest_pose.pose.pose
        self.pose_pub.publish(pose)


def main(args=None):
    """Run the Pinky AMCL pose reporter."""
    rclpy.init(args=args)
    node = None
    try:
        node = PoseReporter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
