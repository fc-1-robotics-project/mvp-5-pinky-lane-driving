"""Subscribe to Pinky's reported pose for a control-room status display."""

import math
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PoseMonitor(Node):
    """Print the latest reported Pinky pose and detect stale updates."""

    def __init__(self):
        super().__init__('pinky_pose_monitor')
        # 관제 PC에서는 domain_bridge가 전달한 PoseStamped를 구독한다.
        self.declare_parameter('pose_topic', 'pinky/pose')
        self.declare_parameter('stale_after_s', 2.0)

        topic = str(self.get_parameter('pose_topic').value)
        stale_after = float(self.get_parameter('stale_after_s').value)
        if stale_after <= 0.0:
            raise ValueError('stale_after_s must be > 0')

        self.last_message_at = None
        self.stale_after = stale_after
        self.subscription = self.create_subscription(
            PoseStamped, topic, self.pose_callback, 10)
        self.timer = self.create_timer(1.0, self.stale_callback)
        self.get_logger().info(
            f'Subscribing to {topic}; stale after {stale_after:.1f} s')

    def pose_callback(self, message: PoseStamped):
        """Print each fresh pose received from the robot."""
        # 수신 시각을 기록해 통신이 끊겼을 때 stale 상태를 판단한다.
        self.last_message_at = time.monotonic()
        position = message.pose.position
        orientation = message.pose.orientation
        yaw = math.atan2(
            2.0 * orientation.w * orientation.z,
            1.0 - 2.0 * orientation.z * orientation.z)
        # quaternion에서 2D 주행 방향인 yaw를 라디안에서 도 단위로 변환한다.
        self.get_logger().info(
            f'frame={message.header.frame_id} '
            f'x={position.x:.3f} m y={position.y:.3f} m '
            f'z={position.z:.3f} m yaw={math.degrees(yaw):.1f} deg')

    def stale_callback(self):
        """Warn when the robot has stopped reporting its position."""
        # 일정 시간 동안 메시지가 없으면 위치 정보가 최신 상태가 아님을 알린다.
        if self.last_message_at is None:
            self.get_logger().warning('No pose received yet')
            return
        elapsed = time.monotonic() - self.last_message_at
        if elapsed > self.stale_after:
            self.get_logger().warning(
                f'Pose report is stale ({elapsed:.1f} s without an update)')


def main(args=None):
    """Run the control-room pose monitor node."""
    rclpy.init(args=args)
    node = None
    try:
        node = PoseMonitor()
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
