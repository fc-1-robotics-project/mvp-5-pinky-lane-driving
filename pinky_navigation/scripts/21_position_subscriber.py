# position_subscriber.py

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D


class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')

        self.subscription = self.create_subscription(
            Pose2D,
            '/pinky_position',
            self.position_callback,
            10
        )

        self.get_logger().info(
            'Pinky position subscriber started'
        )

    def position_callback(self, msg):
        self.get_logger().info(
            f'현재 위치 => '
            f'x={msg.x:.3f}, '
            f'y={msg.y:.3f}, '
            f'yaw={math.degrees(msg.theta):.1f} deg'
        )


def main(args=None):
    rclpy.init(args=args)

    node = PositionSubscriber()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()