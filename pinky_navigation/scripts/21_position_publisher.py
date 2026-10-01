# position_publisher.py

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from tf2_ros import Buffer, TransformListener, TransformException


class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')

        self.publisher_ = self.create_publisher(
            Pose2D,
            '/pinky_position',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # 1초마다 실행
        self.timer = self.create_timer(
            1.0,
            self.publish_position
        )

        self.get_logger().info(
            'Pinky position publisher started'
        )

    def publish_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation

            # Quaternion -> yaw
            siny_cosp = 2.0 * (
                q.w * q.z +
                q.x * q.y
            )

            cosy_cosp = 1.0 - 2.0 * (
                q.y * q.y +
                q.z * q.z
            )

            yaw = math.atan2(
                siny_cosp,
                cosy_cosp
            )

            msg = Pose2D()

            msg.x = x
            msg.y = y
            msg.theta = yaw

            self.publisher_.publish(msg)

            self.get_logger().info(
                f'publish => '
                f'x={x:.3f}, '
                f'y={y:.3f}, '
                f'yaw={math.degrees(yaw):.1f} deg'
            )

        except TransformException as e:
            self.get_logger().warn(
                f'TF 조회 실패: {e}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = PositionPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()