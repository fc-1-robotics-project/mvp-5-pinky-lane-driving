"""Tests for periodic fleet pose message conversion."""

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped

from pinky_fleet_safety.pose_reporter import pose_from_transform


def test_pose_from_transform_uses_current_tf_and_amcl_covariance() -> None:
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.transform.translation.x = 1.25
    transform.transform.translation.y = -0.5
    transform.transform.rotation.z = 0.5
    transform.transform.rotation.w = 0.866
    covariance = [0.0] * 36
    covariance[0] = 0.25
    covariance[7] = 0.5
    stamp = Time(sec=12, nanosec=34)

    message = pose_from_transform(transform, covariance, stamp)

    assert message.header.frame_id == 'map'
    assert message.header.stamp == stamp
    assert message.pose.pose.position.x == 1.25
    assert message.pose.pose.position.y == -0.5
    assert message.pose.pose.orientation.z == 0.5
    assert list(message.pose.covariance) == covariance
