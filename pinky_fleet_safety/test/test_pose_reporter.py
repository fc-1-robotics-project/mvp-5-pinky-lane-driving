"""Tests for periodic fleet pose message conversion."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from pinky_fleet_safety.pose_reporter import FleetPoseReporter, pose_from_transform
import pytest
from rclpy.clock import ClockType
from rclpy.time import Time as RosTime


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
    transform.header.stamp = Time(sec=10, nanosec=200000000)

    message = pose_from_transform(transform, covariance)

    assert message.header.frame_id == 'map'
    assert message.header.stamp == transform.header.stamp
    assert message.pose.pose.position.x == 1.25
    assert message.pose.pose.position.y == -0.5
    assert message.pose.pose.orientation.z == 0.5
    assert list(message.pose.covariance) == covariance


def source_stamp(seconds):
    nanoseconds = round(seconds * 1000000000)
    return Time(sec=nanoseconds // 1000000000, nanosec=nanoseconds % 1000000000)


def reporter(transform_stamp=9.8):
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.header.stamp = source_stamp(transform_stamp)
    statuses, published = [], []
    node = SimpleNamespace(
        latest_covariance=[0.0] * 36, latest_covariance_stamp=RosTime(seconds=9.).to_msg(),
        global_frame='map', robot_base_frame='base_footprint', transform_timeout=None,
        transform_stale_sec=2.,
        tf_buffer=SimpleNamespace(lookup_transform=lambda *a, **k: transform),
        get_clock=lambda: SimpleNamespace(
            now=lambda: RosTime(seconds=10., clock_type=ClockType.ROS_TIME)),
        pose_publisher=SimpleNamespace(publish=published.append), _report_status=statuses.append,
    )
    return node, statuses, published


def test_publication_preserves_tf_source_age():
    node, statuses, published = reporter(8.2)
    FleetPoseReporter._publish_pose(node)
    assert statuses == ['REPORTING']
    assert len(published) == 1
    assert RosTime.from_msg(published[0].header.stamp).nanoseconds == 8200000000


@pytest.mark.parametrize('stamp', [0., 10.001, 7.9, -1.])
def test_invalid_future_or_stale_tf_does_not_publish(stamp):
    node, statuses, published = reporter(stamp)
    FleetPoseReporter._publish_pose(node)
    assert not published
    assert statuses[-1] != 'REPORTING'


@pytest.mark.parametrize('stamp', [0., 10.001, -1.])
def test_invalid_covariance_source_time_does_not_publish(stamp):
    node, statuses, published = reporter()
    node.latest_covariance_stamp = source_stamp(stamp)
    FleetPoseReporter._publish_pose(node)
    assert not published
    assert statuses[-1] != 'REPORTING'


def test_stationary_amcl_covariance_is_not_required_at_tf_publish_rate():
    node, statuses, published = reporter()
    node.latest_covariance_stamp = RosTime(seconds=1.).to_msg()
    FleetPoseReporter._publish_pose(node)
    assert len(published) == 1


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -0.01])
def test_invalid_covariance_fails_closed(value):
    node, statuses, published = reporter()
    node.latest_covariance[0] = value
    FleetPoseReporter._publish_pose(node)
    assert not published


def test_out_of_order_amcl_update_invalidates_covariance():
    node, statuses, published = reporter()
    node.latest_covariance_stamp = RosTime(seconds=9.5).to_msg()
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.header.stamp = RosTime(seconds=9.).to_msg()
    FleetPoseReporter._pose_callback(node, message)
    FleetPoseReporter._publish_pose(node)
    assert not published
