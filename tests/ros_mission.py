"""Lane action readiness and finish handshake on an isolated ROS domain."""

import json
import time
import unittest

from action_msgs.msg import GoalStatus
from pinky_interfaces.action import FollowLane
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pinky_interfaces.msg import RobotHeartbeat
from pinky_lane_driving.ros_safety import SafetyNode

from pinky_lane_driving.lane_mission_server import LaneMissionServer


class LaneMissionTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.server = LaneMissionServer()
        self.probe = rclpy.create_node('lane_mission_test_probe')
        self.safety = SafetyNode()
        self.watchdog = rclpy.create_node('lane_watchdog')
        for key, value in LaneMissionServer.WATCHDOG.items():
            self.watchdog.declare_parameter(key, value)
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.safety)
        self.executor.add_node(self.watchdog)
        self.executor.add_node(self.server)
        self.executor.add_node(self.probe)
        self.reason = 'follow'
        self.send_commands = True
        self.command_pub = self.probe.create_publisher(String, 'lane/command', 10)
        self.finish_pub = self.probe.create_publisher(Bool, 'lane/finish', 10)
        self.modes = []
        self.live_mode = 'STOP'
        self.linear_speed = 0.
        self.permit = True
        self.diagnostic_pub = self.probe.create_publisher(String, 'lane/diagnostics', 10)
        self.mode_pub = self.probe.create_publisher(String, 'drive/mode_status', 10)
        self.cmd_pub = self.probe.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_pub = self.probe.create_publisher(Odometry, 'odom', 10)
        self.gate_pub = self.probe.create_publisher(RobotHeartbeat, 'fleet/heartbeat', 10)
        self.mode_sub = self.probe.create_subscription(
            String, 'drive/mode_request', self.mode_request, 10)
        self.timer = self.probe.create_timer(.03, self.publish)
        self.client = ActionClient(self.probe, FollowLane, 'follow_lane')
        self.wait(lambda: self.client.server_is_ready() and self.server.local.service_is_ready()
                  and self.server.safety.service_is_ready() and self.server.watchdog.service_is_ready())

    def test_exit_report_carries_observation_and_actual_odom_evidence(self):
        server = self.server
        now = time.monotonic()
        with server.lock:
            server.guard.note('diagnostic', dict(boundary_observation=dict(left_visible=False, right_visible=False),
                observation_age_s=.1, capture_age_s=None, scan_age_s=.1, odom_age_s=.1,
                scan_coverage_ok=True, scan_hit=False), now)
            server.guard.note('gate', dict(permit_fresh=True, mode=1), now)
            server.last_reason = 'sensor_failure'
        odom = Odometry()
        odom.header.stamp = server.get_clock().now().to_msg()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
        odom.pose.pose.orientation.w = 1.
        server._odom_callback(odom)
        status = server._exit_status(time.monotonic())
        self.assertEqual(status['version'], 1)
        self.assertTrue(status['sensors_ok'])
        self.assertTrue(status['gate_run'])
        self.assertFalse(status['boundaries']['left_visible'])
        self.assertIsNotNone(status['stationary_s'])
        self.assertGreaterEqual(status['observation_age_s'], .1)
        odom.header.stamp = server.get_clock().now().to_msg()
        odom.twist.twist.angular.z = .1
        server._odom_callback(odom)
        self.assertIsNone(server._exit_status(time.monotonic())['stationary_s'])

    def test_wait_command_source_must_be_valid_before_guard_exemption(self):
        for offset, speed in [(-2., 0.), (1., 0.), (0., float('nan'))]:
            with self.subTest(offset=offset, speed=speed):
                stamp = self.server.get_clock().now().nanoseconds / 1e9 + offset
                self.server._command_callback(String(data=json.dumps(
                    dict(reason='obstacle', capture_stamp=stamp, speed=speed, omega=0.))))
                self.assertEqual(self.server.guard.values['command'], 'invalid_command')
        stamp = self.server.get_clock().now().nanoseconds / 1e9
        self.server._command_callback(String(data=json.dumps(
            dict(reason='obstacle', capture_stamp=stamp, speed=0., omega=0.))))
        self.assertEqual(self.server.guard.values['command'], 'obstacle')
        self.assertEqual(self.server.ready_streak, 0)

    def tearDown(self):
        self.executor.shutdown()
        self.client.destroy()
        self.server.destroy_node()
        self.safety.destroy_node()
        self.watchdog.destroy_node()
        self.probe.destroy_node()
        rclpy.shutdown()

    def mode_request(self, message):
        self.modes.append(message.data)
        self.live_mode = message.data

    def publish(self):
        self.mode_pub.publish(String(data=json.dumps(dict(mode=self.live_mode))))
        velocity = Twist()
        if self.live_mode == 'LANE':
            velocity.linear.x = self.linear_speed
        self.cmd_pub.publish(velocity)
        self.odom_pub.publish(Odometry())
        self.gate_pub.publish(RobotHeartbeat(permit_fresh=self.permit, gate_mode=1))
        self.diagnostic_pub.publish(String(data=json.dumps(dict(lane_valid=True,
            lane_reason='paired', scan_hit=False, scan_coverage_ok=True,
            capture_age_s=.1, scan_age_s=.1, odom_age_s=.1))))
        if self.send_commands:
            payload = dict(reason=self.reason, capture_stamp=time.time(), speed=.09, omega=0.)
            self.command_pub.publish(String(data=json.dumps(payload)))

    def wait(self, predicate, timeout=5.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        self.fail(f'Lane mission timed out: {self.server.state}: {self.server.detail}; values={self.server.guard.values}')

    def goal(self, detection_timeout=2.):
        goal = FollowLane.Goal(mission_id='test', route_id='right_lane',
                               detection_timeout_sec=detection_timeout, max_duration_sec=5.)
        future = self.client.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        self.handle = handle
        return handle.get_result_async()

    def test_final_follow_reason_arms_and_finish_stops(self):
        self.linear_speed = .09
        result = self.goal()
        self.wait(lambda: self.server.guard.values.get('velocity') == (.09, 0.))
        self.finish_pub.publish(Bool(data=True))
        self.wait(result.done)
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.result().result.code, FollowLane.Result.RESULT_SUCCESS)
        self.wait(lambda: self.modes[-1] == 'STOP')

    def test_old_watchdog_limit_is_rejected_before_arming(self):
        self.watchdog.set_parameters([Parameter('max_speed_mps', value=.06)])
        result = self.goal()
        self.wait(result.done)
        self.assertEqual(result.result().result.code, FollowLane.Result.RESULT_FAULT)
        self.assertIn('watchdog_setting_mismatch:max_speed_mps', result.result().result.message)
        self.assertNotIn('LANE', self.modes)
        self.assertFalse(self.safety.enabled)

    def test_speed_over_nine_cm_aborts_and_releases_permission(self):
        self.linear_speed = .091
        result = self.goal()
        self.wait(result.done)
        self.assertEqual(result.result().result.code, FollowLane.Result.RESULT_FAULT)
        self.assertIn('velocity_limit_exceeded', result.result().result.message)
        self.assertFalse(self.safety.enabled)

    def test_sensor_failure_never_arms(self):
        self.reason = 'sensor_failure'
        result = self.goal(detection_timeout=.2)
        self.wait(result.done)
        self.assertEqual(result.result().result.code, FollowLane.Result.RESULT_LANE_NOT_READY, result.result().result.message)
        self.assertNotIn('LANE', self.modes)

    def test_stale_ready_streak_never_arms(self):
        self.wait(lambda: self.server.ready_streak >= 5)
        self.send_commands = False
        self.wait(lambda: time.monotonic() - self.server.last_command_received > .6)
        result = self.goal(detection_timeout=.2)
        self.wait(result.done)
        self.assertEqual(result.result().result.code, FollowLane.Result.RESULT_LANE_NOT_READY, result.result().result.message)
        self.assertNotIn('LANE', self.modes)


    def test_cancel_releases_local_permission(self):
        result = self.goal()
        self.wait(lambda: 'LANE' in self.modes)
        self.handle.cancel_goal_async()
        self.wait(result.done)
        self.assertFalse(self.safety.enabled)
        self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
        self.assertTrue(self.server.cleanup_ok)

    def test_connection_loss_aborts_and_releases_permission(self):
        result = self.goal()
        self.wait(lambda: 'LANE' in self.modes)
        self.permit = False
        self.wait(result.done)
        self.assertFalse(self.safety.enabled)
        self.assertEqual(result.result().result.code, FollowLane.Result.RESULT_FAULT)
        self.assertTrue(self.server.cleanup_ok)

    def test_two_runs_can_finish_without_restarting_launch(self):
        first = self.goal()
        self.wait(lambda: self.live_mode == 'LANE')
        self.finish_pub.publish(Bool(data=True))
        self.wait(first.done)
        self.assertFalse(self.safety.enabled)
        self.finish_pub.publish(Bool(data=False))
        self.wait(lambda: not self.server.finish_high)
        second = self.goal()
        self.wait(lambda: self.live_mode == 'LANE')
        self.finish_pub.publish(Bool(data=True))
        self.wait(second.done)
        self.assertEqual(second.result().status, GoalStatus.STATUS_SUCCEEDED)
        self.assertFalse(self.safety.enabled)
