# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Synthetic sensor graph -> controller -> independent dry-run watchdog."""

from dataclasses import asdict
import copy
import json
import math
import os
import time
import unittest
from unittest.mock import Mock, patch

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformException

from pinky_lane_driving.ros_control import ControlNode, nonnegative_margin
from pinky_lane_driving.control import Proposal
from pinky_lane_driving.ros_watchdog import WatchdogNode
from test_calibration import synthetic_config
import test_runtime
from test_tracking import observation


class RosControlTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        helper = test_runtime.RuntimeTest()
        helper.setUp()
        control = asdict(helper.core.limits)
        control['stop_margin'] = .02
        config = dict(calibration=synthetic_config(), mounting_id='synthetic_fixture',
                      path=asdict(helper.core.tracker.settings), control=control,
                      recovery_min_path_length_m=.06,
                      behavior=asdict(helper.core.behavior.settings),
                      crosswalk=dict(group_gap=.2, association_distance=.4, passed_margin=.15),
                      sensors=dict(scan_timeout_s=.2, odom_timeout_s=.2, estop_timeout_s=.2,
                                   max_scan_gap_rad=.03, footprint_radius_m=.15,
                                   obstacle_stop_margin_m=.06,
                                   infinity_is_clear=False))
        self.config = config
        namespace = f'/control_test_{os.getpid()}'
        self.controller = ControlNode(config=config, namespace=namespace)
        self.watchdog = WatchdogNode(namespace=namespace)
        self.probe = rclpy.create_node('synthetic_sensors', namespace=namespace)
        self.executor = SingleThreadedExecutor()
        for node in (self.controller, self.watchdog, self.probe):
            self.executor.add_node(node)
        self.obs_pub = self.probe.create_publisher(String, 'lane/observation', 1)
        self.scan_pub = self.probe.create_publisher(LaserScan, 'scan', 1)
        self.odom_pub = self.probe.create_publisher(Odometry, 'odom', 1)
        self.estop_pub = self.probe.create_publisher(Bool, 'lane/estop', 1)
        self.messages = []
        self.diagnostics = []
        self.diag_sub = self.probe.create_subscription(
            String, 'lane/diagnostics', lambda msg: self.diagnostics.append(json.loads(msg.data)), 10)
        self.sub = self.probe.create_subscription(Twist, 'lane/dry_run_cmd_vel',
                                                 lambda msg: self.messages.append(msg), 10)
        self.tf = StaticTransformBroadcaster(self.probe)
        transforms = []
        for parent, child in [('odom', 'base_footprint'), ('base_footprint', 'laser')]:
            tf = TransformStamped()
            tf.header.stamp = self.probe.get_clock().now().to_msg()
            tf.header.frame_id, tf.child_frame_id = parent, child
            tf.transform.rotation.w = 1.
            transforms.append(tf)
        self.tf.sendTransform(transforms)  # Stationary synthetic robot, not physical TF evidence.
        self.send_scan = True
        self.partial_scan = False
        self.emergency = False
        self.scan_overrides = {}
        self.timer = self.probe.create_timer(.03, self.publish_sensors)

    def test_exit_visibility_distinguishes_empty_frame_from_invalid_or_old_frame(self):
        c = self.controller
        c.tf_pose = Mock(return_value=(0., 0., 0.))
        stamp = c.get_clock().now().nanoseconds / 1e9
        frame = observation(stamp)
        for detection in frame['detections']:
            detection.update(confidence=.9, polygon_px=((0., 0.), (2., 0.), (2., 2.)))
        c.observe(String(data=json.dumps(frame)))
        self.assertEqual(c.exit_observation, dict(left_visible=True, right_visible=True))
        frame['capture_time_s'] = c.get_clock().now().nanoseconds / 1e9
        frame['detections'] = []
        c.observe(String(data=json.dumps(frame)))
        self.assertEqual(c.exit_observation, dict(left_visible=False, right_visible=False))
        self.assertIsNotNone(c.exit_capture_stamp)
        # Duplicate source frames and malformed/stale frames are not exit evidence.
        c.observe(String(data=json.dumps(frame)))
        self.assertIsNone(c.exit_observation)
        frame['capture_time_s'] -= 5.
        c.observe(String(data=json.dumps(frame)))
        self.assertIsNone(c.exit_observation)
        c.observe(String(data='{}'))
        self.assertIsNone(c.exit_capture_stamp)

    def tearDown(self):
        self.executor.shutdown()
        for node in (self.controller, self.watchdog, self.probe):
            node.destroy_node()
        rclpy.shutdown()

    def publish_sensors(self):
        stamp = self.probe.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
        odom.pose.pose.orientation.w = 1.
        self.odom_pub.publish(odom)
        self.estop_pub.publish(Bool(data=self.emergency))
        if self.send_scan:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'laser'
            scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 360
            scan.range_min, scan.range_max = .02, 3.
            scan.ranges = [2.] * (90 if self.partial_scan else 360)
            for index, value in self.scan_overrides.items():
                if index < len(scan.ranges):
                    scan.ranges[index] = value
            scan.angle_max = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
            self.scan_pub.publish(scan)
        self.obs_pub.publish(String(data=json.dumps(observation(stamp.sec + stamp.nanosec / 1e9))))

    def wait(self, predicate, timeout=4.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        self.fail('Controller graph condition timed out')

    def wait_duration(self, seconds):
        end = time.monotonic() + seconds
        self.wait(lambda: time.monotonic() >= end)

    def test_new_mission_id_resets_crosswalk_once(self):
        status = self.probe.create_publisher(String, 'lane/mission_status', 10)
        core = self.controller.core
        calls = []
        core.start_mission = lambda: calls.append(core.crosswalks.active_id)

        def send(**data):
            status.publish(String(data=json.dumps(data)))
            self.wait_duration(.15)

        self.wait(lambda: status.get_subscription_count() > 0)
        send(mission_id='m1', active=False)          # not started yet
        send(mission_id='', active=True)
        status.publish(String(data='not json'))
        self.wait_duration(.15)
        self.assertEqual(calls, [])
        send(mission_id='m1', active=True)
        send(mission_id='m1', active=True)           # 2 Hz status repeats the same mission
        self.assertEqual(len(calls), 1)
        send(mission_id='m2', active=True)
        self.assertEqual(len(calls), 2)

    def test_valid_graph_moves_dry_run_then_scan_loss_stops(self):
        self.wait(lambda: any(m.linear.x > 0 for m in self.messages))
        self.wait(lambda: bool(self.diagnostics))
        self.assertIn('lane_reason', self.diagnostics[-1])
        self.assertEqual(self.diagnostics[-1]['behavior_state'], 'FOLLOW')
        self.send_scan = False
        self.wait_duration(.4)
        self.assertEqual(self.messages[-1].linear.x, 0.)
        self.wait(lambda: self.diagnostics[-1]['sensor_error'] == 'Scan unavailable or stale')

    def test_controller_endpoint_margin_does_not_shrink_lidar_horizon(self):
        self.wait(lambda: any(d['collision_horizon_m'] is not None for d in self.diagnostics))
        diagnostic = next(d for d in reversed(self.diagnostics)
                          if d['collision_horizon_m'] is not None)
        limits = self.controller.core.limits
        self.assertEqual(limits.stop_margin, .02)
        self.assertEqual(self.controller.core.recovery_min_path_length_m, .06)
        self.assertEqual(self.controller.obstacle_stop_margin_m, .06)
        expected = (limits.max_speed *
                    (limits.latency + self.controller.sensors['scan_timeout_s']
                     + limits.timeout + .05)
                    + limits.max_speed**2 / (2 * limits.braking_decel) + .06)
        self.assertAlmostEqual(diagnostic['collision_horizon_m'], expected)
        self.assertEqual(diagnostic['obstacle_stop_margin_m'], .06)
        self.assertEqual(diagnostic['recovery_min_path_length_m'], .06)

    def test_configured_obstacle_stop_disabled_is_visible_and_preserves_estop(self):
        config = copy.deepcopy(self.config)
        config['lidar_obstacle_stop_enabled'] = False
        node = ControlNode(config=config, namespace=f'/disabled_stop_{os.getpid()}')
        try:
            self.assertFalse(node.core.lidar_obstacle_stop_enabled)
        finally:
            node.destroy_node()
        self.controller.core.lidar_obstacle_stop_enabled = False
        self.scan_overrides[180] = .1
        self.wait(lambda: any(d.get('raw_scan_hit') is True
                             and d['scan_hit'] is False for d in self.diagnostics))
        self.messages.clear()
        self.wait(lambda: any(m.linear.x > 0 for m in self.messages))
        self.assertFalse(self.diagnostics[-1]['lidar_obstacle_stop_enabled'])
        self.emergency = True
        self.wait(lambda: self.diagnostics[-1]['emergency'] is True
                  and self.messages[-1].linear.x == 0.)
        self.emergency = False
        self.send_scan = False
        self.wait(lambda: self.diagnostics[-1].get('fault') == 'Scan unavailable or stale'
                  and self.messages[-1].linear.x == 0.)

    def test_valid_command_checks_forward_stop_and_command_arc(self):
        self.wait(lambda: any(m.linear.x > 0 for m in self.messages))
        self.controller.timer.cancel()
        with patch('pinky_lane_driving.ros_control.command', return_value=
                   Proposal(.03, .09, 3., (.19, .05), 'tracking')), \
             patch('pinky_lane_driving.ros_control.scan_collision', return_value=False) as scan:
            self.controller.tick()
        scan.assert_called_once()
        args = scan.call_args.kwargs
        self.assertEqual(args['path'][0], (0., 0.))
        self.assertEqual(args['path'][-1][1], 0.)
        self.assertGreater(args['path'][-1][0], .06)
        limits = self.controller.core.limits
        forward = (limits.max_speed *
                   (limits.latency + self.controller.sensors['scan_timeout_s'] + .05)
                   + limits.max_speed**2 / (2 * limits.braking_decel))
        self.assertAlmostEqual(args['path'][-1][0], forward)
        self.assertEqual(args['steering_path'][0], (0., 0.))
        self.assertGreater(args['steering_path'][-1][1], 0.)
        self.assertGreater(args['steering_guard'], 0.)
        # the heading-straight check is limited to the lane, not to the whole lane path preview
        self.assertGreater(len(args['lane_path']), 1)
        self.assertGreater(args['lane_path'][-1][0], args['path'][-1][0])
        self.assertAlmostEqual(args['lane_half_width'], self.controller.straight_check_half_width_m)
        self.assertGreater(args['lane_half_width'], 0.)

    def test_turn_preview_clears_side_return_but_keeps_front_and_turn_hits(self):
        # Field profile: a full 11.25 cm straight preview intersects the outside
        # of a left bend even though the executable turn clears it. Mirror the
        # scene to cover both directions. Exercise the real scan/footprint code.
        from dataclasses import replace
        self.wait(lambda: any(m.linear.x > 0 for m in self.messages))
        self.controller.timer.cancel()
        self.timer.cancel()
        self.controller.core.limits = replace(
            self.controller.core.limits, max_speed=.03, latency=.2,
            braking_decel=.15, timeout=1.1)
        self.controller.sensors.update(
            scan_timeout_s=.3, footprint_radius_m=.11,
            footprint_polygon_m=((-.08,-.06),(.06,-.06),(.06,.06),(-.08,.06)),
            footprint_padding_m=0., infinity_is_clear=True)

        def inspect(point, curvature, measured_speed=0.):
            now = self.probe.get_clock().now()
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = now.to_msg(), 'laser'
            scan.angle_min = math.atan2(point[1], point[0])
            scan.angle_increment = 2 * math.pi / 360
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
            scan.range_min, scan.range_max = .02, 3.
            scan.ranges = [math.hypot(*point)] + [math.inf] * 359
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
            odom.pose.pose.orientation.w = 1.
            odom.twist.twist.linear.x = measured_speed
            self.controller.receive_odom(odom)
            self.controller.receive_scan(scan)
            self.controller.last_diagnostic_time = -math.inf
            with patch('pinky_lane_driving.ros_control.command', return_value=
                       Proposal(.03, .03 * curvature, curvature, (.19,.1), 'tracking')), \
                 patch.object(self.controller.diagnostic_publisher, 'publish') as pub:
                self.controller.tick()
            diagnostic = json.loads(pub.call_args.args[0].data)
            self.assertTrue(diagnostic['scan_coverage_ok'], diagnostic)
            self.assertIsNone(diagnostic['sensor_error'], diagnostic)
            return diagnostic

        for sign in (-1., 1.):
            clear = inspect((.110, -sign * .055), sign * 5.56)
            self.assertFalse(clear['scan_hit'])
            self.assertAlmostEqual(clear['collision_horizon_m'], .1125)
            self.assertAlmostEqual(clear['forward_stop_horizon_m'], .0195)
            # An outside-lane point is still blocking if the physical body can
            # hit it. Keep close front, commanded-turn, and current-body hits.
            for point in ((.075, 0.), (.14, sign * .07), (.08,-sign * .045), (.04,0.)):
                self.assertTrue(inspect(point, sign * 5.56)['scan_hit'], point)
            # The forward envelope expands for measured overspeed; it must not
            # shrink merely because the requested command is only 3 cm/s.
            fast = inspect((.110, -sign * .055), sign * 5.56, measured_speed=.1)
            self.assertTrue(fast['scan_hit'])
            self.assertGreater(fast['forward_stop_horizon_m'], .08)

    def test_legacy_missing_margin_keys_default_to_controller_margin(self):
        legacy = copy.deepcopy(self.config)
        legacy.pop('recovery_min_path_length_m')
        legacy['sensors'].pop('obstacle_stop_margin_m')
        node = ControlNode(config=legacy, namespace=f'/legacy_margin_{os.getpid()}')
        try:
            self.assertEqual(node.core.recovery_min_path_length_m, .02)
            self.assertEqual(node.obstacle_stop_margin_m, .02)
        finally:
            node.destroy_node()

    def test_margin_values_must_be_finite_nonnegative_numbers(self):
        for value in (-.01, math.nan, math.inf, True, '0.06'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                nonnegative_margin(value, 'obstacle_stop_margin_m')
        self.assertEqual(nonnegative_margin(0, 'obstacle_stop_margin_m'), 0.)

    def test_partial_scan_and_emergency_never_count_as_clear(self):
        self.partial_scan = True
        self.wait_duration(.3)
        self.assertTrue(self.messages)
        self.assertTrue(all(m.linear.x == 0. for m in self.messages))
        self.partial_scan = False
        self.wait(lambda: self.messages[-1].linear.x > 0.)
        self.emergency = True
        self.wait_duration(.15)
        self.assertEqual(self.messages[-1].linear.x, 0.)

    def test_polygon_self_return_side_wall_and_real_front_obstacle(self):
        self.controller.sensors.update(
            footprint_polygon_m=((- .08,- .06),(.06,- .06),(.06,.06),(- .08,.06)),
            footprint_padding_m=.01, self_filter_bounds_m=(.035,.052,-.04,.04))
        self.scan_overrides = {180: .044, 90: .107}
        self.wait(lambda: any(m.linear.x > 0 for m in self.messages))
        self.scan_overrides[180] = .2
        self.wait(lambda: self.diagnostics and self.diagnostics[-1]['reason'] == 'obstacle')
        self.wait(lambda: self.messages[-1].linear.x == 0.)
        self.scan_overrides[180] = math.nan
        self.wait(lambda: self.diagnostics[-1].get('fault') == 'unknown_scan_collision_result')
        self.assertEqual(self.messages[-1].linear.x, 0.)

    def test_nine_cm_mask_keeps_sensor_fault_and_external_obstacle_stop(self):
        self.controller.sensors.update(
            footprint_polygon_m=((- .08,- .06),(.06,- .06),(.06,.06),(- .08,.06)),
            footprint_padding_m=.01, self_filter_radius_m=.09)
        self.scan_overrides = {180: .01, 90: .107}
        self.wait(lambda: any(m.linear.x > 0 for m in self.messages))
        self.scan_overrides[180] = .1
        self.wait(lambda: self.diagnostics and self.diagnostics[-1]['reason'] == 'obstacle')
        self.wait(lambda: self.messages[-1].linear.x == 0.)
        self.scan_overrides[180] = math.nan
        self.wait(lambda: self.diagnostics[-1].get('fault') == 'unknown_scan_collision_result')
        self.assertEqual(self.messages[-1].linear.x, 0.)

    def test_odom_before_tf_uses_one_recent_matching_odom(self):
        self.timer.cancel()
        now = self.probe.get_clock().now()
        previous = Odometry()
        previous.header.stamp = (now - Duration(seconds=.03)).to_msg()
        previous.header.frame_id, previous.child_frame_id = 'odom', 'base_footprint'
        previous.pose.pose.position.x = .012
        previous.pose.pose.orientation.w = 1.
        previous.twist.twist.linear.x = .011
        latest = Odometry()
        latest.header.stamp = now.to_msg()
        latest.header.frame_id, latest.child_frame_id = 'odom', 'base_footprint'
        latest.pose.pose.position.x = .023
        latest.pose.pose.orientation.w = 1.
        latest.twist.twist.linear.x = .022
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = now.to_msg(), 'laser'
        self.controller.receive_odom(previous)
        self.controller.receive_odom(latest)
        self.controller.receive_scan(scan)
        fake_buffer = Mock()
        aligned_tf = TransformStamped()
        aligned_tf.transform.rotation.w = 1.
        def lookup(target, target_time, source, source_time, fixed):
            if target_time.nanoseconds == now.nanoseconds:
                raise TransformException('Latest odom TF has not arrived')
            return aligned_tf
        fake_buffer.lookup_transform_full.side_effect = lookup
        self.controller.buffer = fake_buffer
        selected, selected_scan, transform = self.controller.scan_transform_with_odom(
            (now + Duration(seconds=.01)).nanoseconds / 1e9)
        self.assertIs(selected, previous)
        self.assertIs(selected_scan, scan)
        self.assertEqual(selected.pose.pose.position.x, .012)
        self.assertEqual(selected.twist.twist.linear.x, .011)
        self.assertIs(transform, aligned_tf)
        self.assertEqual(fake_buffer.lookup_transform_full.call_count, 2)

    def test_older_odom_or_unavailable_scan_tf_still_fails(self):
        self.timer.cancel()
        now = self.probe.get_clock().now()
        previous = Odometry()
        previous.header.stamp = (now - Duration(seconds=.19)).to_msg()
        previous.header.frame_id, previous.child_frame_id = 'odom', 'base_footprint'
        latest = Odometry()
        latest.header.stamp = now.to_msg()
        latest.header.frame_id, latest.child_frame_id = 'odom', 'base_footprint'
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = now.to_msg(), 'laser'
        fake_buffer = Mock()
        fake_buffer.lookup_transform_full.side_effect = TransformException('Scan TF unavailable')
        self.controller.buffer = fake_buffer
        self.controller.receive_odom(previous)
        self.controller.receive_odom(latest)
        self.controller.receive_scan(scan)
        with self.assertRaises(TransformException):
            self.controller.scan_transform_with_odom(
                (now + Duration(seconds=.01)).nanoseconds / 1e9)
        self.assertEqual(fake_buffer.lookup_transform_full.call_count, 1)
        previous.header.stamp = (now - Duration(seconds=.03)).to_msg()
        self.controller.odom_history.clear()
        self.controller.odom_history.append((previous, time.monotonic()))
        self.controller.odom_history.append(self.controller.odom)
        fake_buffer.lookup_transform_full.reset_mock()
        with self.assertRaises(TransformException):
            self.controller.scan_transform_with_odom(
                (now + Duration(seconds=.01)).nanoseconds / 1e9)
        self.assertEqual(fake_buffer.lookup_transform_full.call_count, 2)

    def test_thirteen_centisecond_tf_lag_uses_matching_odom_sample(self):
        self.timer.cancel()
        now = self.probe.get_clock().now()
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = now.to_msg(), 'laser'
        self.controller.receive_scan(scan)
        odoms = []
        for index, age in enumerate((.167, .133, .100, .067, .033, 0.)):
            odom = Odometry()
            odom.header.stamp = (now - Duration(seconds=age)).to_msg()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
            odom.pose.pose.position.x = index * .001
            odom.pose.pose.orientation.w = 1.
            odom.twist.twist.linear.x = index * .002
            self.controller.receive_odom(odom)
            odoms.append(odom)
        aligned_tf = TransformStamped()
        aligned_tf.transform.rotation.w = 1.
        available_at = (now - Duration(seconds=.13)).nanoseconds
        fake_buffer = Mock()
        def lookup(target, target_time, source, source_time, fixed):
            if target_time.nanoseconds > available_at:
                raise TransformException('Requested odom time ahead of TF buffer')
            return aligned_tf
        fake_buffer.lookup_transform_full.side_effect = lookup
        self.controller.buffer = fake_buffer
        selected, selected_scan, transform = self.controller.scan_transform_with_odom(
            (now + Duration(seconds=.01)).nanoseconds / 1e9)
        self.assertIs(selected, odoms[1])
        self.assertIs(selected_scan, scan)
        self.assertEqual(selected.pose.pose.position.x, .001)
        self.assertEqual(selected.twist.twist.linear.x, .002)
        self.assertIs(transform, aligned_tf)
        self.assertEqual(fake_buffer.lookup_transform_full.call_count, 5)

    def test_scan_and_odom_tf_lag_selects_and_reports_older_scan(self):
        self.timer.cancel()
        now = self.probe.get_clock().now()
        for age in (.167, .133, .100, .067, .033, 0.):
            odom = Odometry()
            odom.header.stamp = (now - Duration(seconds=age)).to_msg()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
            odom.pose.pose.orientation.w = 1.
            self.controller.receive_odom(odom)
        scans = []
        for age, count in ((.15, 90), (.07, 360), (0., 360)):
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = (now - Duration(seconds=age)).to_msg(), 'laser'
            scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 360
            scan.range_min, scan.range_max = .02, 3.
            scan.ranges = [2.] * count
            scan.angle_max = scan.angle_min + (count - 1) * scan.angle_increment
            self.controller.receive_scan(scan)
            scans.append(scan)
        fake_buffer = Mock()
        aligned_tf = TransformStamped()
        aligned_tf.transform.rotation.w = 1.
        available_at = (now - Duration(seconds=.13)).nanoseconds
        def lookup(target, target_time, source, source_time, fixed):
            if target_time.nanoseconds > available_at or source_time.nanoseconds > available_at:
                raise TransformException('Scan or odom TF has not arrived')
            return aligned_tf
        fake_buffer.lookup_transform_full.side_effect = lookup
        self.controller.buffer = fake_buffer
        odom, scan, transform = self.controller.scan_transform_with_odom(
            (now + Duration(seconds=.01)).nanoseconds / 1e9)
        self.assertIs(scan, scans[0])
        self.assertEqual(odom.header.stamp, (now - Duration(seconds=.133)).to_msg())
        self.assertIs(transform, aligned_tf)
        self.controller.diagnostic_publisher = Mock()
        self.controller.tick()
        diagnostic = json.loads(self.controller.diagnostic_publisher.publish.call_args.args[0].data)
        self.assertFalse(diagnostic['scan_coverage_ok'])
        self.assertGreater(diagnostic['tf_scan_rewind_s'], .14)
