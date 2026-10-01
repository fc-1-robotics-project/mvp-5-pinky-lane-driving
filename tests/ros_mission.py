"""Lane action readiness and finish handshake on an isolated ROS domain."""

import json
import time
import unittest

from action_msgs.msg import GoalStatus
from pinky_interfaces.action import FollowLane
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String

from pinky_lane_driving.lane_mission_server import LaneMissionServer


class LaneMissionTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.server = LaneMissionServer()
        self.probe = rclpy.create_node('lane_mission_test_probe')
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.server)
        self.executor.add_node(self.probe)
        self.reason = 'follow'
        self.send_commands = True
        self.command_pub = self.probe.create_publisher(String, 'lane/command', 10)
        self.finish_pub = self.probe.create_publisher(Bool, 'lane/finish', 10)
        self.modes = []
        self.mode_sub = self.probe.create_subscription(
            String, 'drive/mode_request', lambda msg: self.modes.append(msg.data), 10)
        self.timer = self.probe.create_timer(.03, self.publish)
        self.client = ActionClient(self.probe, FollowLane, 'follow_lane')
        self.wait(self.client.server_is_ready)

    def tearDown(self):
        self.executor.shutdown()
        self.client.destroy()
        self.server.destroy_node()
        self.probe.destroy_node()
        rclpy.shutdown()

    def publish(self):
        if self.send_commands:
            payload = dict(reason=self.reason, capture_stamp=time.time(), speed=.03, omega=0.)
            self.command_pub.publish(String(data=json.dumps(payload)))

    def wait(self, predicate, timeout=3.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        self.fail('Lane mission condition timed out')

    def goal(self, detection_timeout=1.):
        goal = FollowLane.Goal(mission_id='test', route_id='right_lane',
                               detection_timeout_sec=detection_timeout, max_duration_sec=2.)
        future = self.client.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        return handle.get_result_async()

    def test_final_follow_reason_arms_and_finish_stops(self):
        result = self.goal()
        self.wait(lambda: 'LANE' in self.modes)
        self.finish_pub.publish(Bool(data=True))
        self.wait(result.done)
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.result().result.code, FollowLane.Result.RESULT_SUCCESS)
        self.wait(lambda: self.modes[-1] == 'STOP')

    def test_sensor_failure_never_arms(self):
        self.reason = 'sensor_failure'
        result = self.goal(detection_timeout=.2)
        self.wait(result.done)
        self.assertEqual(result.result().result.code, FollowLane.Result.RESULT_LANE_NOT_READY)
        self.assertNotIn('LANE', self.modes)

    def test_stale_ready_streak_never_arms(self):
        self.wait(lambda: self.server.ready_streak >= 5)
        self.send_commands = False
        self.wait(lambda: time.monotonic() - self.server.last_command_received > .6)
        result = self.goal(detection_timeout=.2)
        self.wait(result.done)
        self.assertEqual(result.result().result.code, FollowLane.Result.RESULT_LANE_NOT_READY)
        self.assertNotIn('LANE', self.modes)
