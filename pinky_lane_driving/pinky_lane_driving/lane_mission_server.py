"""Robot-local FollowLane action server with an explicit Bool finish edge."""

import json
import math
from threading import Lock
import time

from pinky_interfaces.action import FollowLane
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String


class LaneMissionServer(Node):
    """Arm lane driving after stable control output and stop on a finish edge."""

    # Final behavior arbitration names normal motion `follow`. Keep `tracking`
    # compatible with controllers that expose the geometric proposal directly.
    # `crosswalk` includes an intentional hold at a detected crossing.
    HEALTHY_REASONS = {'follow', 'tracking', 'crosswalk'}

    def __init__(self) -> None:
        super().__init__('lane_mission_server')
        self.declare_parameter('action_name', 'follow_lane')
        self.declare_parameter('finish_topic', 'lane/finish')
        self.declare_parameter('command_topic', 'lane/command')
        self.declare_parameter('mode_request_topic', 'drive/mode_request')
        self.declare_parameter('stable_command_count', 5)
        self.declare_parameter('command_stale_sec', 0.5)
        self.stable_count_required = int(
            self.get_parameter('stable_command_count').value,
        )
        self.command_stale_sec = float(
            self.get_parameter('command_stale_sec').value,
        )
        if self.stable_count_required <= 0:
            raise ValueError('stable_command_count must be positive')
        if (not math.isfinite(self.command_stale_sec)
                or self.command_stale_sec <= 0.0):
            raise ValueError('command_stale_sec must be finite and positive')

        self.lock = Lock()
        self.goal_active = False
        self.ready_streak = 0
        self.last_command_received = None
        self.last_reason = 'no_command'
        self.finish_high = False
        self.finish_generation = 0
        self.mode_publisher = self.create_publisher(
            String, self._string_parameter('mode_request_topic'), 10,
        )
        self.command_subscription = self.create_subscription(
            String,
            self._string_parameter('command_topic'),
            self._command_callback,
            10,
        )
        self.finish_subscription = self.create_subscription(
            Bool,
            self._string_parameter('finish_topic'),
            self._finish_callback,
            10,
        )
        callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            FollowLane,
            self._string_parameter('action_name'),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

    def _string_parameter(self, name: str) -> str:
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value:
            raise ValueError(f'{name} must be a non-empty string')
        return value

    def _goal_callback(self, goal_request):
        if (not goal_request.mission_id or not goal_request.route_id
                or not math.isfinite(goal_request.detection_timeout_sec)
                or goal_request.detection_timeout_sec <= 0.0
                or not math.isfinite(goal_request.max_duration_sec)
                or goal_request.max_duration_sec <= 0.0):
            return GoalResponse.REJECT
        with self.lock:
            if self.goal_active:
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _command_callback(self, message: String) -> None:
        healthy = False
        reason = 'malformed_command'
        try:
            payload = json.loads(message.data)
            reason = payload.get('reason', '')
            healthy = (
                isinstance(reason, str)
                and reason in self.HEALTHY_REASONS
                and math.isfinite(float(payload['capture_stamp']))
            )
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            healthy = False
        with self.lock:
            self.ready_streak = self.ready_streak + 1 if healthy else 0
            self.last_command_received = time.monotonic()
            self.last_reason = reason

    def _finish_callback(self, message: Bool) -> None:
        with self.lock:
            rising = message.data and not self.finish_high
            self.finish_high = message.data
            if rising:
                self.finish_generation += 1

    def _request_mode(self, mode: str) -> None:
        self.mode_publisher.publish(String(data=mode))

    def _snapshot(self):
        with self.lock:
            return (
                self.ready_streak,
                self.last_command_received,
                self.last_reason,
                self.finish_generation,
            )

    def _feedback(self, goal_handle, state, ready, detail):
        feedback = FollowLane.Feedback()
        feedback.state = state
        feedback.lane_ready = ready
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def _result(self, code, message):
        result = FollowLane.Result()
        result.code = code
        result.message = message
        return result

    def _execute(self, goal_handle):
        goal = goal_handle.request
        started = time.monotonic()
        _, _, _, start_finish_generation = self._snapshot()
        self._request_mode('STOP')
        try:
            while True:
                if goal_handle.is_cancel_requested:
                    self._request_mode('STOP')
                    goal_handle.canceled()
                    return self._result(
                        FollowLane.Result.RESULT_CANCELLED,
                        'lane mission cancelled',
                    )
                now = time.monotonic()
                streak, received, reason, _ = self._snapshot()
                if (streak >= self.stable_count_required and received is not None
                        and 0 <= now - received <= self.command_stale_sec):
                    break
                if now - started >= goal.detection_timeout_sec:
                    self._request_mode('STOP')
                    goal_handle.abort()
                    return self._result(
                        FollowLane.Result.RESULT_LANE_NOT_READY,
                        f'lane was not ready: {reason}',
                    )
                self._feedback(
                    goal_handle, 'WAITING_FOR_LANE', False, reason,
                )
                time.sleep(0.05)

            self._request_mode('LANE')
            self._feedback(goal_handle, 'FOLLOWING', True, 'lane ready')
            while True:
                if goal_handle.is_cancel_requested:
                    self._request_mode('STOP')
                    goal_handle.canceled()
                    return self._result(
                        FollowLane.Result.RESULT_CANCELLED,
                        'lane mission cancelled',
                    )
                now = time.monotonic()
                streak, received, reason, finish_generation = self._snapshot()
                if finish_generation > start_finish_generation:
                    self._request_mode('STOP')
                    goal_handle.succeed()
                    return self._result(
                        FollowLane.Result.RESULT_SUCCESS,
                        'lane finish edge received',
                    )
                if now - started >= goal.max_duration_sec:
                    self._request_mode('STOP')
                    goal_handle.abort()
                    return self._result(
                        FollowLane.Result.RESULT_TIMEOUT,
                        'lane mission timed out',
                    )
                if (received is None
                        or now - received > self.command_stale_sec):
                    self._request_mode('STOP')
                    goal_handle.abort()
                    return self._result(
                        FollowLane.Result.RESULT_FAULT,
                        'lane command stream became stale',
                    )
                self._feedback(
                    goal_handle,
                    'FOLLOWING' if streak else 'SAFETY_HOLD',
                    bool(streak),
                    reason,
                )
                time.sleep(0.05)
        finally:
            with self.lock:
                self.goal_active = False

    def destroy_node(self):
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneMissionServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._request_mode('STOP')
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
