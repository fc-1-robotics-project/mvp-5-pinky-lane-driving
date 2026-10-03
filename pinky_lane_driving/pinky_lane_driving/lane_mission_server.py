"""Integrated local readiness, permission lease and FollowLane supervision."""

import json
import math
from threading import RLock
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pinky_interfaces.action import FollowLane
from pinky_interfaces.msg import RobotHeartbeat
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .mission_guard import MissionGuard


class LaneMissionServer(Node):
    HEALTHY_REASONS = {'follow', 'tracking', 'crosswalk'}
    WATCHDOG = dict(dry_run=False, hardware_watchdog_confirmed=True,
                    command_timeout_s=.2, max_source_age_s=1.1,
                    max_speed_mps=.03, max_omega_radps=.6,
                    output_topic='cmd_vel_lane_candidate')

    def __init__(self):
        super().__init__('lane_mission_server')
        for key, value in dict(action_name='follow_lane', finish_topic='lane/finish',
                               command_topic='lane/command',
                               mode_request_topic='drive/mode_request').items():
            self.declare_parameter(key, value)
        self.lock = RLock()
        self.guard = MissionGuard()
        self.goal_active = False
        self.arm_requested = False
        self.renewal = None
        self.ready_streak = 0
        self.last_command_received = None
        self.last_reason = 'no_command'
        self.finish_high = False
        self.finish_generation = 0
        self.state = 'IDLE'
        self.detail = 'waiting_for_operator'
        self.mission_id = ''
        self.cleanup_ok = True
        self.last_status_sent = 0.
        group = ReentrantCallbackGroup()
        self.local = self.create_client(SetBool, 'lane/set_enabled', callback_group=group)
        self.watchdog = self.create_client(GetParameters, 'lane_watchdog/get_parameters', callback_group=group)
        self.safety = self.create_client(GetParameters, 'lane_safety/get_parameters', callback_group=group)
        self.mode_publisher = self.create_publisher(String, self._topic('mode_request_topic'), 10)
        self.status_publisher = self.create_publisher(String, 'lane/mission_status', 10)
        self.create_subscription(String, self._topic('command_topic'), self._command_callback, 10)
        self.create_subscription(Bool, self._topic('finish_topic'), self._finish_callback, 10)
        self.create_subscription(String, 'lane/diagnostics', self._diagnostic_callback, qos_profile_sensor_data)
        self.create_subscription(String, 'drive/mode_status', self._mode_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self._note('velocity', (m.linear.x, m.angular.z)), qos_profile_sensor_data)
        self.create_subscription(Bool, 'lane/estop', lambda m: self._note('estop', m.data), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', lambda m: self._note('pose', (m.pose.pose.position.x, m.pose.pose.position.y)), qos_profile_sensor_data)
        self.create_subscription(RobotHeartbeat, 'fleet/heartbeat', lambda m: self._note('gate', dict(permit_fresh=m.permit_fresh, mode=m.gate_mode)), 10)
        self.timer = self.create_timer(.2, self._tick)
        self.action_server = ActionServer(self, FollowLane, self._topic('action_name'),
                                         execute_callback=self._execute,
                                         goal_callback=self._goal_callback,
                                         cancel_callback=lambda _: CancelResponse.ACCEPT,
                                         callback_group=group)

    def _topic(self, name):
        return self.get_parameter(name).value

    def _note(self, key, value):
        with self.lock:
            self.guard.note(key, value, time.monotonic())

    def _diagnostic_callback(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                self._note('diagnostic', value)
        except (ValueError, TypeError):
            pass

    def _mode_callback(self, message):
        try:
            self._note('mode', json.loads(message.data)['mode'])
        except (ValueError, TypeError, KeyError):
            pass

    def _command_callback(self, message):
        healthy, reason = False, 'malformed_command'
        try:
            data = json.loads(message.data)
            reason = data.get('reason', '')
            age = self.get_clock().now().nanoseconds / 1e9 - float(data['capture_stamp'])
            healthy = reason in self.HEALTHY_REASONS and 0 <= age <= 1.1
        except (ValueError, TypeError, KeyError):
            pass
        with self.lock:
            self.ready_streak = self.ready_streak + 1 if healthy else 0
            self.last_command_received = time.monotonic()
            self.last_reason = reason
            self.guard.note('command', reason, self.last_command_received)

    def _finish_callback(self, message):
        with self.lock:
            if message.data and not self.finish_high:
                self.finish_generation += 1
            self.finish_high = message.data

    def _request_mode(self, mode):
        self.mode_publisher.publish(String(data=mode))

    def _tick(self):
        now = time.monotonic()
        with self.lock:
            if self.arm_requested and (self.renewal is None or self.renewal.done()):
                if self.local.service_is_ready():
                    self.renewal = self.local.call_async(SetBool.Request(data=True))
            if now - self.last_status_sent < .5:
                return
            self.last_status_sent = now
            ready, reason = self.guard.readiness(now)
            data = dict(state=self.state, detail=self.detail, active=self.goal_active,
                        mission_id=self.mission_id, ready=ready, readiness_reason=reason,
                        cleanup_ok=self.cleanup_ok, distance_m=round(self.guard.distance, 4),
                        mode=self.guard.values.get('mode', 'UNKNOWN'),
                        lane_reason=self.guard.values.get('diagnostic', {}).get('lane_reason'),
                        counts=dict(self.guard.counts),
                        permission_enabled=self.guard.values.get('estop') is False)
        self.status_publisher.publish(String(data=json.dumps(data, allow_nan=False)))

    def _goal_callback(self, request):
        if (not request.mission_id or not request.route_id or
                not all(math.isfinite(x) and x > 0 for x in
                        (request.detection_timeout_sec, request.max_duration_sec))):
            return GoalResponse.REJECT
        with self.lock:
            if self.goal_active:
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    def _feedback(self, handle, state, detail):
        with self.lock:
            self.state, self.detail = state, detail
        handle.publish_feedback(FollowLane.Feedback(
            state=state, lane_ready=state == 'FOLLOWING', detail=detail))

    def _await(self, future, timeout=2.):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        if not future.done():
            raise TimeoutError('service_response_timeout')
        return future.result()

    def _read_parameters(self, client, names):
        if not client.service_is_ready():
            raise RuntimeError('parameter_service_unavailable')
        response = self._await(client.call_async(GetParameters.Request(names=list(names))))
        values = {}
        for name, value in zip(names, response.values):
            if value.type == 1:
                values[name] = value.bool_value
            elif value.type == 3:
                values[name] = value.double_value
            elif value.type == 4:
                values[name] = value.string_value
        if len(values) != len(names):
            raise RuntimeError('invalid_parameter_types')
        return values

    def _verify_settings(self):
        values = self._read_parameters(self.watchdog, self.WATCHDOG)
        for name, want in self.WATCHDOG.items():
            got = values[name]
            if type(got) is not type(want) or (not math.isclose(got, want, abs_tol=1e-6)
                                               if isinstance(want, float) else got != want):
                raise RuntimeError('watchdog_setting_mismatch:' + name)
        safety = self._read_parameters(self.safety, ('start_enabled', 'enable_timeout_s'))
        if safety['start_enabled'] or not 0 < safety['enable_timeout_s'] <= 1.:
            raise RuntimeError('local_safety_must_start_disabled_with_lease')

    def _run(self, handle):
        goal = handle.request
        started = time.monotonic()
        self._request_mode('STOP')
        self._verify_settings()
        with self.lock:
            finish_at_start = self.finish_generation
            self.guard.reset_run()
        stable, previous_sample = 0, None
        while rclpy.ok():
            if handle.is_cancel_requested:
                return FollowLane.Result.RESULT_CANCELLED, 'operator_cancel'
            now = time.monotonic()
            if now - started >= goal.detection_timeout_sec:
                return FollowLane.Result.RESULT_LANE_NOT_READY, self.detail
            with self.lock:
                ready, reason = self.guard.readiness(now)
                sample = self.guard.received.get('diagnostic')
                if sample != previous_sample:
                    stable = stable + 1 if ready else 0
                    previous_sample = sample
                finished = self.finish_generation > finish_at_start
            if finished:
                return FollowLane.Result.RESULT_CANCELLED, 'finish_before_start'
            if stable >= 3:
                break
            self._feedback(handle, 'WAITING_FOR_LANE', reason)
            time.sleep(.05)
        if not rclpy.ok():
            raise RuntimeError('shutdown')
        with self.lock:
            self.ready_streak = 0
            self.arm_requested = True
        arm_start = time.monotonic()
        while rclpy.ok():
            if handle.is_cancel_requested:
                return FollowLane.Result.RESULT_CANCELLED, 'operator_cancel'
            now = time.monotonic()
            with self.lock:
                armed = (self.guard.fresh('estop', now) and self.guard.values['estop'] is False
                         and self.ready_streak >= 5 and self.guard.fresh('command', now, .5))
                gate_ok = self.guard.fresh('gate', now) and self.guard.values['gate']['permit_fresh']
            if not gate_ok:
                raise RuntimeError('central_connection_lost')
            if armed:
                break
            if now - arm_start > 3. or now - started >= goal.detection_timeout_sec:
                return FollowLane.Result.RESULT_LANE_NOT_READY, 'local_arm_or_lane_readiness_failed'
            self._feedback(handle, 'WAITING_FOR_LANE', 'arming_local_permission')
            time.sleep(.05)
        self._request_mode('LANE')
        self._feedback(handle, 'FOLLOWING', 'lane_ready')
        while rclpy.ok():
            if handle.is_cancel_requested:
                return FollowLane.Result.RESULT_CANCELLED, 'operator_cancel'
            now = time.monotonic()
            with self.lock:
                finished = self.finish_generation > finish_at_start
                fault = self.guard.fault(now)
                held = (self.guard.values.get('gate', {}).get('mode') == 0
                        or self.guard.values.get('mode') == 'MANUAL')
                healthy = bool(self.ready_streak)
                detail = self.last_reason
            if fault:
                return FollowLane.Result.RESULT_FAULT, fault
            if finished:
                return FollowLane.Result.RESULT_SUCCESS, 'operator_finish'
            if now - started > goal.max_duration_sec:
                return FollowLane.Result.RESULT_TIMEOUT, 'mission_timeout'
            self._feedback(handle, 'SAFETY_HOLD' if held or not healthy else 'FOLLOWING', detail)
            time.sleep(.05)
        raise RuntimeError('shutdown')

    def _release(self):
        with self.lock:
            self.arm_requested = False
            pending = self.renewal
        self._request_mode('STOP')
        if pending is not None and not pending.done():
            try:
                self._await(pending, 1.)
            except Exception:
                pass
        response = self._await(self.local.call_async(SetBool.Request(data=False)))
        if response is None or not response.success:
            raise RuntimeError('local_permission_release_failed')
        released = time.monotonic()
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            self._request_mode('STOP')
            with self.lock:
                now = time.monotonic()
                safe = (self.guard.stationary(now) and self.guard.fresh('estop', now)
                        and self.guard.values.get('estop') is True
                        and all(self.guard.received.get(key, 0.) > released
                                for key in ('mode', 'velocity', 'estop')))
            if safe:
                return True
            time.sleep(.05)
        raise RuntimeError('stop_confirmation_timeout')

    def _execute(self, handle):
        with self.lock:
            self.mission_id = handle.request.mission_id
            self.cleanup_ok = False
        code, detail = FollowLane.Result.RESULT_FAULT, 'unhandled_mission_failure'
        try:
            code, detail = self._run(handle)
        except Exception as error:
            detail = str(error)
        finally:
            try:
                self._feedback(handle, 'STOPPING', detail)
            except Exception:
                pass  # Context shutdown must still revoke the local lease.
            try:
                self.cleanup_ok = self._release()
            except Exception as error:
                self.cleanup_ok = False
                code = FollowLane.Result.RESULT_FAULT
                detail += '; cleanup:' + str(error)
            with self.lock:
                self.goal_active = False
                self.state = 'COMPLETE' if code == FollowLane.Result.RESULT_SUCCESS else 'STOPPED'
                self.detail = detail
        if handle.is_cancel_requested:
            handle.canceled()
        elif code == FollowLane.Result.RESULT_SUCCESS:
            handle.succeed()
        else:
            handle.abort()
        return FollowLane.Result(code=code, message=detail)

    def destroy_node(self):
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneMissionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.arm_requested = False
        if rclpy.ok():
            node._request_mode('STOP')
        executor.shutdown(timeout_sec=4.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
