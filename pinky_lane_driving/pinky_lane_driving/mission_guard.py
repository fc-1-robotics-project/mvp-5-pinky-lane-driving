"""Pure readiness and progress checks shared by integrated lane missions."""

from collections import Counter
import math


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class MissionGuard:
    def __init__(self):
        self.values = {}
        self.received = {}
        self.reset_run()

    def reset_run(self):
        self.distance = 0.0
        self.previous_pose = None
        self.anchor = None
        self.last_progress = None
        self.holds = {}
        self.counts = Counter()

    def note(self, key, value, now):
        self.values[key] = value
        self.received[key] = now
        if key == 'diagnostic':
            reason = value.get('lane_reason')
            focus = ('single_boundary', 'recent_path_no_boundaries', 'no_current_lane')
            self.counts[reason if isinstance(reason, str) and reason in focus else 'other'] += 1
            if 'lane_observation_stale' in str(value.get('fault', '')):
                self.counts['lane_observation_stale'] += 1

    def fresh(self, key, now, limit=.75):
        return key in self.received and 0 <= now - self.received[key] <= limit

    def stationary(self, now):
        return (self.fresh('mode', now) and self.values.get('mode') == 'STOP'
                and self.fresh('velocity', now)
                and all(finite(v) and abs(v) < 1e-4 for v in self.values['velocity']))

    def sensor_problem(self, now):
        if not self.fresh('diagnostic', now):
            return 'diagnostics_stale'
        d = self.values['diagnostic']
        for name, limit in [('capture_age_s', 1.1), ('scan_age_s', .3), ('odom_age_s', .3)]:
            age = d.get(name)
            if not finite(age) or not 0 <= age <= limit:
                return name + '_stale'
        if d.get('observation_error') is not None or d.get('sensor_error') is not None:
            return 'sensor_or_tf_error'
        if d.get('scan_coverage_ok') is not True:
            return 'scan_coverage_unknown'
        return ''

    def readiness(self, now):
        if not self.stationary(now):
            return False, 'STOP_and_zero_velocity_required'
        if not self.fresh('estop', now) or self.values['estop'] is not True:
            return False, 'local_permission_not_released'
        if not self.fresh('gate', now) or not self.values['gate']['permit_fresh']:
            return False, 'central_permit_unavailable'
        if self.values['gate']['mode'] == 2:
            return False, 'central_estop'
        problem = self.sensor_problem(now)
        if problem:
            return False, problem
        d = self.values['diagnostic']
        if d.get('lane_valid') is not True:
            return False, d.get('lane_reason', 'lane_not_ready')
        if d.get('scan_hit') is not False:
            return False, 'obstacle_or_unknown'
        return True, 'ready'

    def sustained(self, key, condition, now, seconds):
        if not condition:
            self.holds.pop(key, None)
            return False
        return now - self.holds.setdefault(key, now) >= seconds

    def fault(self, now):
        if not self.fresh('gate', now) or not self.values['gate']['permit_fresh']:
            return 'central_connection_lost'
        if self.values['gate']['mode'] == 2:
            return 'central_estop'
        if not self.fresh('mode', now) or not self.fresh('velocity', now):
            return 'motion_status_stale'
        mode = self.values['mode']
        # Intentional fleet HOLD and manual control must not consume stall timers.
        if mode == 'MANUAL' or self.values['gate']['mode'] == 0:
            self.holds.clear()
            self.previous_pose = None
            self.anchor = None
            self.last_progress = now
            return ''
        if mode != 'LANE':
            return 'unexpected_drive_mode' if self.sustained('mode', True, now, 2.) else ''
        self.holds.pop('mode', None)
        v, w = self.values['velocity']
        if not finite(v) or not finite(w) or abs(v) > .0305 or abs(w) > .605:
            return 'velocity_limit_exceeded'
        if not self.fresh('command', now, .5):
            return 'lane_command_stale'
        if not self.fresh('estop', now) or self.values['estop'] is True:
            return 'local_permission_lost'
        problem = self.sensor_problem(now)
        if self.sustained('sensor', bool(problem), now, 2.):
            return problem
        d = self.values.get('diagnostic', {})
        lane_bad = not problem and d.get('lane_valid') is not True
        if self.sustained('lane', lane_bad, now, 15.):
            return 'persistent_lane_loss'
        if self.sustained('zero', abs(v) < .002 and abs(w) < .01, now, 15.):
            return 'persistent_stop'
        if not self.fresh('pose', now, .5):
            return 'odom_stale'
        pose = self.values['pose']
        if not all(finite(x) for x in pose):
            return 'invalid_odom'
        if self.previous_pose is not None:
            step = math.dist(pose, self.previous_pose)
            if step > .2:
                return 'odom_jump'
            self.distance += step
        self.previous_pose = pose
        if self.anchor is None or math.dist(pose, self.anchor) >= .003:
            self.anchor, self.last_progress = pose, now
        if abs(v) >= .002 and now - self.last_progress >= 6.:
            return 'no_odom_progress'
        return ''
