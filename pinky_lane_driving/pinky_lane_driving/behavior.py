# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Nonblocking behavior state; caller supplies fresh, path-associated observations."""

from dataclasses import dataclass, fields, replace
import math

from .control import Proposal


@dataclass(frozen=True)
class Settings:
    cruise_speed: float
    approach_speed: float
    stop_distance: float
    stopped_speed: float
    wait_s: float
    clear_s: float
    decel: float
    latency: float

    def __post_init__(self):
        if any(not math.isfinite(getattr(self, f.name)) or getattr(self, f.name) < 0
               for f in fields(self)):
            raise ValueError('Settings must be finite and nonnegative')
        if min(self.cruise_speed, self.approach_speed, self.wait_s,
               self.clear_s, self.decel) <= 0 or self.approach_speed > self.cruise_speed:
            raise ValueError('Invalid positive speed, hold or braking limits')


@dataclass(frozen=True)
class Decision:
    state: str
    speed_limit: float
    reason: str


class Behavior:
    """FOLLOW -> APPROACH -> WAIT -> PASS, with independent safety holds.

    Crosswalk is (stable_event_id, distance_along_current_path_m) or None.
    passed_id must come from positive geometric/odometry passage evidence,
    never mere loss of detection. Event identity association is upstream.
    now uses one monotonic clock; all settings are explicit, not robot defaults.
    """

    def __init__(self, settings):
        self.settings = settings
        self.state = 'FOLLOW'
        self.event_id = None
        self.completed_id = None
        self.wait_since = None
        self.clear_since = None
        self.last_time = None

    def step(self, now, *, sensors_ok, estop, obstacle, crosswalk, passed_id, measured_speed):
        def stop(reason):
            return Decision(self.state, 0., reason)

        if (not math.isfinite(now) or now < 0
                or self.last_time is not None and now < self.last_time):
            self.clear_since = self.wait_since = None
            return stop('clock_failure')
        self.last_time = now
        invalid = not math.isfinite(measured_speed)
        if crosswalk is not None:
            invalid |= (len(crosswalk) != 2 or not isinstance(crosswalk[0], str)
                        or not crosswalk[0])
            if not invalid:
                invalid = not math.isfinite(crosswalk[1]) or crosswalk[1] < 0
        if estop is not False or sensors_ok is not True or obstacle not in (True, False) or invalid:
            self.clear_since = self.wait_since = None
            return stop('emergency' if estop is not False else 'sensor_failure')
        if obstacle:
            self.clear_since = self.wait_since = None
            return stop('obstacle')
        if self.clear_since is None:
            self.clear_since = now
        clear = now - self.clear_since >= self.settings.clear_s

        if self.state == 'PASS' and passed_id == self.event_id:
            self.completed_id = self.event_id
            self.event_id = None
            self.state = 'FOLLOW'
        if self.state == 'FOLLOW' and crosswalk is not None and crosswalk[0] != self.completed_id:
            self.event_id = crosswalk[0]
            self.state = 'APPROACH'
        speed = self.settings.cruise_speed
        if self.state == 'APPROACH':
            if crosswalk is None or crosswalk[0] != self.event_id:
                return stop('crosswalk_lost')
            distance = crosswalk[1] - self.settings.stop_distance
            if distance <= 0:
                speed = 0.
                if abs(measured_speed) <= self.settings.stopped_speed:
                    self.state = 'WAIT'
                    self.wait_since = now
            else:
                a = self.settings.decel
                speed = min(self.settings.approach_speed,
                            math.sqrt((a * self.settings.latency)**2 + 2 * a * distance)
                            - a * self.settings.latency)
        if self.state == 'WAIT':
            speed = 0.
            if abs(measured_speed) > self.settings.stopped_speed:
                self.wait_since = None
            elif self.wait_since is None:
                self.wait_since = now
            if (self.wait_since is not None and clear
                    and now - self.wait_since >= self.settings.wait_s):
                self.state = 'PASS'
                speed = self.settings.approach_speed
        elif self.state == 'PASS':
            speed = self.settings.approach_speed
        if not clear:
            return stop('clear_hold')
        return Decision(self.state, speed, 'crosswalk' if self.state != 'FOLLOW' else 'follow')


def arbitrate(proposal, decision):
    """One final proposal; preserve curvature when behavior lowers linear speed."""
    if (proposal.reason != 'tracking'
            or not all(math.isfinite(v) for v in (proposal.speed, proposal.omega,
                                                  decision.speed_limit))
            or proposal.speed <= 0 or decision.speed_limit <= 0):
        return Proposal(reason=decision.reason if decision.speed_limit <= 0 else proposal.reason)
    speed = min(proposal.speed, decision.speed_limit)
    return replace(proposal, speed=speed, omega=proposal.omega * speed / proposal.speed,
                   reason=decision.reason)
