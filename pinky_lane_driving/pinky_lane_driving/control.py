# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Pure Pursuit in base_footprint metres; outputs are proposals, never commands to ROS."""

from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class Limits:
    """Explicit hardware tuning: metres, seconds, m/s, rad/s and m/s squared."""

    max_speed: float
    max_omega: float
    max_accel: float
    max_lateral_accel: float
    braking_decel: float
    latency: float
    stop_margin: float
    timeout: float
    min_lookahead: float
    max_lookahead: float
    lookahead_time: float

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{field.name} must be finite and nonnegative')
        for name in ('max_speed', 'max_omega', 'max_accel', 'max_lateral_accel',
                     'braking_decel', 'timeout', 'min_lookahead'):
            if getattr(self, name) == 0:
                raise ValueError(f'{name} must be positive')
        if self.max_lookahead < self.min_lookahead:
            raise ValueError('Lookahead bounds are reversed')


@dataclass(frozen=True)
class Proposal:
    speed: float = 0.
    omega: float = 0.
    curvature: float = 0.
    target: tuple = (0., 0.)
    reason: str = 'invalid_input'


def command(path, *, age, dt, previous_speed, requested_speed, metric_valid, limits):
    """Track a near-to-far polyline; safety reductions bypass acceleration ramp.

    The caller must transform the path into the CURRENT robot frame and provide
    capture age in a single clock domain. No implicit previous-path reuse.
    Lower layers must enforce obstacle priority, watchdog, and physical braking.
    """
    if (metric_valid is not True
            or not all(math.isfinite(v) for v in (age, dt, previous_speed, requested_speed))
            or not 0 <= age <= limits.timeout or dt <= 0
            or previous_speed < 0 or requested_speed < 0 or len(path) < 2):
        return Proposal()
    if any(len(p) != 2 or not all(math.isfinite(v) for v in p) or p[0] < 0 for p in path):
        return Proposal()
    # Choose closest point on the supplied polyline, then follow its arc length.
    segments = []
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return Proposal()
        t = min(1., max(0., -(a[0] * dx + a[1] * dy) / length**2))
        q = (a[0] + t * dx, a[1] + t * dy)
        segments.append((math.hypot(*q), q, t, length))
    nearest = min(range(len(segments)), key=lambda i: segments[i][0])
    _, start, fraction, length = segments[nearest]
    remaining = (1 - fraction) * length + sum(s[3] for s in segments[nearest + 1:])
    usable = remaining - limits.stop_margin
    if usable <= 0 or requested_speed == 0:
        return Proposal(reason='stop_requested_or_short_path')
    lookahead = min(limits.max_lookahead,
                    max(limits.min_lookahead, previous_speed * limits.lookahead_time))
    distance = min(lookahead, remaining)
    target = tuple(path[-1])
    for end in path[nearest + 1:]:
        length = math.dist(start, end)
        if length > 0 and distance <= length:
            target = tuple(a + (b - a) * distance / length for a, b in zip(start, end))
            break
        distance -= length
        start = end
    radius2 = target[0]**2 + target[1]**2
    if radius2 <= 1e-12:
        return Proposal()
    curvature = 2 * target[1] / radius2
    decel = limits.braking_decel
    stop_speed = (math.sqrt((decel * limits.latency)**2 + 2 * decel * usable)
                  - decel * limits.latency)
    speed = min(requested_speed, limits.max_speed, stop_speed,
                previous_speed + limits.max_accel * dt)
    if abs(curvature) > 1e-12:
        speed = min(speed, limits.max_omega / abs(curvature),
                    math.sqrt(limits.max_lateral_accel / abs(curvature)))
    return Proposal(speed, speed * curvature, curvature, target, 'tracking')
