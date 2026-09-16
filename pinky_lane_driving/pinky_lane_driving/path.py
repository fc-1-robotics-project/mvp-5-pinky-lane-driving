# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Current-lane selection from calibrated, near-to-far boundary polylines."""

from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class Boundary:
    class_id: int
    points: tuple

    def __post_init__(self):
        if self.class_id not in (1, 2) or len(self.points) < 2:
            raise ValueError('A boundary needs a lane class and two or more points')
        if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in self.points):
            raise ValueError('Boundary points must be finite metric coordinates')
        if any(math.dist(a, b) < 1e-9 for a, b in zip(self.points, self.points[1:])):
            raise ValueError('Repeated boundary vertices')


@dataclass(frozen=True)
class PathSettings:
    width: float
    width_tolerance: float
    max_near: float
    sample_step: float
    fallback_timeout: float
    fallback_speed: float
    ambiguity_margin: float
    min_coverage: float = .7

    def __post_init__(self):
        if any(not math.isfinite(getattr(self, f.name)) or getattr(self, f.name) <= 0
               for f in fields(self)):
            raise ValueError('Path limits must be finite and positive')
        if self.width_tolerance >= self.width or not 0 < self.min_coverage <= 1:
            raise ValueError('Invalid width tolerance or coverage')


@dataclass(frozen=True)
class LanePath:
    points: tuple = ()
    selected: tuple = ()
    valid: bool = False
    degraded: bool = False
    speed_limit: float | None = None
    reason: str = 'no_current_lane'


def sections(points, step):
    """Uniform arc samples with local right normals; retain sharp bends."""
    result = []
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        count = max(1, math.ceil(length / step))
        for i in range(count):
            t = i / count
            result.append(((a[0] + t * dx, a[1] + t * dy), (dy / length, -dx / length)))
    result.append((tuple(points[-1]), result[-1][1]))
    return result


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def paired_path(left, right, config):
    samples = sections(left, config.sample_step)
    centers = []
    last_arc = -1.
    for point, normal in samples:
        hits = []
        arc = 0.
        for a, b in zip(right, right[1:]):
            segment = (b[0] - a[0], b[1] - a[1])
            length = math.hypot(*segment)
            denominator = cross(normal, segment)
            if abs(denominator) > 1e-9:
                offset = (a[0] - point[0], a[1] - point[1])
                width = cross(offset, segment) / denominator
                fraction = cross(offset, normal) / denominator
                position = arc + max(0., fraction) * length
                if (-1e-9 <= fraction <= 1 + 1e-9
                        and abs(width - config.width) <= config.width_tolerance
                        and position >= last_arc - 1e-6):
                    hits.append((abs(width - config.width), width, position))
            arc += length
        if not hits:
            # Do not bridge an unobserved gap or extrapolate a missing opposite rail.
            break
        _, width, last_arc = min(hits)
        centers.append((point[0] + normal[0] * width / 2,
                        point[1] + normal[1] * width / 2))
    if len(centers) < 2 or len(centers) / len(samples) < config.min_coverage:
        return ()
    return tuple(centers)


def select_path(boundaries, config, *, fallback_age=None, previous=()):
    """Select the currently occupied right-hand lane, not the largest instance.

    Left/right classes already mean current-lane boundaries. Inputs are ordered
    near-to-far in CURRENT base_footprint. previous must be odometry-transformed
    to this same frame; it is only a continuity hint, never reused as a path.
    fallback_age is elapsed time since the last two-sided observation, maintained
    upstream; omission disables one-sided inference, never grants a fresh lease.
    """
    if previous and any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in previous):
        return LanePath(reason='invalid_previous')
    lefts = [(i, b.points) for i, b in enumerate(boundaries) if b.class_id == 1]
    rights = [(i, b.points) for i, b in enumerate(boundaries) if b.class_id == 2]
    candidates = []

    def accept(points, selected, degraded):
        if len(points) < 2 or math.hypot(*points[0]) > config.max_near or points[0][0] < 0:
            return
        continuity = math.dist(points[0], previous[0]) if previous else 0.
        if continuity > config.max_near:
            return
        score = math.hypot(*points[0]) + continuity
        candidates.append((score, LanePath(points, selected, True, degraded,
                                          config.fallback_speed if degraded else None,
                                          'single_boundary' if degraded else 'paired')))

    for li, left in lefts:
        point, normal = sections(left, config.sample_step)[0]
        origin_side = -point[0] * normal[0] - point[1] * normal[1]
        if not -config.width_tolerance <= origin_side <= config.width + config.width_tolerance:
            continue
        for ri, right in rights:
            if math.dist(left[0], right[0]) > config.width + config.width_tolerance:
                continue
            accept(paired_path(left, right, config), (li, ri), False)
    if not lefts or not rights:
        if (fallback_age is None or not math.isfinite(fallback_age)
                or not 0 <= fallback_age <= config.fallback_timeout):
            return LanePath(reason='fallback_expired_or_unavailable')
        for index, boundary in enumerate(boundaries):
            direction = 1 if boundary.class_id == 1 else -1
            offset = direction * config.width / 2
            points = tuple((p[0] + n[0] * offset, p[1] + n[1] * offset)
                           for p, n in sections(boundary.points, config.sample_step))
            accept(points, (index,), True)
    candidates.sort(key=lambda item: item[0])
    if not candidates:
        return LanePath()
    if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < config.ambiguity_margin:
        return LanePath(reason='ambiguous_boundaries')
    return candidates[0][1]
