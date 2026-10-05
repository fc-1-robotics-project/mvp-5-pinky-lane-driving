# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Metric crosswalk overlap and odometry-anchored event identity."""

import math

from .obstacles import convex_hull, point_segment_distance, transform_points
from .path import cross
from .tracking import relative_pose


def inside(point, polygon):
    hit = False
    x, y = point
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if point_segment_distance(point, a, b) < 1e-9:
            return True
        if (a[1] > y) != (b[1] > y):
            if x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
                hit = not hit
    return hit


def outline(points):
    """Convex hull on a 1 cm grid: a bounded, conservative crosswalk outline.

    Mask contours carry hundreds of vertices, the control timer re-projects
    each observation at every tick and every new detection jitters. An exact
    point set therefore grew without bound and starved the 20 Hz loop.
    Passage and overlap only need the outer extent; filling a concave notch
    can only make the robot treat the crosswalk as slightly nearer.
    """
    cells = {(round(x, 2), round(y, 2)) for x, y in points}
    return convex_hull(cells) or tuple(cells)


def center(points):
    """Bounding-box centre: independent of how densely the outline is sampled."""
    return tuple((min(p[i] for p in points) + max(p[i] for p in points)) / 2 for i in (0, 1))


def path_interval(polygon, path):
    """First/last overlapping path arc position in metres, or None."""
    polygon = transform_points(polygon, (0., 0., 0.))
    path = transform_points(path, (0., 0., 0.))
    if len(polygon) < 3 or len(path) < 2:
        raise ValueError('Polygon and path geometry required')
    arc = 0.
    hits = []
    for a, b in zip(path, path[1:]):
        direction = (b[0] - a[0], b[1] - a[1])
        length = math.hypot(*direction)
        if length < 1e-9:
            raise ValueError('Repeated path vertex')
        if inside(a, polygon):
            hits.append(arc)
        if inside(b, polygon):
            hits.append(arc + length)
        for c, d in zip(polygon, polygon[1:] + polygon[:1]):
            edge = (d[0] - c[0], d[1] - c[1])
            denominator = cross(direction, edge)
            if abs(denominator) < 1e-9:
                continue
            offset = (c[0] - a[0], c[1] - a[1])
            t, u = cross(offset, edge) / denominator, cross(offset, direction) / denominator
            if 0 <= t <= 1 and 0 <= u <= 1:
                hits.append(arc + t * length)
        arc += length
    return (min(hits), max(hits)) if hits else None


class CrosswalkTracker:
    """Track one grouped event until positive passage, not detection disappearance.

    Polygons and path are current base_footprint metres, pose is odom_from_base
    at the SAME timestamp. Caller must reject stale/discontinuous odometry.
    group_gap is measured maximum spacing between stripes of one crosswalk.
    """

    def __init__(self, *, group_gap, association_distance, passed_margin):
        if any(not math.isfinite(v) or v <= 0
               for v in (group_gap, association_distance, passed_margin)):
            raise ValueError('Crosswalk limits must be finite and positive')
        self.group_gap = group_gap
        self.association_distance = association_distance
        self.passed_margin = passed_margin
        self.serial = 0
        self.active_id = None
        self.passed_id = None
        self.world_points = ()
        self.start_pose = None

    def reset(self):
        """Drop the tracked event; serial keeps later identities unique."""
        self.active_id = self.passed_id = self.start_pose = None
        self.world_points = ()

    def update(self, polygons, path, pose):
        transform_points([], pose)
        if self.active_id is not None:
            current = transform_points(self.world_points, relative_pose((0., 0., 0.), pose))
            start_frame = transform_points(self.world_points,
                                           relative_pose((0., 0., 0.), self.start_pose))
            travel = relative_pose(pose, self.start_pose)[0]
            if (max(p[0] for p in current) < -self.passed_margin
                    and travel > max(p[0] for p in start_frame) + self.passed_margin):
                self.passed_id = self.active_id
                self.active_id = None
                self.world_points = ()
        candidates = []
        for polygon in polygons:
            interval = path_interval(polygon, path)
            if interval is not None:
                candidates.append((interval[0], interval[1], tuple(polygon)))
        candidates.sort(key=lambda item: item[0])
        if not candidates:
            return None, self.passed_id
        start, end, polygon = candidates[0]
        grouped = list(polygon)
        for first, last, polygon in candidates[1:]:
            if first - end > self.group_gap:
                break
            end = max(end, last)
            grouped.extend(polygon)
        world = outline(transform_points(grouped, pose))
        if self.active_id is not None:
            if math.dist(center(world), center(self.world_points)) > self.association_distance:
                return None, self.passed_id
            # Preserve farthest observed extent so partial stripe loss cannot
            # shorten passage evidence.
            self.world_points = outline(self.world_points + world)
        else:
            self.serial += 1
            self.active_id = f'crosswalk-{self.serial}'
            self.start_pose = pose
            self.world_points = world
        return (self.active_id, start), self.passed_id
