# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Planar scan-return collision primitives; coverage validation belongs to adapter."""

import math


def transform_points(points, pose):
    """Apply target_from_source planar rigid transform (x_m, y_m, yaw_rad)."""
    if pose is None or len(pose) != 3 or not all(math.isfinite(v) for v in pose):
        raise ValueError('A finite planar transform is required')
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    result = []
    for point in points:
        if len(point) != 2 or not all(math.isfinite(v) for v in point):
            raise ValueError('Points must be finite 2D coordinates')
        u, v = point
        result.append((x + c * u - s * v, y + s * u + c * v))
    return tuple(result)


def point_segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return math.dist(point, a)
    t = max(0., min(1., ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def convex_footprint(vertices):
    """Normalize an origin-containing, strictly convex physical footprint."""
    points = transform_points(vertices, (0., 0., 0.))
    if len(points) < 3 or len(set(points)) != len(points):
        raise ValueError('Footprint needs at least three distinct vertices')
    hull = convex_hull(points)
    if len(hull) != len(points) or polygon_distance((0., 0.), hull) != 0.:
        raise ValueError('Footprint must be convex and contain the robot origin')
    return hull


def convex_hull(points):
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    points = sorted(set(points))
    halves = []
    for ordered in (points, list(reversed(points))):
        half = []
        for point in ordered:
            while len(half) >= 2 and cross(half[-2], half[-1], point) <= 0:
                half.pop()
            half.append(point)
        halves.append(half[:-1])
    return tuple(halves[0] + halves[1])


def polygon_distance(point, polygon):
    edges = tuple(zip(polygon, polygon[1:] + polygon[:1]))
    if all((b[0]-a[0])*(point[1]-a[1]) - (b[1]-a[1])*(point[0]-a[0]) >= 0
           for a, b in edges):
        return 0.
    return min(point_segment_distance(point, a, b) for a, b in edges)


def validate_self_filter(bounds, footprint):
    """A confirmed self-return mask must lie inside the physical footprint."""
    if (len(bounds) != 4 or not all(math.isfinite(v) for v in bounds)
            or bounds[0] >= bounds[1] or bounds[2] >= bounds[3]
            or any(polygon_distance(p, footprint) != 0.
                   for p in ((bounds[0], bounds[2]), (bounds[0], bounds[3]),
                             (bounds[1], bounds[2]), (bounds[1], bounds[3])))):
        raise ValueError('Self-filter must be a finite rectangle inside the body')
    return tuple(bounds)


def validate_self_radius(radius, footprint):
    if (not math.isfinite(radius) or radius <= 0
            or radius > max(math.hypot(*v) for v in footprint)):
        raise ValueError('Self-filter radius must be positive and no larger than the body envelope')
    return radius


def stopping_corridor(path, *, max_speed, measured_speed, decel, latency,
                      scan_timeout, observation_timeout, timer_period, stop_margin):
    """Clip to a conservative stopping horizon, never just current zero speed.

    Assume the configured maximum speed even when stationary. Sensor leases,
    control scheduling and actuator latency all contribute to reaction travel;
    braking distance and the configured extra stop margin are added. A nearer
    end of the visible path remains intact (the controller brakes for that end).
    """
    values=(max_speed, measured_speed, decel, latency, scan_timeout,
            observation_timeout, timer_period, stop_margin)
    if (not all(math.isfinite(v) for v in values) or max_speed <= 0 or decel <= 0
            or min(latency,scan_timeout,observation_timeout,timer_period,stop_margin) < 0):
        raise ValueError('Stopping horizon requires finite conservative limits')
    speed=max(max_speed,abs(measured_speed))
    horizon=(speed*(latency+scan_timeout+observation_timeout+timer_period)
             + speed**2/(2*decel) + stop_margin)
    points=transform_points(path,(0.,0.,0.))
    if len(points)<2:
        raise ValueError('Stopping corridor needs a valid path')
    result=[(0.,0.)]
    remaining=horizon
    for point in points:
        last=result[-1]
        length=math.dist(last,point)
        if length<1e-9:
            continue
        if length>=remaining:
            result.append(tuple(a+(b-a)*remaining/length for a,b in zip(last,point)))
            break
        result.append(point)
        remaining-=length
    if len(result)<2:
        raise ValueError('Stopping corridor has no forward extent')
    return tuple(result), horizon


def steering_corridor(distance, curvature, footprint_reach):
    """Constant-curvature command arc plus a discretization error bound."""
    if (not all(math.isfinite(v) for v in (distance,curvature,footprint_reach))
            or distance <= 0 or footprint_reach <= 0):
        raise ValueError('Invalid steering corridor')
    count=max(1,math.ceil(distance/.01),math.ceil(abs(curvature)*distance/.04))
    if count>1000:
        raise ValueError('Steering corridor exceeds bounded computation')
    points=[]
    for i in range(count+1):
        arc=distance*i/count
        if abs(curvature)<1e-9:
            points.append((arc,0.))
        else:
            angle=curvature*arc
            points.append((math.sin(angle)/curvature,(1.-math.cos(angle))/curvature))
    half_angle=abs(curvature)*distance/count/2
    # Chord-vs-arc position error, plus body heading-vs-chord orientation error.
    guard=(0. if abs(curvature)<1e-9 else (1.-math.cos(half_angle))/abs(curvature))
    guard+=2*footprint_reach*math.sin(half_angle/2)
    return tuple(points),guard


def swept_footprint_hit(points, path, footprint, padding):
    """Conservative polygon sweep, including turns between segment headings.

    Translation sweeps are exact convex hulls. At a heading change, the hull of
    endpoint footprints is expanded by the maximum vertex arc sagitta, covering
    the intervening rigid rotation. Initial heading is the current body x axis.
    """
    previous_heading = 0.
    reach = max(math.hypot(*v) for v in footprint)
    def intersects(polygon, margin):
        xmin=min(v[0] for v in polygon)-margin
        xmax=max(v[0] for v in polygon)+margin
        ymin=min(v[1] for v in polygon)-margin
        ymax=max(v[1] for v in polygon)+margin
        return any(xmin <= p[0] <= xmax and ymin <= p[1] <= ymax
                   and polygon_distance(p, polygon) <= margin for p in points)
    for a, b in zip(path, path[1:]):
        if math.dist(a, b) < 1e-9:
            continue
        heading = math.atan2(b[1]-a[1], b[0]-a[0])
        at_a = transform_points(footprint, (*a, heading))
        at_b = transform_points(footprint, (*b, heading))
        translation = convex_hull(at_a + at_b)
        old = transform_points(footprint, (*a, previous_heading))
        rotation = convex_hull(old + at_a)
        delta = abs(math.atan2(math.sin(heading-previous_heading),
                              math.cos(heading-previous_heading)))
        sagitta = reach * (1. - math.cos(delta / 2.))
        if intersects(translation, padding) or intersects(rotation, padding + sagitta):
            return True
        previous_heading = heading
    return False


def scan_collision(ranges, *, angle_min, angle_increment, range_min, range_max,
                   scan_pose, path, radius, age, timeout, infinity_is_clear=False,
                   footprint=None, padding=0., self_filter_bounds=None,
                   self_filter_pose=None, self_filter_radius=None,
                   steering_path=None, steering_guard=0.):
    """Return hit/no-hit/unknown for available laser returns in a swept corridor.

    radius is measured robot circumscribed footprint radius plus safety margin.
    A disk over-approximates footprint for every heading (conservative at corners).
    An explicit convex physical footprint replaces that disk with a conservative
    translational/rotational polygon sweep plus padding. Self-filtering is opt-in
    and bounded by the body envelope. A circular envelope may also hide nearby
    real objects; non-spatial/invalid returns outside the mask remain unknown.
    False means no sampled return collides, NOT proof of free unseen space: the
    ROS adapter must independently validate FOV/angular coverage of the corridor.
    Infinity is unknown unless the driver explicitly documents no-return as clear.
    This cannot detect objects below the scan plane; it never commands avoidance.
    """
    values = (angle_min, angle_increment, range_min, range_max, radius, age, timeout)
    if (not all(math.isfinite(v) for v in values) or radius <= 0 or timeout <= 0
            or not 0 <= age <= timeout or not 0 <= range_min < range_max
            or angle_increment == 0 or len(ranges) == 0 or len(path) < 2):
        return None
    try:
        path = transform_points(path, (0., 0., 0.))
        transform_points([], scan_pose)
        if footprint is not None:
            footprint = convex_footprint(footprint)
            if not math.isfinite(padding) or padding < 0:
                return None
            radius = max(math.hypot(*v) for v in footprint) + padding
        if self_filter_bounds is not None:
            if footprint is None:
                return None
            self_filter_bounds = validate_self_filter(self_filter_bounds, footprint)
        if self_filter_radius is not None:
            if footprint is None:
                return None
            validate_self_radius(self_filter_radius, footprint)
        if self_filter_bounds is not None or self_filter_radius is not None:
            transform_points([], self_filter_pose)
        if steering_path is not None:
            steering_path=transform_points(steering_path,(0.,0.,0.))
            if (len(steering_path)<2 or not math.isfinite(steering_guard)
                    or steering_guard<0):
                return None
    except (ValueError, TypeError):
        return None
    if max(math.dist(scan_pose[:2], p) for p in path) + radius > range_max:
        return None
    if steering_path is not None and max(math.dist(scan_pose[:2],p) for p in steering_path)+radius+steering_guard>range_max:
        return None
    laser_points = []
    self_mask = self_filter_bounds is not None or self_filter_radius is not None
    if self_mask:
        # Filter in scan-time body coordinates, BEFORE motion compensation.
        # Otherwise robot motion could move an external return into the mask.
        sx,sy,syaw=self_filter_pose
        c,s=math.cos(syaw),math.sin(syaw)
    for index, value in enumerate(ranges):
        if value == math.inf and infinity_is_clear is True:
            continue
        if not math.isfinite(value) or value <= 0:
            return None
        angle = angle_min + index * angle_increment
        point=(value * math.cos(angle), value * math.sin(angle))
        if self_mask:
            x,y=sx+c*point[0]-s*point[1],sy+s*point[0]+c*point[1]
            rectangle=(self_filter_bounds is not None
                       and self_filter_bounds[0]<=x<=self_filter_bounds[1]
                       and self_filter_bounds[2]<=y<=self_filter_bounds[3])
            circle=self_filter_radius is not None and math.hypot(x,y)<=self_filter_radius
            if rectangle or circle:
                # Explicit near-body mask also excludes finite positive returns
                # below the driver's range_min. NaN/negative/zero values cannot
                # establish a spatial position and always remain unknown.
                continue
        if not range_min <= value <= range_max:
            return None
        laser_points.append(point)
    points = transform_points(laser_points, scan_pose)
    swept = ((0., 0.),) + path
    if footprint is not None:
        return (swept_footprint_hit(points, swept, footprint, padding)
                or steering_path is not None and swept_footprint_hit(
                    points, steering_path, footprint, padding+steering_guard))
    if steering_path is not None and any(point_segment_distance(p,a,b)<=radius+steering_guard
                                        for p in points for a,b in zip(steering_path,steering_path[1:])):
        return True
    return any(point_segment_distance(p, a, b) <= radius
               for p in points for a, b in zip(swept, swept[1:]))
