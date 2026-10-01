# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Current-lane selection from calibrated, near-to-far boundary polylines."""

from dataclasses import dataclass, fields
import bisect
import math

from .obstacles import point_segment_distance


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
    allow_single_boundary_start: bool = False
    blind_timeout: float = 0.
    blind_distance: float = 0.
    blind_speed: float = 0.

    def __post_init__(self):
        blind = ('blind_timeout', 'blind_distance', 'blind_speed')
        if any(not math.isfinite(getattr(self, f.name)) or getattr(self, f.name) <= 0
               for f in fields(self) if f.name not in ('allow_single_boundary_start', *blind)):
            raise ValueError('Path limits must be finite and positive')
        if type(self.allow_single_boundary_start) is not bool:
            raise ValueError('Single-boundary startup requires an explicit boolean')
        if self.width_tolerance >= self.width or not 0 < self.min_coverage <= 1:
            raise ValueError('Invalid width tolerance or coverage')
        values = tuple(getattr(self, name) for name in blind)
        if (any(not math.isfinite(value) or value < 0 for value in values)
                or any(values) and (not all(values) or self.blind_distance > self.width
                                   or self.blind_speed > self.fallback_speed)):
            raise ValueError('Blind continuation must have finite bounded time, distance and speed')


@dataclass(frozen=True)
class LanePath:
    points: tuple = ()
    selected: tuple = ()
    valid: bool = False
    degraded: bool = False
    speed_limit: float | None = None
    reason: str = 'no_current_lane'


def sections(points, step):
    """Uniform arc samples with normals spanning one metric sample interval.

    Pixel skeletons project to staircase-like metric polylines.  A normal from
    every one-pixel segment can therefore point sideways even when the lane is
    smooth.  Resampling before taking a centred tangent makes association
    depend on measured lane geometry rather than raster step direction.
    """
    cumulative = [0.]
    for a, b in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.dist(a, b))
    total = cumulative[-1]
    distances = [0.]
    distance = step
    # Repeated addition can put a nominal final grid point one or two ULPs
    # before the measured endpoint.  Emitting both produces a near-zero last
    # segment that the controller correctly rejects as invalid geometry.
    # Limit the tolerance so a genuinely measured terminal segment of at
    # least 1 nm is always retained at the scale of this controller.
    terminal_roundoff = min(1e-10, 8 * math.ulp(total))
    while distance < total and total - distance > terminal_roundoff:
        distances.append(distance)
        distance += step
    distances.append(total)

    def point_at(distance):
        index = min(
            bisect.bisect_right(cumulative, distance) - 1,
            len(points) - 2,
        )
        length = cumulative[index + 1] - cumulative[index]
        fraction = (distance - cumulative[index]) / length
        a, b = points[index], points[index + 1]
        return (
            a[0] + fraction * (b[0] - a[0]),
            a[1] + fraction * (b[1] - a[1]),
        )

    result = []
    for distance in distances:
        point = point_at(distance)
        # Use a measured arc-length window even at a short final remainder.
        # The last one-pixel segment is not a reliable endpoint tangent.
        # Retain the full two-step tangent window at either endpoint. A half
        # window there magnifies mask-tip jitter and can make the offset path
        # briefly double back, falsely failing temporal direction continuity.
        low = max(0., distance-step)
        high = min(total, low+2*step)
        low = max(0., high-2*step)
        before = point_at(low)
        after = point_at(high)
        dx, dy = after[0] - before[0], after[1] - before[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            raise ValueError('Boundary tangent is undefined')
        result.append((point, (dy / length, -dx / length)))
    return result


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def route_direction(points, index, window):
    """A measured arc-window heading, not a noisy one-pixel segment."""
    cumulative = [0.]
    for a, b in zip(points, points[1:]):
        cumulative.append(cumulative[-1]+math.dist(a, b))
    low = max(0., cumulative[index]-window/2)
    high = min(cumulative[-1], low+window)
    low = max(0., high-window)

    def at(distance):
        segment = min(bisect.bisect_right(cumulative, distance)-1, len(points)-2)
        length = cumulative[segment+1]-cumulative[segment]
        if length <= 1e-9:
            raise ValueError('Undefined route direction')
        fraction = (distance-cumulative[segment])/length
        return tuple(a+(b-a)*fraction for a, b in zip(points[segment], points[segment+1]))

    return tuple(b-a for a, b in zip(at(low), at(high)))


def equivalent_paths(first, second, tolerance):
    """Recognize repeated masks of the same route, not intersecting routes.

    Require matching near/far endpoints, direction and the WHOLE polyline in
    both directions. Sample spacing and a distance guard bound unsampled
    segment interiors too: distance to a polyline is 1-Lipschitz. A diverging
    branch or neighbouring lane must retain the ambiguity stop.
    """
    if any(math.dist(a, b) > tolerance for a, b in
           ((first[0], second[0]), (first[-1], second[-1]))):
        return False
    a = tuple(v-u for u, v in zip(first[0], first[-1]))
    b = tuple(v-u for u, v in zip(second[0], second[-1]))
    length = math.hypot(*a) * math.hypot(*b)
    if length <= 1e-12 or sum(u*v for u, v in zip(a, b))/length < .95:
        return False
    for source, target in ((first, second), (second, first)):
        segments = tuple(zip(target, target[1:]))
        for a, b in zip(source, source[1:]):
            count = max(1, math.ceil(math.dist(a, b)/(tolerance/4)))
            for index in range(count+1):
                point = tuple(u+(v-u)*index/count for u, v in zip(a, b))
                if min(point_segment_distance(point, u, v) for u, v in segments) > tolerance*7/8:
                    return False
    return True


def paired_runs(left, right, config):
    samples = sections(left, config.sample_step)
    runs = []
    centers = []
    preceding_hit = None
    preceding_arc = None
    run_preceding_hit = None
    run_arc_regression = 0.
    last_arc = -1.
    for point, normal in samples:
        crossings = []
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
                        and abs(width - config.width) <= config.width_tolerance):
                    crossings.append((abs(width - config.width), width, position))
            arc += length
        hits = [hit for hit in crossings if hit[2] >= last_arc - 1e-6]
        if not hits:
            # A spurious near crossing or one unmatched sample must not hide
            # a later, longer run where BOTH measured boundaries corroborate
            # the lane. Close the current run instead of bridging its gap.
            if len(centers) >= 2:
                runs.append((tuple(centers), run_preceding_hit, run_arc_regression))
            if centers:
                preceding_hit = centers[-1]
                preceding_arc = last_arc
            centers = []
            last_arc = -1.
            hits = crossings
            if not hits:
                continue
        _, width, last_arc = min(hits)
        if not centers:
            run_preceding_hit = preceding_hit
            run_arc_regression = (max(0., preceding_arc - last_arc)
                                  if preceding_arc is not None else 0.)
        centers.append((point[0] + normal[0] * width / 2,
                        point[1] + normal[1] * width / 2))
    if len(centers) >= 2:
        runs.append((tuple(centers), run_preceding_hit, run_arc_regression))
    # A curved inner and outer boundary leave the calibrated image at different
    # arc positions. Keep their contiguous, width-validated shared segment;
    # dividing by the ENTIRE outer mask wrongly rejects valid curve entries.
    # The controller still brakes for short available path length. No gap is
    # bridged: every missing cross-section terminates its own run.
    return tuple(runs)


def paired_path(left, right, config):
    """Geometry-only longest run; select_path applies reach and continuity gates."""
    runs = paired_runs(left, right, config)
    return max((points for points, _, _ in runs), key=lambda points: sum(
        math.dist(a, b) for a, b in zip(points, points[1:]))) if runs else ()


def select_path(boundaries, config, *, fallback_age=None, previous=()):
    """Select the currently occupied right-hand lane, not the largest instance.

    Left/right classes already mean current-lane boundaries. Inputs are ordered
    near-to-far in CURRENT base_footprint. previous must be odometry-transformed
    to this same frame; it is only a continuity hint, never reused as a path.
    A fresh single boundary can continue a previously validated, odometry-aligned
    path indefinitely; the age of the last PAIR is not the age of that boundary.
    Without continuity, startup requires an explicit calibrated-width opt-in or
    the existing short paired-observation lease. No boundary means no path.
    """
    if previous and (len(previous) < 2
                     or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in previous)):
        return LanePath(reason='invalid_previous')
    def arc_length(points):
        return sum(math.dist(a, b) for a, b in zip(points, points[1:]))

    # An instance with less than one metric sample interval is not a usable
    # boundary. A few pixels of the disappearing side must not suppress the
    # calibrated single-boundary path from the other, fully visible side.
    usable = [(i, b) for i, b in enumerate(boundaries)
              if arc_length(b.points) >= config.sample_step]
    lefts = [(i, b.points) for i, b in usable if b.class_id == 1]
    rights = [(i, b.points) for i, b in usable if b.class_id == 2]
    candidates = []
    recent_pair = (fallback_age is not None and math.isfinite(fallback_age)
                   and 0 <= fallback_age <= config.fallback_timeout)
    single_allowed = bool(previous or recent_pair or config.allow_single_boundary_start)

    def single_points(boundary):
        direction = 1 if boundary.class_id == 1 else -1
        samples = sections(boundary.points, config.sample_step)
        point, normal = samples[0]
        origin_side = direction * (-point[0] * normal[0] - point[1] * normal[1])
        if not -config.width_tolerance <= origin_side <= config.width + config.width_tolerance:
            return ()
        offset = direction * config.width / 2
        return tuple((p[0] + n[0] * offset, p[1] + n[1] * offset)
                     for p, n in samples)

    def follows(points, reference, tolerance):
        segments = tuple(zip(reference, reference[1:]))
        return bool(segments) and all(
            min(point_segment_distance(point, a, b) for a, b in segments)
            <= tolerance for point in points)

    def paired_start_agrees(points):
        """A later separate run must begin along the previous route heading."""
        if not previous:
            return True
        index = min(range(len(previous) - 1), key=lambda i:
                    point_segment_distance(points[0], previous[i], previous[i + 1]))
        current = route_direction(points, 0, 2 * config.sample_step)
        old = route_direction(previous, index, 2 * config.sample_step)
        length = math.hypot(*current) * math.hypot(*old)
        if length <= 1e-12 or sum(a * b for a, b in zip(current, old)) / length < .5:
            return False
        if (point_segment_distance(points[0], previous[index], previous[index + 1])
                <= config.width_tolerance / 2):
            return True
        # A fresh measured run may start just beyond the old measured tip.
        # Check a short forward extension of the old tangent; this permits
        # newly visible road without extrapolating it into the drive path.
        endpoint = previous[-1]
        end_direction = route_direction(previous, len(previous) - 1,
                                        2 * config.sample_step)
        scale = math.hypot(*end_direction)
        unit = (end_direction[0] / scale, end_direction[1] / scale)
        offset = (points[0][0] - endpoint[0], points[0][1] - endpoint[1])
        along = sum(a * b for a, b in zip(offset, unit))
        lateral = abs(cross(offset, unit))
        return (0 <= along <= 2 * config.sample_step
                and lateral <= config.width_tolerance / 2)

    def paired_run_is_continuous(points, prior_hit, arc_regression):
        if arc_regression > config.sample_step / 2 + 1e-9:
            return False
        if previous and not paired_start_agrees(points):
            return False
        if prior_hit is None:
            return True
        return (bool(previous) or
                math.dist(points[0], prior_hit) <= 2 * config.sample_step + 1e-9)

    def accept(points, selected, degraded):
        if len(points) < 2 or math.hypot(*points[0]) > config.max_near or points[0][0] < 0:
            return
        continuity = math.dist(points[0], previous[0]) if previous else 0.
        if degraded and previous:
            # Compare overlapping geometry, not unrelated near endpoints. This
            # prevents switching to a neighbouring lane when the visible side
            # changes, while allowing normal camera-crop/odometry differences.
            index = min(range(len(previous) - 1), key=lambda i:
                        point_segment_distance(points[0], previous[i], previous[i + 1]))
            continuity = point_segment_distance(points[0], previous[index], previous[index + 1])
            if continuity > config.width_tolerance:
                return
            current = route_direction(points, 0, 2*config.sample_step)
            old = route_direction(previous, index, 2*config.sample_step)
            length = math.hypot(*current) * math.hypot(*old)
            if length <= 1e-12 or sum(a * b for a, b in zip(current, old)) / length < .5:
                return
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
            runs = paired_runs(left, right, config)
            # Pick the longest run the controller could actually reach. A
            # farther run cannot suppress a shorter, near valid one. If a
            # later separate run wins, check its start and heading against
            # the odometry-aligned previous route when one exists.
            feasible = [(index, points) for index, (points, prior_hit, regression)
                        in enumerate(runs)
                        if math.hypot(*points[0]) <= config.max_near
                        and points[0][0] >= 0
                        and paired_run_is_continuous(points, prior_hit, regression)]
            shared = (max(feasible, key=lambda item: arc_length(item[1]))[1]
                      if feasible else ())
            if (single_allowed and shared
                    and arc_length(shared) < 2 * config.sample_step):
                # A disappearing side may still emit a mask just long enough
                # to count as a boundary. Its tiny shared arc must not hide a
                # much longer, fresh opposite edge. Require a clear length
                # advantage AND agreement with the measured paired segment.
                left_length, right_length = arc_length(left), arc_length(right)
                left_single = single_points(boundaries[li])
                right_single = single_points(boundaries[ri])
                # Raster folds add raw boundary arc without adding forward
                # measured reach. Compare the paths we could actually use.
                left_reach, right_reach = (arc_length(left_single),
                                           arc_length(right_single))
                dominant = []
                if (left_single and right_single
                        and right_reach < 3 * config.sample_step
                        and left_reach >= max(3 * config.sample_step,
                                              right_reach + 2 * config.sample_step)):
                    dominant.append((li, left_single))
                if (left_single and right_single
                        and left_reach < 3 * config.sample_step
                        and right_reach >= max(3 * config.sample_step,
                                               left_reach + 2 * config.sample_step)):
                    dominant.append((ri, right_single))
                extended_accepted = False
                for index, extended in dominant:
                    if (extended and all(
                            min(point_segment_distance(point, a, b)
                                for a, b in zip(extended, extended[1:]))
                            <= config.width_tolerance / 4
                            for point in shared)):
                        before = len(candidates)
                        accept(extended, (index,), True)
                        if len(candidates) > before:
                            extended_accepted = True
                            break
                if extended_accepted:
                    continue
                if (previous and left_length >= 3 * config.sample_step
                        and right_length >= 3 * config.sample_step):
                    # Both masks can be long yet share only a short visible
                    # cross-section. A fresh measured edge may extend that
                    # section only when BOTH offsets corroborate the pair and
                    # the whole proposed route agrees with odometry-aligned
                    # prior geometry. Do not invent a path beyond either.
                    singles = ((li, single_points(boundaries[li])),
                               (ri, single_points(boundaries[ri])))
                    tolerance = config.width_tolerance / 4
                    if all(points and follows(shared, points, tolerance)
                           for _, points in singles):
                        supported = [
                            (index, points) for index, points in singles
                            if (arc_length(points) >= arc_length(shared)
                                + 2 * config.sample_step
                                and follows(points, previous, tolerance))
                        ]
                        if supported:
                            # Prefer the nearer start, then greater measured
                            # reach. Accept still enforces heading continuity.
                            index, points = min(
                                supported,
                                key=lambda item: (math.hypot(*item[1][0]),
                                                  -arc_length(item[1])))
                            before = len(candidates)
                            accept(points, (index,), True)
                            if len(candidates) > before:
                                continue
            accept(shared, (li, ri), False)
    if not candidates and single_allowed and lefts and rights:
        # A short opposite mask can be inside the ROI yet fail width pairing
        # entirely as it exits the camera. Treat it like a disappearing side
        # only when the other side is clearly longer. Two substantial,
        # contradictory boundaries must still stop as ambiguous geometry.
        left_max = max(arc_length(points) for _, points in lefts)
        right_max = max(arc_length(points) for _, points in rights)
        if right_max <= 3 * config.sample_step:
            for index, points in lefts:
                if arc_length(points) >= max(3 * config.sample_step,
                                             right_max + 2 * config.sample_step):
                    accept(single_points(boundaries[index]), (index,), True)
        if left_max <= 3 * config.sample_step:
            for index, points in rights:
                if arc_length(points) >= max(3 * config.sample_step,
                                             left_max + 2 * config.sample_step):
                    accept(single_points(boundaries[index]), (index,), True)
        if not candidates:
            # As in the short-pair case above, raw mask arc can overstate the
            # usable center path of a folded/curved opposite instance. Apply
            # the same dominance test to measured normal-offset paths even
            # when the two instances have no width-valid paired section.
            # Require every offset to pass the occupied-side gate; an invalid
            # substantial instance is not silently treated as zero reach.
            left_offsets = [(i, single_points(boundaries[i])) for i, _ in lefts]
            right_offsets = [(i, single_points(boundaries[i])) for i, _ in rights]
            if all(points for _, points in left_offsets + right_offsets):
                left_reach = max(arc_length(points) for _, points in left_offsets)
                right_reach = max(arc_length(points) for _, points in right_offsets)
                for offsets, opposite_reach in ((left_offsets, right_reach),
                                                (right_offsets, left_reach)):
                    if opposite_reach <= 3 * config.sample_step:
                        for index, points in offsets:
                            if arc_length(points) >= max(
                                    3 * config.sample_step,
                                    opposite_reach + 2 * config.sample_step):
                                accept(points, (index,), True)
    if not lefts or not rights:
        if not single_allowed:
            return LanePath(reason='fallback_expired_or_unavailable')
        for index, boundary in usable:
            accept(single_points(boundary), (index,), True)
    candidates.sort(key=lambda item: item[0])
    if not candidates:
        return LanePath()
    # YOLO can emit two overlapping instances for one physical line. Do not
    # confuse geometrically equivalent paths with two possible lanes. Compare
    # every close-scored alternative to the winner (no transitive clustering).
    duplicate_tolerance = min(config.width_tolerance/4, config.width/8,
                              config.sample_step)
    for score, candidate in candidates[1:]:
        if score-candidates[0][0] >= config.ambiguity_margin:
            break
        if not equivalent_paths(candidates[0][1].points, candidate.points,
                                duplicate_tolerance):
            return LanePath(reason='ambiguous_boundaries')
    return candidates[0][1]
