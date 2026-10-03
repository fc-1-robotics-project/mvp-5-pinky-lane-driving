# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Pixel boundary traces; deliberately no metric or drivable-path claims."""

import heapq
import math


def trace_polygon(polygon, image_size):
    """Thin a filled instance then trace its near-to-far skeleton geodesic.

    Uses existing OpenCV/NumPy, no opencv-contrib dependency. Skeletonization is
    Zhang-Suen's two alternating deletion passes. Preserve instance boundaries;
    start at the bottom-most open endpoint, never the largest contour area.
    Pixel traces still require rectification/calibration and metric association.
    """
    import cv2
    import numpy as np

    width, height = image_size
    points = np.asarray(polygon, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 3
            or not np.isfinite(points).all() or width <= 0 or height <= 0):
        raise ValueError('Invalid polygon or image size')
    if (points[:, 0].min() < 0 or points[:, 0].max() > width
            or points[:, 1].min() < 0 or points[:, 1].max() > height):
        raise ValueError('Polygon outside image')
    contour = np.rint(points).astype(np.int32)
    if cv2.contourArea(contour) < 1:
        raise ValueError('Degenerate polygon')
    # Tight crop plus bounded working resolution keeps iterative thinning below
    # the live-frame latency budget on the Raspberry Pi.  The traced skeleton
    # is mapped back to original image coordinates before calibration.
    x, y, w, h = cv2.boundingRect(contour)
    scale = min(1., 160. / max(w, h))
    work_w = max(1, int(math.ceil(w * scale)))
    work_h = max(1, int(math.ceil(h * scale)))
    local = points - np.asarray((x, y), dtype=float)
    local = np.rint(local * scale + 1).astype(np.int32)
    mask = np.zeros((work_h + 2, work_w + 2), dtype=np.uint8)
    cv2.fillPoly(mask, [local], 1)
    while True:
        changed = False
        for phase in (0, 1):
            center = mask[1:-1, 1:-1]
            n = [mask[:-2, 1:-1], mask[:-2, 2:], mask[1:-1, 2:], mask[2:, 2:],
                 mask[2:, 1:-1], mask[2:, :-2], mask[1:-1, :-2], mask[:-2, :-2]]
            count = sum(n)
            transitions = sum(((a == 0) & (b == 1)).astype(np.uint8)
                              for a, b in zip(n, n[1:] + n[:1]))
            if phase == 0:
                preserve = (n[0] * n[2] * n[4] == 0) & (n[2] * n[4] * n[6] == 0)
            else:
                preserve = (n[0] * n[2] * n[6] == 0) & (n[0] * n[4] * n[6] == 0)
            remove = (center == 1) & (count >= 2) & (count <= 6) & (transitions == 1) & preserve
            if remove.any():
                center[remove] = 0
                changed = True
        if not changed:
            break
    nodes = {tuple(map(int, p)) for p in np.argwhere(mask)}

    def neighbors(p):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                q = (p[0] + dy, p[1] + dx)
                if (dy or dx) and q in nodes:
                    # A stair-step corner already has an orthogonal edge.
                    # Its redundant diagonal makes the terminal pixel appear
                    # branched (degree 2), hiding valid open-line endpoints.
                    if dy and dx and ((p[0], q[1]) in nodes or (q[0], p[1]) in nodes):
                        continue
                    yield q, math.hypot(dx, dy)

    ends = [p for p in nodes if sum(1 for _ in neighbors(p)) == 1]
    if len(ends) < 2:
        raise ValueError('Skeleton has no unambiguous open boundary')
    # On the calibrated ground view, increasing image v is the reliable
    # near-camera direction.  Distance to bottom-centre is not: an outer lane
    # endpoint can be lower in the image yet farther from bottom-centre in u,
    # which reverses only that boundary and breaks paired-path association.
    # Prefer the bottom-most endpoint, using horizontal distance only as a
    # deterministic tie-breaker.
    def image_point(point):
        return (
            x + (point[1] - 1) / scale,
            y + (point[0] - 1) / scale,
        )

    start = min(
        ends,
        key=lambda p: (
            -image_point(p)[1],
            abs(image_point(p)[0] - width / 2),
        ),
    )
    queue = [(0., start)]
    distance = {start: 0.}
    parent = {}
    while queue:
        cost, node = heapq.heappop(queue)
        if cost > distance[node]:
            continue
        for next_node, step in neighbors(node):
            candidate = cost + step
            if candidate < distance.get(next_node, math.inf):
                distance[next_node] = candidate
                parent[next_node] = node
                heapq.heappush(queue, (candidate, next_node))
    reachable = [p for p in ends if p != start and p in distance]
    if not reachable:
        raise ValueError('Disconnected skeleton')
    end = max(reachable, key=lambda p: distance[p])
    trace = [end]
    while trace[-1] != start:
        trace.append(parent[trace[-1]])
    return tuple(image_point(point) for point in reversed(trace))
