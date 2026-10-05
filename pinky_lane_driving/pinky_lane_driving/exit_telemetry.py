"""Small, motion-free evidence for the PC's map-position lane exit policy."""

import math

from .perception import observe


def boundary_visibility(observation):
    """Validate the perception contract; a malformed frame is never an empty frame."""
    detections = observation['detections']
    if not isinstance(detections, list):
        raise ValueError('Expected detection list')
    checked = observe(observation['capture_time_s'], observation['frame_id'],
                      observation['image_size'],
                      [d['class_id'] for d in detections],
                      [d['confidence'] for d in detections],
                      [d['polygon_px'] for d in detections])
    classes = {d['class_id'] for d in checked['detections']}
    return {'left_visible': 1 in classes, 'right_visible': 2 in classes}


class OdomStationarity:
    """Accumulate actual stillness, including rotation and movement between reports."""

    def __init__(self):
        self.anchor = self.since = self.stamp = self.received = None

    def update(self, pose, velocity, stamp, now, age):
        valid = (all(math.isfinite(v) for v in (*pose, *velocity, stamp, now, age))
                 and 0 <= age <= .3)
        continuous = (self.stamp is not None and stamp > self.stamp
                      and 0 <= now - self.received <= .3)
        still = (valid and math.hypot(*velocity[:2]) < .003
                 and abs(velocity[2]) < .02)
        moved = (self.anchor is None or math.dist(pose[:2], self.anchor[:2]) >= .003
                 or abs(math.remainder(pose[2] - self.anchor[2], 2 * math.pi)) >= .02)
        if not still or not continuous or moved:
            self.anchor = pose if valid else None
            self.since = now if still else None
        self.stamp, self.received = stamp, now

    def duration(self, now):
        if self.since is None or self.received is None or not 0 <= now - self.received <= .3:
            return None
        return max(0., now - self.since)
