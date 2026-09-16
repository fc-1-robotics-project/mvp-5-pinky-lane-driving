# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Explicit camera calibration contract; no guessed scale or fabricated defaults."""

import math

from .perception import project_ground


class Calibration:
    """Homography consumes undistorted pixels with the same camera matrix P=K.

    source documents measurement provenance; mounting_id binds the physical camera
    pose. These labels are operator assertions, not automatic measurement proof.
    rectified_roi is a measured-valid rectangle, not the whole image by default.
    """

    def __init__(self, config):
        try:
            self.image_size = tuple(config['image_size'])
            self.camera_frame = config['camera_frame']
            self.mounting_id = config['mounting_id']
            self.source = config['source']
            self.lane_width = config['lane_width_m']
            self.matrix = tuple(tuple(row) for row in config['camera_matrix'])
            self.distortion = tuple(config['distortion'])
            self.homography = tuple(tuple(row) for row in config['homography'])
            self.roi = tuple(config['rectified_roi'])
            if config['ground_frame'] != 'base_footprint':
                raise ValueError('Ground projection must be in base_footprint metres')
            if (len(self.image_size) != 2
                    or any(type(v) is not int or v <= 0 for v in self.image_size)):
                raise ValueError('Image size must be positive integers')
            if any(not isinstance(v, str) or not v.strip()
                   for v in (self.camera_frame, self.mounting_id, self.source)):
                raise ValueError('Camera frame, mounting identity and provenance required')
            if not math.isfinite(self.lane_width) or self.lane_width <= 0:
                raise ValueError('Measured lane width must be positive metres')
            if (len(self.matrix) != 3 or any(len(row) != 3 for row in self.matrix)
                    or not all(math.isfinite(v) for row in self.matrix for v in row)
                    or self.matrix[0][0] <= 0 or self.matrix[1][1] <= 0
                    or self.matrix[2] != (0., 0., 1.)
                    or self.matrix[0][1] != 0 or self.matrix[1][0] != 0):
                raise ValueError('Expected finite pinhole intrinsic matrix')
            if (len(self.distortion) not in (4, 5, 8, 12, 14)
                    or not all(math.isfinite(v) for v in self.distortion)):
                raise ValueError('Unsupported distortion coefficients')
            if len(self.roi) != 4 or not all(math.isfinite(v) for v in self.roi):
                raise ValueError('ROI must be finite xmin,ymin,xmax,ymax')
            x0, y0, x1, y1 = self.roi
            if not (0 <= x0 < x1 <= self.image_size[0]
                    and 0 <= y0 < y1 <= self.image_size[1]):
                raise ValueError('Invalid calibrated ROI')
            project_ground(((x0, y0), (x1, y0), (x1, y1), (x0, y1)),
                           self.homography, rectified=True)
        except (KeyError, TypeError, IndexError) as error:
            raise ValueError('Incomplete or malformed calibration') from error

    def validate_source(self, image_size, frame_id, mounting_id):
        if (tuple(image_size) != self.image_size or frame_id != self.camera_frame
                or mounting_id != self.mounting_id):
            raise ValueError('Camera resolution, optical frame or mounting identity changed')

    def rectify(self, points):
        points = tuple(tuple(p) for p in points)
        if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('Pixels must be finite 2D points')
        if not points:
            return ()
        if any(self.distortion):
            import cv2
            import numpy as np
            points = cv2.undistortPoints(np.asarray(points, dtype=float).reshape(-1, 1, 2),
                                        np.asarray(self.matrix), np.asarray(self.distortion),
                                        P=np.asarray(self.matrix)).reshape(-1, 2)
        return tuple(tuple(map(float, p)) for p in points)

    def project(self, points):
        points = self.rectify(points)
        x0, y0, x1, y1 = self.roi
        if any(not x0 <= u <= x1 or not y0 <= v <= y1 for u, v in points):
            raise ValueError('Pixels are outside the measured calibration domain')
        return project_ground(points, self.homography, rectified=True)
