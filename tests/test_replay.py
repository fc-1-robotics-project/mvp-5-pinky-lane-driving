# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Replay timing and model-contract tests, independent of YOLO and hardware."""

import math
import unittest
from types import SimpleNamespace

from tools.replay import frame_plan, validate_model
from tools.replay import observe_result


class ReplayTest(unittest.TestCase):
    def test_model_adapter_preserves_source_time_and_instances(self):
        class Values(list):
            def tolist(self):
                return list(self)

        result = SimpleNamespace(
            orig_shape=(480, 640),
            boxes=SimpleNamespace(cls=Values([0., 1., 1.]), conf=Values([.9] * 3)),
            masks=SimpleNamespace(xy=[Values([(10., 20.), (30., 20.), (20., 40.)])] * 3))
        row = observe_result(result, 133.)
        self.assertEqual(row['capture_time_s'], 133.)
        self.assertEqual(row['image_size'], (640, 480))
        self.assertEqual(len(row['detections']), 3)
        self.assertFalse(row['metric_valid'])
        result.masks = None
        with self.assertRaises(ValueError):
            observe_result(result, 133.)

    def test_half_open_window_and_original_timing(self):
        frames, rate = frame_plan(30., 9000, 133., 136., 5.)
        self.assertEqual(list(frames), list(range(3990, 4080, 6)))
        self.assertEqual(rate, 5.)
        self.assertEqual(len(frames), 15)

    def test_fractional_fps_uses_frame_timestamps(self):
        frames, rate = frame_plan(29.998, 5462, 109., 112., 5.)
        self.assertTrue(all(109 <= i / 29.998 < 112 for i in frames))
        self.assertAlmostEqual(rate, 29.998 / 6)

    def test_invalid_or_out_of_range_windows_fail(self):
        for fps, total, start, end, rate in [
                (0, 100, 0, 1, 5), (30, 0, 0, 1, 5),
                (30, 100, -1, 1, 5), (30, 100, 1, 1, 5),
                (30, 100, 0, 4, 5), (30, 100, 0, 1, 0),
                (math.nan, 100, 0, 1, 5), (30, 100, 0, math.inf, 5)]:
            with self.subTest(values=(fps, total, start, end, rate)):
                with self.assertRaises(ValueError):
                    frame_plan(fps, total, start, end, rate)

    def test_model_must_match_segmentation_class_contract(self):
        names = {0: 'crosswalk', 1: 'left line', 2: 'right line'}
        validate_model('segment', names)
        with self.assertRaises(ValueError):
            validate_model('detect', names)
        with self.assertRaises(ValueError):
            validate_model('segment', {0: 'left line', 1: 'crosswalk', 2: 'right line'})


if __name__ == '__main__':
    unittest.main()
