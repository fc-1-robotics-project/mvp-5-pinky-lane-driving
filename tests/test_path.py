# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Current-lane geometry from metric near-to-far boundary polylines."""

import json
import math
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from pinky_lane_driving.control import Limits, command
from pinky_lane_driving.path import Boundary, PathSettings, select_path, sections, route_direction


class PathTest(unittest.TestCase):
    def setUp(self):
        self.config = PathSettings(width=.4, width_tolerance=.08, max_near=.6,
                                   sample_step=.05, fallback_timeout=.3,
                                   fallback_speed=.05, ambiguity_margin=.02)
        self.left = Boundary(1, ((.1, .2), (1., .2)))
        self.right = Boundary(2, ((.1, -.2), (1., -.2)))

    def select(self, boundaries, **kwargs):
        return select_path(boundaries, self.config, **kwargs)

    def stopped_frame(self):
        detections = json.loads(Path(__file__).with_name(
            'stopped_frame_boundaries.json').read_text())
        boundaries = [Boundary(d['class_id'], tuple(map(tuple, d['points'])))
                      for d in detections]
        config = replace(self.config, width=.17, sample_step=.03,
                         fallback_speed=.03, ambiguity_margin=.08,
                         allow_single_boundary_start=True)
        return boundaries, config

    def test_straight_center_and_opposite_lane_distractor(self):
        other = Boundary(1, ((.1, .6), (3., .6)))
        result = self.select([other, self.left, self.right])
        self.assertTrue(result.valid)
        self.assertEqual(result.selected, (1, 2))
        self.assertTrue(all(abs(y) < 1e-9 for _, y in result.points))
        self.assertFalse(result.degraded)

    def test_normal_sections_follow_120_degree_curve(self):
        angles = [math.radians(i * 3) for i in range(41)]
        left = Boundary(1, tuple((.8 * math.sin(a), 1 - .8 * math.cos(a)) for a in angles))
        right = Boundary(2, tuple((1.2 * math.sin(a), 1 - 1.2 * math.cos(a)) for a in angles))
        result = self.select([left, right])
        self.assertTrue(result.valid)
        self.assertGreater(result.points[-1][1], 1.3)
        self.assertTrue(all(abs(math.hypot(x, y - 1) - 1) < .04 for x, y in result.points))

    def test_sections_do_not_duplicate_roundoff_terminal_but_keep_real_tail(self):
        start = (.2, 0.)
        rounded_endpoint = (.35000000000000003, 0.)
        rounded = [point for point, _ in sections((start, rounded_endpoint), .03)]
        self.assertEqual(rounded[-1], rounded_endpoint)
        self.assertEqual(len(rounded), 6)
        self.assertTrue(all(math.dist(a, b) > 1e-9
                            for a, b in zip(rounded, rounded[1:])))
        limits = Limits(max_speed=.03, max_omega=.6, max_accel=.1,
                        max_lateral_accel=.12, braking_decel=.15, latency=.2,
                        stop_margin=.02, timeout=1.1, min_lookahead=.12,
                        max_lookahead=.35, lookahead_time=.1)
        proposal = command(rounded, age=.1, dt=.1, previous_speed=0.,
                           requested_speed=.03, metric_valid=True, limits=limits)
        self.assertEqual(proposal.reason, 'tracking')

        measured_endpoint = (.350000001, 0.)
        measured = [point for point, _ in sections((start, measured_endpoint), .03)]
        self.assertEqual(measured[-1], measured_endpoint)
        self.assertEqual(len(measured), 7)
        self.assertGreater(math.dist(measured[-2], measured[-1]), 1e-9)

    def test_missing_boundary_normal_offset_is_time_bounded(self):
        result = self.select([self.left], fallback_age=.1)
        self.assertTrue(result.valid)
        self.assertTrue(result.degraded)
        self.assertEqual(result.speed_limit, .05)
        self.assertTrue(all(abs(y) < 1e-9 for _, y in result.points))
        self.assertFalse(self.select([self.left]).valid)
        self.assertFalse(self.select([self.left], fallback_age=.31).valid)
        self.assertFalse(self.select([], fallback_age=.1).valid)

    def test_fresh_single_boundary_continues_with_previous_beyond_pair_lease(self):
        result = self.select([self.left], fallback_age=20., previous=((.1, 0.), (1., 0.)))
        self.assertTrue(result.valid)
        self.assertTrue(result.degraded)
        self.assertEqual(result.speed_limit, .05)
        self.assertFalse(self.select([], fallback_age=20., previous=result.points).valid)

    def test_tiny_detected_opposite_fragment_does_not_block_visible_single_side(self):
        config = replace(self.config, allow_single_boundary_start=True)
        tiny = Boundary(2, ((.1,-.2),(.12,-.2)))
        route = select_path([self.left,tiny],config)
        self.assertTrue(route.valid)
        self.assertTrue(route.degraded)
        self.assertEqual(route.selected,(0,))
        self.assertFalse(select_path([tiny],config).valid)

    def test_short_shared_pair_uses_long_corroborated_boundary(self):
        config = replace(self.config, allow_single_boundary_start=True)
        left = Boundary(1, ((.1, .2), (.4, .2)))
        right_tip = Boundary(2, ((.24, -.2), (.3, -.2)))
        route = select_path([left, right_tip], config)
        self.assertTrue(route.valid)
        self.assertTrue(route.degraded)
        self.assertEqual(route.selected, (0,))
        self.assertLess(route.points[0][0], .2)
        self.assertGreater(route.points[-1][0], .38)
        self.assertFalse(select_path([], config, previous=route.points).valid)

    def test_stopped_frame_uses_measured_reach_despite_folded_opposite_mask(self):
        boundaries, config = self.stopped_frame()
        arc = lambda points: sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        left, right = boundaries
        self.assertGreater(arc(right.points), 3 * config.sample_step)
        left_single = tuple((p[0] + n[0] * config.width / 2,
                             p[1] + n[1] * config.width / 2)
                            for p, n in sections(left.points, config.sample_step))
        right_single = tuple((p[0] - n[0] * config.width / 2,
                              p[1] - n[1] * config.width / 2)
                             for p, n in sections(right.points, config.sample_step))
        self.assertLess(arc(right_single), 3 * config.sample_step)
        self.assertGreater(arc(left_single), arc(right_single) + 2 * config.sample_step)
        paired = select_path(boundaries, replace(config, allow_single_boundary_start=False))
        self.assertEqual(paired.reason, 'paired')
        self.assertLess(arc(paired.points), 2 * config.sample_step)
        route = select_path(boundaries, config)
        self.assertEqual(route.reason, 'single_boundary')
        self.assertEqual(route.selected, (0,))
        self.assertEqual(route.points, left_single)
        self.assertGreater(arc(route.points), .20)

    def test_stopped_frame_disagreement_does_not_extend_short_pair(self):
        boundaries, config = self.stopped_frame()
        left, right = boundaries
        shifted = Boundary(2, tuple((x, y + .04) for x, y in right.points))
        route = select_path([left, shifted], config)
        self.assertEqual(route.reason, 'paired')
        self.assertLess(len(route.points), 4)
        prior_elsewhere = ((.2, .1), (.4, .1))
        self.assertFalse(select_path(boundaries, config,
                                     previous=prior_elsewhere).valid)

    def test_unpaired_curve_uses_long_measured_offset_not_raw_opposite_arc(self):
        detections = json.loads(Path(__file__).with_name(
            'unpaired_curve_boundaries.json').read_text())
        config = replace(self.config, width=.17, sample_step=.03,
                         fallback_speed=.03, ambiguity_margin=.08,
                         allow_single_boundary_start=True)
        arc = lambda points: sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        for mirror in (False, True):
            boundaries = [Boundary(3-d['class_id'] if mirror else d['class_id'],
                                   tuple((x, -y if mirror else y) for x, y in d['points']))
                          for d in detections]
            long_only = select_path(boundaries[:1], config)
            short_only = select_path(boundaries[1:], config)
            self.assertGreater(arc(boundaries[1].points), 3 * config.sample_step)
            self.assertLess(arc(short_only.points), 3 * config.sample_step)
            self.assertGreater(arc(long_only.points), .25)
            route = select_path(boundaries, config)
            self.assertEqual(route.reason, 'single_boundary')
            self.assertEqual(route.selected, (0,))
            self.assertEqual(route.points, long_only.points)
            without_single = select_path(boundaries, replace(
                config, allow_single_boundary_start=False))
            self.assertNotEqual(without_single.reason, 'single_boundary')
            if not mirror:
                self.assertFalse(without_single.valid)
            self.assertFalse(select_path(boundaries, config,
                previous=((.15, .4), (.45, .4))).valid)
            self.assertFalse(select_path([], config, previous=route.points).valid)

    def test_short_pair_is_not_extended_without_single_start_permission(self):
        left = Boundary(1, ((.1, .2), (.4, .2)))
        right_tip = Boundary(2, ((.24, -.2), (.3, -.2)))
        route = self.select([left, right_tip])
        self.assertTrue(route.valid)
        self.assertFalse(route.degraded)
        self.assertEqual(route.selected, (0, 1))
        self.assertGreater(route.points[0][0], .2)

    def test_pair_chooses_later_longer_measured_run_without_bridging_gap(self):
        # A near pair is interrupted by a right-mask excursion. Both masks
        # resume their measured, width-consistent overlap farther ahead.
        left = Boundary(1, ((.1, .2), (.6, .2)))
        right = Boundary(2, ((.1, -.2), (.18, -.2), (.2, -.5),
                             (.22, -.5), (.24, -.2), (.65, -.2)))
        route = self.select([left, right])
        self.assertTrue(route.valid)
        self.assertEqual(route.reason, 'paired')
        self.assertEqual(route.selected, (0, 1))
        self.assertGreaterEqual(route.points[0][0], .24)
        self.assertGreater(route.points[-1][0] - route.points[0][0], .2)
        self.assertTrue(all(abs(y) < 1e-8 for _, y in route.points))

    def test_far_longest_pair_does_not_hide_reachable_near_run(self):
        left = Boundary(1, ((.1, .2), (1.2, .2)))
        right = Boundary(2, ((.1, -.2), (.18, -.2), (.22, -.35),
                             (.8, -.35), (.84, -.2), (1.25, -.2)))
        route = self.select([left, right])
        self.assertTrue(route.valid)
        self.assertEqual(route.reason, 'paired')
        self.assertLess(route.points[0][0], .2)
        self.assertLess(route.points[-1][0], .4)

    def test_later_pair_run_rejects_contradictory_prior_route(self):
        left = Boundary(1, ((.1, .2), (.6, .2)))
        right = Boundary(2, ((.1, -.2), (.18, -.2), (.2, -.5),
                             (.22, -.5), (.24, -.2), (.65, -.2)))
        prior = ((.1, .2), (.2, .2))
        route = self.select([left, right], previous=prior)
        self.assertFalse(route.valid)
        self.assertEqual(route.reason, 'no_current_lane')

    def test_later_pair_run_may_extend_just_beyond_prior_tip(self):
        left = Boundary(1, ((.1, .2), (.6, .2)))
        right = Boundary(2, ((.1, -.2), (.18, -.2), (.2, -.5),
                             (.22, -.5), (.24, -.2), (.65, -.2)))
        route = self.select([left, right], previous=((.1, 0.), (.2, 0.)))
        self.assertTrue(route.valid)
        self.assertGreaterEqual(route.points[0][0], .24)
        self.assertGreater(route.points[-1][0], .55)

    def test_later_run_rejects_large_right_boundary_arc_regression(self):
        near = ((.1, 0.), (.15, 0.))
        later = ((.25, 0.), (.35, 0.), (.45, 0.), (.55, 0.))
        with patch('pinky_lane_driving.path.paired_runs', return_value=(
                (near, None, 0.),
                (later, near[-1], .03))):
            route = self.select([self.left, self.right])
        self.assertTrue(route.valid)
        self.assertEqual(route.points, near)
        with patch('pinky_lane_driving.path.paired_runs', return_value=(
                (near, None, 0.),
                (later, near[-1], .01))):
            route = self.select([self.left, self.right])
        self.assertTrue(route.valid)
        self.assertEqual(route.points, later)

    def test_two_long_edges_with_short_overlap_use_measured_prior_aligned_edge(self):
        left = Boundary(1, ((.1, .2), (.35, .2)))
        right = Boundary(2, ((.29, -.2), (.54, -.2)))
        previous = ((.1, 0.), (.54, 0.))
        route = self.select([left, right], previous=previous)
        self.assertTrue(route.valid)
        self.assertTrue(route.degraded)
        self.assertEqual(route.reason, 'single_boundary')
        self.assertEqual(route.selected, (0,))
        self.assertGreater(route.points[-1][0] - route.points[0][0], .2)
        # A single small shared arc is insufficient without prior geometry.
        unprimed = self.select([left, right])
        self.assertEqual(unprimed.reason, 'paired')
        self.assertLess(unprimed.points[-1][0] - unprimed.points[0][0], .1)
        # A substantial but shorter mask is not a disappearing tip.
        longer = Boundary(1, ((.05, .2), (.35, .2)))
        self.assertEqual(self.select([longer, right]).reason, 'paired')

    def test_short_pair_extension_requires_both_edges_and_prior_to_agree(self):
        left = Boundary(1, ((.1, .2), (.35, .2)))
        right = Boundary(2, ((.29, -.2), (.54, -.2)))
        previous = ((.1, 0.), (.54, 0.))
        divergent = Boundary(2, ((.29, -.2), (.54, -.28)))
        route = self.select([left, divergent], previous=previous)
        self.assertNotEqual(route.reason, 'single_boundary')
        shifted_prior = ((.1, .1), (.54, .1))
        route = self.select([left, right], previous=shifted_prior)
        self.assertNotEqual(route.reason, 'single_boundary')

    def test_unpaired_short_opposite_fragment_does_not_hide_long_side(self):
        config = replace(self.config, allow_single_boundary_start=True)
        long_left = Boundary(1, ((.1, .2), (.5, .2)))
        short_wrong_width = Boundary(2, ((.24, -.3), (.3, -.3)))
        route = select_path([long_left, short_wrong_width], config)
        self.assertTrue(route.valid)
        self.assertTrue(route.degraded)
        self.assertEqual(route.selected, (0,))
        self.assertFalse(self.select([long_left, short_wrong_width]).valid)

    def test_explicit_single_start_side_and_ambiguity_checks(self):
        config = replace(self.config, allow_single_boundary_start=True)
        self.assertTrue(select_path([self.right], config).valid)
        self.assertFalse(select_path([Boundary(1, ((.1, -.2), (1., -.2)))], config).valid)
        self.assertTrue(select_path([self.left, self.left], config).valid)
        self.assertFalse(select_path([], config).valid)

    def test_single_boundary_cannot_jump_to_another_lane(self):
        previous = ((.1, 0.), (1., 0.))
        shifted = Boundary(1, ((.1, .31), (1., .31)))
        self.assertFalse(self.select([shifted], fallback_age=2., previous=previous).valid)
        # A clipped near endpoint may move along the same path without a jump.
        clipped = Boundary(2, ((.25, -.2), (1., -.2)))
        self.assertTrue(self.select([clipped], fallback_age=2., previous=previous).valid)

    def test_wrong_width_crossed_or_disconnected_are_invalid(self):
        for right in [Boundary(2, ((.1, -.8), (1., -.8))),
                      Boundary(2, ((.1, .3), (1., .3))),
                      Boundary(2, ((2., -.2), (3., -.2)))]:
            self.assertFalse(self.select([self.left, right]).valid)

    def test_overlapping_duplicate_candidates_are_the_same_route(self):
        self.assertTrue(self.select([self.left, self.left, self.right]).valid)
        jitter = Boundary(1, ((.105, .205), (1.005, .205)))
        self.assertTrue(self.select([self.left, jitter], fallback_age=.1).valid)

    def test_distinct_close_scored_routes_remain_ambiguous(self):
        shifted = Boundary(1, ((.1, .23), (1., .23)))
        result = self.select([self.left, shifted], fallback_age=.1)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, 'ambiguous_boundaries')
        # Similar endpoints do not establish equivalence if the middle splits.
        split = Boundary(1, ((.1,.2),(.5,.32),(1.,.2)))
        config = replace(self.config, ambiguity_margin=.2)
        result = select_path([self.left, split], config, fallback_age=.1)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, 'ambiguous_boundaries')

    def test_duplicate_mask_does_not_hide_a_third_distinct_candidate(self):
        shifted = Boundary(1, ((.1,.23),(1.,.23)))
        self.assertEqual(self.select([self.left,self.left,shifted],fallback_age=.1).reason,
                         'ambiguous_boundaries')

    def test_previous_path_must_be_in_current_frame(self):
        self.assertTrue(self.select([self.left, self.right],
                                   previous=((.1, 0.), (.5, 0.))).valid)
        self.assertFalse(self.select([self.left, self.right],
                                    previous=((.1, 2.), (.5, 2.))).valid)

    def test_raster_steps_and_mismatched_near_endpoints_still_pair(self):
        left = Boundary(1, tuple(
            (.195 + index * .001, .06 + (index % 2) * .0002)
            for index in range(161)
        ))
        right = Boundary(2, tuple(
            (.199 + index * .001, -.34 - (index % 2) * .0002)
            for index in range(157)
        ))
        result = self.select([left, right])
        self.assertTrue(result.valid)
        self.assertGreater(len(result.points), 2)

    def test_clipped_far_boundary_keeps_only_shared_contiguous_segment(self):
        left = Boundary(1, ((.1, .2), (1., .2)))
        right = Boundary(2, ((.1, -.2), (.45, -.2)))
        result = self.select([left, right])
        self.assertTrue(result.valid)
        self.assertFalse(result.degraded)
        self.assertLessEqual(result.points[-1][0], .45 + 1e-9)
        self.assertGreater(result.points[-1][0] - result.points[0][0], .2)

    def test_short_final_raster_remainder_does_not_flip_offset_normal(self):
        points = ((.1, .2), (.25, .2), (.251, .201))
        sample = sections(points, .05)[-1]
        self.assertLess(abs(sample[1][0]), .1)
        self.assertLess(sample[1][1], -.99)

    def test_endpoint_normals_keep_the_same_full_metric_window(self):
        points=((.1,.2),(.105,.203),(.13,.2),(.16,.2),(.2,.2))
        samples=sections(points,.03)
        # The first pixel-tip kink must not establish a different normal from
        # the immediately following sample of the same measured straight line.
        self.assertEqual(samples[0][1],samples[1][1])
        config=replace(self.config, sample_step=.03,allow_single_boundary_start=True)
        noisy=Boundary(1,points)
        fresh=select_path([noisy],config)
        continued=select_path([noisy],config,previous=fresh.points)
        self.assertTrue(fresh.valid)
        self.assertTrue(continued.valid)

    def test_continuity_heading_ignores_short_raster_backtrack(self):
        points=((.15455,.00889),(.17305,.00055),(.15838,.01938),
                (.15456,.01394),(.17319,-.02692),(.2,-.06))
        # At the nearest short segment the raw x direction is backwards, but
        # the measured route still continues forward and right around a curve.
        direction=route_direction(points,2,.06)
        self.assertLess(direction[1],0.)
        fresh=((.14239,.01741),(.16125,.00792),(.15484,.0138),
               (.16147,-.00094),(.17641,-.02703),(.2,-.06))
        current=route_direction(fresh,0,.06)
        cosine=sum(a*b for a,b in zip(current,direction))/(math.hypot(*current)*math.hypot(*direction))
        self.assertGreater(cosine,.5)
        reversed_direction=route_direction(tuple(reversed(points)),len(points)-3,.06)
        cosine=sum(a*b for a,b in zip(current,reversed_direction))/(math.hypot(*current)*math.hypot(*reversed_direction))
        self.assertLess(cosine,.5)

    def test_invalid_geometry_and_settings(self):
        with self.assertRaises(ValueError):
            replace(self.config, blind_timeout=.5)
        with self.assertRaises(ValueError):
            replace(self.config, blind_timeout=.5, blind_distance=.5,
                    blind_speed=.02)
        with self.assertRaises(ValueError):
            Boundary(1, ((float('nan'), 0.), (1., 0.)))
        with self.assertRaises(ValueError):
            Boundary(0, ((0., 0.), (1., 0.)))
