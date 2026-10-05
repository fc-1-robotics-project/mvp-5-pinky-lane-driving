# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Synthetic laser scans and rigid transforms, in metres/radians."""

import math
import unittest

from pinky_lane_driving.obstacles import scan_collision, transform_points, swept_footprint_hit, convex_footprint, stopping_corridor, steering_corridor


class ObstacleTest(unittest.TestCase):
    def scan(self, ranges, **overrides):
        args = dict(angle_min=0., angle_increment=.1, range_min=.02, range_max=5.,
                    scan_pose=(0., 0., 0.), path=[(0., 0.), (1., 0.)],
                    radius=.15, age=.01, timeout=.2)
        args.update(overrides)
        return scan_collision(ranges, **args)

    def test_hit_clear_and_footprint_margin(self):
        self.assertTrue(self.scan([.5]))
        self.assertFalse(self.scan([2.]))
        self.assertTrue(self.scan([.5], scan_pose=(0., .14, 0.)))
        self.assertFalse(self.scan([.5], scan_pose=(0., .16, 0.)))

    def test_laser_extrinsic_rotation_and_curved_path(self):
        self.assertFalse(self.scan([.5], scan_pose=(0., 0., math.pi / 2)))
        self.assertTrue(self.scan([.5], scan_pose=(.5, 0., math.pi / 2),
                                  path=[(0., 0.), (.5, 0.), (.5, 1.)]))

    def test_unknown_scan_is_never_clear(self):
        for ranges, overrides in [([], {}), ([math.nan], {}), ([-math.inf], {}),
                                   ([.01], {}), ([6.], {}), ([2.], {'age': .3}),
                                   ([2.], {'age': -.1}), ([2.], {'path': []}),
                                   ([2.], {'scan_pose': None}),
                                   ([2.], {'range_max': .5})]:
            with self.subTest(ranges=ranges, overrides=overrides):
                self.assertIsNone(self.scan(ranges, **overrides))

    def test_infinite_no_return_requires_explicit_driver_contract(self):
        self.assertIsNone(self.scan([math.inf]))
        self.assertFalse(self.scan([math.inf], infinity_is_clear=True))

    def test_rigid_transform_for_current_frame_reuse(self):
        point = transform_points([(1., 0.)], (2., 3., math.pi / 2))[0]
        self.assertAlmostEqual(point[0], 2.)
        self.assertAlmostEqual(point[1], 4.)
        with self.assertRaises(ValueError):
            transform_points([(math.nan, 0.)], (0., 0., 0.))

    def body(self):
        return convex_footprint([(-.08,-.06),(.06,-.06),(.06,.06),(-.08,.06)])

    def test_polygon_keeps_clear_side_wall_but_stops_front(self):
        self.assertFalse(self.scan([.107], angle_min=-math.pi/2,
                                   footprint=self.body(), padding=.01))
        self.assertTrue(self.scan([.2], footprint=self.body(), padding=.01))
        self.assertTrue(self.scan([.065], angle_min=-math.pi/2,
                                  footprint=self.body(), padding=.01))

    def test_polygon_includes_rear_body_and_initial_heading(self):
        self.assertTrue(swept_footprint_hit([(-.075,0.)], [(0.,0.),(.3,0.)], self.body(), .01))
        self.assertTrue(swept_footprint_hit([(.05,-.05)], [(0.,0.),(0.,.3)], self.body(), .01))

    def test_corner_rotation_is_conservative(self):
        body=convex_footprint([(-.1,-.01),(.1,-.01),(.1,.01),(-.1,.01)])
        p=(.1*math.cos(math.pi/4),.1*math.sin(math.pi/4))
        self.assertTrue(swept_footprint_hit([p],[(0.,0.),(0.,.4)],body,0.))

    def test_self_filter_only_excludes_confirmed_small_body_region(self):
        kwargs=dict(footprint=self.body(), padding=.01,
                    self_filter_bounds=(.035,.052,-.04,.04), self_filter_pose=(0.,0.,0.))
        self.assertFalse(self.scan([.044], **kwargs))
        self.assertTrue(self.scan([.07], **kwargs))
        self.assertTrue(self.scan([.2], **kwargs))
        self.assertIsNone(self.scan([math.nan], **kwargs))
        self.assertIsNone(self.scan([.044], **dict(kwargs, age=.5)))
        self.assertIsNone(self.scan([.044], **dict(kwargs, self_filter_bounds=(0.,.2,-.04,.04))))

    def test_self_filter_uses_capture_frame_not_motion_compensation(self):
        kwargs=dict(footprint=self.body(), padding=.01,
                    self_filter_bounds=(.035,.052,-.04,.04), self_filter_pose=(0.,0.,0.))
        # An external point that moves inside the mask in the current frame must
        # not be mistaken for a body return in the original scan-time frame.
        self.assertTrue(self.scan([.1], scan_pose=(-.056,0.,0.), **kwargs))
        self.assertFalse(self.scan([.044], scan_pose=(.1,0.,0.), **kwargs))

    def test_confirmed_body_returns_vary_within_front_assembly(self):
        kwargs=dict(footprint=self.body(),padding=.01,
                    self_filter_bounds=(.025,.055,-.04,.04),self_filter_pose=(-.017,0.,math.pi))
        for distance in (.054,.067,.070):
            # Front-right assembly at the observed ~24 degree laser direction.
            self.assertFalse(self.scan([distance],angle_min=math.pi-math.radians(24),
                                       scan_pose=(-.017,0.,math.pi),**kwargs))
        self.assertTrue(self.scan([.12],angle_min=math.pi-math.radians(24),
                                  scan_pose=(-.017,0.,math.pi),**kwargs))

    def test_explicit_nine_cm_radius_uses_robot_center_not_laser_center(self):
        kwargs=dict(footprint=self.body(),padding=.01,self_filter_radius=.09,
                    self_filter_pose=(-.017,0.,math.pi),scan_pose=(-.017,0.,math.pi))
        # Range .1 from the offset laser is robot-forward x=.083: masked.
        self.assertFalse(self.scan([.1],angle_min=math.pi,**kwargs))
        self.assertTrue(self.scan([.108],angle_min=math.pi,**kwargs))
        self.assertFalse(self.scan([.106],angle_min=math.pi,**kwargs))

    def test_mask_excludes_spatial_near_body_returns_not_unknown_sensor_data(self):
        kwargs=dict(footprint=self.body(),padding=.01,self_filter_radius=.09,
                    self_filter_pose=(0.,0.,0.))
        self.assertFalse(self.scan([.01],**kwargs))
        for value in (math.nan,-math.inf,0.,-.01):
            self.assertIsNone(self.scan([value],**kwargs))
        self.assertIsNone(self.scan([.01],**dict(kwargs,self_filter_pose=(.2,0.,0.))))
        self.assertIsNone(self.scan([.01],**dict(kwargs,self_filter_radius=.3)))
        self.assertIsNone(self.scan([.01],**dict(kwargs,age=.5)))

    def test_invalid_polygon_is_unknown(self):
        for polygon in ([(0.,0.),(1.,0.)],[(1.,1.),(2.,1.),(2.,2.)],
                        [(-.1,-.1),(.1,-.1),(0.,0.),(.1,.1),(-.1,.1)]):
            self.assertIsNone(self.scan([.5], footprint=polygon))

    def corridor(self, path, **overrides):
        args=dict(max_speed=.08,measured_speed=0.,decel=.15,latency=.2,
                  scan_timeout=.3,observation_timeout=.3,timer_period=.05,stop_margin=.06)
        args.update(overrides)
        return stopping_corridor(path,**args)

    def test_stopping_horizon_uses_maximum_speed_even_at_rest(self):
        path,horizon=self.corridor([(.2,0.),(.4,0.)])
        self.assertAlmostEqual(horizon,.08*.85+.08**2/.3+.06)
        self.assertAlmostEqual(path[-1][0],horizon)
        self.assertTrue(self.scan([.15],path=path,footprint=self.body(),padding=.01))
        self.assertFalse(self.scan([.4],path=path,footprint=self.body(),padding=.01))
        self.assertIsNone(self.scan([math.nan],path=path,footprint=self.body(),padding=.01))

    def test_speed_above_limit_or_more_latency_expands_horizon(self):
        _,h1=self.corridor([(.2,0.),(.8,0.)])
        _,h2=self.corridor([(.2,0.),(.8,0.)],measured_speed=.2)
        _,h3=self.corridor([(.2,0.),(.8,0.)],scan_timeout=.6)
        self.assertGreater(h2,h1)
        self.assertGreater(h3,h1)

    def test_stopping_horizon_preserves_near_end_and_curves(self):
        near=((.03,0.),(.06,0.))
        path,_=self.corridor(near)
        self.assertEqual(path,((0.,0.),)+near)
        path,horizon=self.corridor([(.04,0.),(.04,.4)])
        self.assertAlmostEqual(path[-1][0],.04)
        self.assertAlmostEqual(path[-1][1],horizon-.04)
        with self.assertRaises(ValueError):
            self.corridor([(.2,0.),(.4,0.)],decel=0.)

    def test_command_arc_checks_off_lane_obstacles_and_discretization(self):
        path=((0.,0.),(.2,0.))
        arc,guard=steering_corridor(.2,5.,.1)
        self.assertGreater(guard,0.)
        point=arc[-1]
        angle=math.atan2(point[1],point[0])
        ranges=[math.hypot(*point)]
        self.assertFalse(self.scan(ranges,angle_min=angle,path=path,radius=.01))
        self.assertTrue(self.scan(ranges,angle_min=angle,path=path,radius=.01,
                                  steering_path=arc,steering_guard=guard))

    def test_steering_arc_symmetry_and_straight(self):
        left,guard=steering_corridor(.2,2.,.1)
        right,right_guard=steering_corridor(.2,-2.,.1)
        self.assertEqual(guard,right_guard)
        self.assertAlmostEqual(left[-1][0],right[-1][0])
        self.assertAlmostEqual(left[-1][1],-right[-1][1])
        straight,guard=steering_corridor(.2,0.,.1)
        self.assertEqual(straight[-1],(.2,0.))
        self.assertEqual(guard,0.)

    def test_side_wall_off_command_arc_preserves_front_and_turn_obstacles(self):
        body = ((-.08, -.06), (.06, -.06), (.06, .06), (-.08, .06))
        horizon = .1125
        for sign in (-1., 1.):
            arc, guard = steering_corridor(horizon, sign * 2.873239, .1)
            wall = (.153, -sign * .083)
            def hit(point, path):
                return self.scan([math.hypot(*point)],
                    angle_min=math.atan2(point[1], point[0]),
                    path=path, footprint=body, radius=.11,
                    steering_path=arc, steering_guard=guard)
            diagonal = ((0., 0.), (.109629, -sign * .025255))
            forward = ((0., 0.), (horizon, 0.))
            self.assertTrue(hit(wall, diagonal))
            self.assertFalse(hit(wall, forward))
            for obstacle in ((.15, 0.), (.14, sign * .07), (.04, -sign * .04)):
                self.assertTrue(hit(obstacle, forward))

    def test_straight_reaction_corridor_ignores_returns_outside_the_lane(self):
        body = ((-.08, -.06), (.06, -.06), (.06, .06), (-.08, .06))
        horizon = .1125
        arc, guard = steering_corridor(horizon, 2.873239, .1)
        forward = ((0., 0.), (horizon, 0.))
        lane = ((.1, .06), (.2, .15), (.3, .26))   # lane bends left, away from the heading
        def hit(point, **extra):
            return self.scan([math.hypot(*point)], angle_min=math.atan2(point[1], point[0]),
                             path=forward, footprint=body, radius=.11,
                             steering_path=arc, steering_guard=guard, **extra)
        beside = (.14, -.058)       # inside the heading-straight body sweep, outside the lane
        self.assertTrue(hit(beside))
        self.assertFalse(hit(beside, lane_path=lane, lane_half_width=.085))
        self.assertTrue(hit(beside, lane_path=lane, lane_half_width=.15))    # wider lane keeps it
        self.assertTrue(hit((.14, .02), lane_path=lane, lane_half_width=.085))  # inside the lane
        self.assertTrue(hit(beside, lane_path=lane[:1], lane_half_width=.085))  # unusable lane
        self.assertTrue(hit(beside, lane_path=lane, lane_half_width=0.))
        self.assertTrue(hit(beside, lane_path=lane, lane_half_width=math.nan))

    def test_lane_filter_never_hides_the_commanded_arc_or_planned_path(self):
        body = ((-.08, -.06), (.06, -.06), (.06, .06), (-.08, .06))
        arc, guard = steering_corridor(.1125, 2.873239, .1)
        on_arc = arc[-1]
        lane = ((.3, 0.), (.4, 0.))   # lane far from the arc: filter would drop the point
        self.assertTrue(self.scan([math.hypot(*on_arc)], angle_min=math.atan2(on_arc[1], on_arc[0]),
                                  path=((0., 0.), (.1125, 0.)), footprint=body, radius=.11,
                                  steering_path=arc, steering_guard=guard,
                                  lane_path=lane, lane_half_width=.085))
        planned = ((0., 0.), (.5, 0.))
        self.assertTrue(self.scan([.3], angle_min=0., path=planned, footprint=body, radius=.11,
                                  lane_path=((.1, .3), (.2, .4)), lane_half_width=.085))
