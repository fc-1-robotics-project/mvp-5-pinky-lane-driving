import unittest
from pinky_lane_driving.mission_guard import MissionGuard


class MissionGuardTest(unittest.TestCase):
    def active_sample(self, guard, now, velocity, pose=(0., 0.), reason='follow'):
        for key, value in dict(mode='LANE', gate=dict(permit_fresh=True, mode=1),
                               velocity=velocity, command=reason, estop=False,
                               pose=pose, diagnostic=guard.values['diagnostic']).items():
            guard.note(key, value, now)
        return guard.fault(now)

    def ready(self, now=1.):
        guard = MissionGuard()
        for key, value in dict(mode='STOP', velocity=(0.,0.), estop=True,
            gate=dict(permit_fresh=True, mode=0),
            diagnostic=dict(lane_valid=True, lane_reason='paired', scan_hit=False,
                            scan_coverage_ok=True, capture_age_s=.1, scan_age_s=.1, odom_age_s=.1)).items():
            guard.note(key, value, now)
        return guard

    def test_only_fresh_stationary_state_is_ready(self):
        g = self.ready()
        self.assertTrue(g.readiness(1.)[0])
        self.assertFalse(g.readiness(2.)[0])
        g.note('velocity', (.01,0.),1.)
        self.assertFalse(g.readiness(1.)[0])

    def test_lane_loss_and_obstacle_block_start(self):
        g=self.ready()
        g.values['diagnostic']['lane_valid']=False
        self.assertFalse(g.readiness(1.)[0])
        g.values['diagnostic']['lane_valid']=True
        g.values['diagnostic']['scan_hit']=True
        self.assertFalse(g.readiness(1.)[0])

    def test_disabled_obstacle_stop_is_visible_but_unknown_still_blocks(self):
        g = self.ready()
        g.values['diagnostic'].update(lidar_obstacle_stop_enabled=False,
                                     raw_scan_hit=True, scan_hit=False)
        ready, reason = g.readiness(1.)
        self.assertTrue(ready)
        self.assertIn('라이다 장애물 정지 OFF', reason)
        g.values['diagnostic']['scan_hit'] = None
        self.assertFalse(g.readiness(1.)[0])

    def test_hold_does_not_become_persistent_stop(self):
        g=self.ready()
        for now in range(1,30):
            g.note('gate',dict(permit_fresh=True,mode=0),now)
            g.note('velocity',(0.,0.),now)
            g.note('mode','LANE',now)
            self.assertEqual(g.fault(now),'')
        g.note('gate',dict(permit_fresh=False,mode=0),30.)
        self.assertEqual(g.fault(30.),'central_connection_lost')

    def test_live_velocity_and_odom_progress_are_supervised(self):
        g=self.ready()
        for now in range(1,9):
            for key,value in dict(mode='LANE',gate=dict(permit_fresh=True,mode=1),
                                  velocity=(.03,0.),command='follow',estop=False,
                                  pose=(0.,0.),diagnostic=g.values['diagnostic']).items():
                g.note(key,value,now)
            problem=g.fault(now)
        self.assertEqual(problem,'no_odom_progress')
        g.note('velocity',(.091,0.),8.)
        self.assertEqual(g.fault(8.),'velocity_limit_exceeded')

    def test_nine_cm_speed_and_over_limit_boundary(self):
        g = self.ready()
        self.assertEqual(self.active_sample(g, 1., (.09, .6)), '')
        self.assertEqual(self.active_sample(g, 1.1, (.091, 0.)),
                         'velocity_limit_exceeded')
        self.assertEqual(self.active_sample(g, 1.2, (.09, .606)),
                         'velocity_limit_exceeded')

    def test_obstacle_wait_does_not_expire_restart_motion_timer(self):
        g = self.ready()
        self.assertEqual(self.active_sample(g, 1., (.03, 0.)), '')
        for now in range(2, 13):
            self.assertEqual(self.active_sample(g, float(now), (0., 0.)), '')
        # Waiting for a lidar obstacle to clear is not a failed motion request.
        self.assertEqual(self.active_sample(g, 12.1, (.005, 0.)), '')
        self.assertEqual(self.active_sample(g, 12.15, (.01, 0.), (.004, 0.)), '')
        self.assertEqual(self.active_sample(g, 18.14, (.03, 0.), (.004, 0.)), '')
        self.assertEqual(self.active_sample(g, 18.16, (.03, 0.), (.004, 0.)),
                         'no_odom_progress')

    def test_zero_command_still_expires_persistent_stop_timer(self):
        g = self.ready()
        for now in range(1, 16):
            self.assertEqual(self.active_sample(g, float(now), (0., 0.)), '')
        self.assertEqual(self.active_sample(g, 16., (0., 0.)), 'persistent_stop')

    def test_healthy_waits_longer_than_stall_timeout_restart_in_same_run(self):
        for reason, hit, state in [('obstacle', True, 'FOLLOW'),
                                   ('clear_hold', False, 'FOLLOW'),
                                   ('crosswalk', False, 'WAIT')]:
            with self.subTest(reason=reason):
                g = self.ready()
                g.values['diagnostic'].update(reason=reason, scan_hit=hit,
                                               behavior_state=state, emergency=False)
                for now in range(1, 31):
                    self.assertEqual(self.active_sample(g, float(now), (0., 0.),
                                                        reason=reason), '')
                g.values['diagnostic'].update(reason='follow', scan_hit=False,
                                               behavior_state='FOLLOW')
                self.assertEqual(self.active_sample(g, 30.1, (.01, 0.)), '')
                self.assertEqual(self.active_sample(g, 30.2, (.03, 0.), (.004, 0.)), '')
                self.assertEqual(self.active_sample(g, 36.3, (.03, 0.), (.004, 0.)),
                                 'no_odom_progress')

    def test_wait_reason_does_not_hide_path_sensor_or_permission_fault(self):
        for mutation, expected in [({'lane_valid': False}, 'persistent_lane_loss'),
                                    ({'scan_coverage_ok': False}, 'scan_coverage_unknown')]:
            with self.subTest(mutation=mutation):
                g = self.ready()
                g.values['diagnostic'].update(reason='obstacle', scan_hit=True,
                                               emergency=False, **mutation)
                self.active_sample(g, 1., (0., 0.), reason='obstacle')
                self.assertEqual(self.active_sample(g, 16., (0., 0.), reason='obstacle'),
                                 expected)
        g = self.ready()
        g.values['diagnostic'].update(reason='obstacle', scan_hit=True, emergency=False)
        self.active_sample(g, 1., (0., 0.), reason='obstacle')
        g.note('estop', True, 1.)
        self.assertEqual(g.fault(1.), 'local_permission_lost')

    def test_wait_requires_consistent_diagnostic_evidence(self):
        for reason, updates in [('obstacle', {'scan_hit': False}),
                                 ('crosswalk', {'behavior_state': 'PASS'}),
                                 ('clear_hold', {'scan_hit': True}),
                                 ('obstacle', {'reason': 'sensor_failure'})]:
            with self.subTest(reason=reason, updates=updates):
                g = self.ready()
                g.values['diagnostic'].update(reason=reason, scan_hit=True,
                                               emergency=False, behavior_state='WAIT')
                g.values['diagnostic'].update(updates)
                self.active_sample(g, 1., (0., 0.), reason=reason)
                self.assertEqual(self.active_sample(g, 16., (0., 0.), reason=reason),
                                 'persistent_stop')

    def test_wait_does_not_mask_motion_stall_or_odometry_failure(self):
        g = self.ready()
        g.values['diagnostic'].update(reason='obstacle', scan_hit=True, emergency=False)
        self.active_sample(g, 1., (.03, 0.), reason='obstacle')
        self.assertEqual(self.active_sample(g, 7.1, (.03, 0.), reason='obstacle'),
                         'no_odom_progress')
        self.assertEqual(self.active_sample(g, 7.2, (0., 0.), (float('nan'), 0.),
                                            reason='obstacle'), 'invalid_odom')
