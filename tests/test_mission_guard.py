import unittest
from pinky_lane_driving.mission_guard import MissionGuard


class MissionGuardTest(unittest.TestCase):
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
        g.note('velocity',(.05,0.),8.)
        self.assertEqual(g.fault(8.),'velocity_limit_exceeded')
