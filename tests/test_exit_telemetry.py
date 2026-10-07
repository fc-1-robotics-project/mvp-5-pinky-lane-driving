"""Exit evidence never treats invalid perception or odometry as stillness."""
import math
import unittest

from pinky_lane_driving.exit_telemetry import boundary_visibility, OdomStationarity
from pinky_lane_driving.perception import observe
import test_mission_guard


class ExitTelemetryTest(unittest.TestCase):
    def test_boundary_absence_requires_valid_detection_contract(self):
        frame = observe(1., 'camera', (640, 480), [], [], [])
        self.assertEqual(boundary_visibility(frame), dict(left_visible=False, right_visible=False))
        frame = observe(1., 'camera', (640, 480), [1], [.8], [[(0., 0.), (5., 0.), (5., 20.)]])
        self.assertEqual(boundary_visibility(frame), dict(left_visible=True, right_visible=False))
        frame['detections'][0]['class_id'] = 7
        with self.assertRaises(ValueError):
            boundary_visibility(frame)
        with self.assertRaises(KeyError):
            boundary_visibility({'capture_time_s': 1.})

    def test_actual_stop_accumulates_but_motion_rotation_and_staleness_reset(self):
        tracker = OdomStationarity()
        for index in range(41):
            now = index * .1
            tracker.update((0., 0., 0.), (0., 0., 0.), now, now, 0.)
        self.assertAlmostEqual(tracker.duration(4.), 4.)
        tracker.update((.004, 0., 0.), (0., 0., 0.), 4.1, 4.1, 0.)
        self.assertEqual(tracker.duration(4.1), 0.)
        tracker.update((.004, 0., .03), (0., 0., 0.), 4.2, 4.2, 0.)
        self.assertEqual(tracker.duration(4.2), 0.)
        tracker.update((.004, 0., .03), (.02, 0., 0.), 4.3, 4.3, 0.)
        self.assertIsNone(tracker.duration(4.3))
        tracker.update((.004, 0., .03), (0., 0., 0.), 4.4, 4.4, 0.)
        self.assertIsNone(tracker.duration(4.8))

    def test_replayed_invalid_odom_and_spin_are_not_stationary(self):
        tracker = OdomStationarity()
        for index in range(40):
            tracker.update((0., 0., 0.), (0., 0., 0.), 1., index * .1, 0.)
        self.assertEqual(tracker.duration(39 * .1), 0.)
        tracker.update((0., 0., 0.), (0., 0., .1), 2., 4., 0.)
        self.assertIsNone(tracker.duration(4.))
        tracker.update((math.nan, 0., 0.), (0., 0., 0.), 2.1, 4.1, 0.)
        self.assertIsNone(tracker.duration(4.1))
        tracker.update((0., 0., 0.), (0., 0., 0.), 2.2, 4.2, .4)
        self.assertIsNone(tracker.duration(4.2))

    def test_motion_to_stop_starts_timer_without_three_mm_position_change(self):
        tracker = OdomStationarity()
        for moving_velocity in ((.09, 0., 0.), (0., 0., .1)):
            tracker.update((0., 0., 0.), moving_velocity, 10., 10., 0.)
            self.assertIsNone(tracker.duration(10.))
            for index in range(1, 82):
                now = 10. + index * .05
                tracker.update((.001, 0., 0.), (0., 0., 0.), now, now, 0.)
            self.assertAlmostEqual(tracker.duration(now), 4.)

    def test_valid_empty_frame_uses_lane_loss_timer_not_camera_fault_timer(self):
        helper = test_mission_guard.MissionGuardTest()
        guard = helper.ready()
        guard.values['diagnostic'].update(capture_age_s=None, observation_age_s=.1,
                                         lane_valid=False, lane_reason='no_current_lane')
        for index in range(41):
            now = 1. + .1 * index
            self.assertEqual(helper.active_sample(guard, now, (0., 0.)), '')
        guard.values['diagnostic']['observation_age_s'] = None
        self.assertEqual(guard.sensor_problem(now), 'observation_age_s_stale')
