"""Leader references must stop at the latest pose without a filter tail."""
import math
import unittest

import numpy as np

from rebotarmcontroller.motion_profiles import advance_velocity_limited_reference


class LeaderMotionTests(unittest.TestCase):
    def test_reached_pose_stops_on_next_tick(self):
        q, v, a = advance_velocity_limited_reference(
            np.array([0.005, -0.004]), np.array([-0.8, 0.8]),
            np.zeros(2), np.ones(2), 0.008,
        )
        np.testing.assert_array_equal(q, [0, 0])
        q, v, a = advance_velocity_limited_reference(q, v, np.zeros(2), np.ones(2), 0.008)
        np.testing.assert_array_equal(q, [0, 0])
        np.testing.assert_array_equal(v, [0, 0])

    def test_large_step_and_reversal_obey_selected_speed(self):
        q, v = np.zeros(2), np.zeros(2)
        limits = np.array([0.3, 1.0])
        for target in ([1, -1], [-1, 1], [0, 0]):
            previous = q.copy()
            q, v, _ = advance_velocity_limited_reference(q, v, np.array(target), limits, 0.008)
            self.assertTrue(np.all(np.abs(q - previous) <= limits * 0.008 + 1e-12))
            self.assertTrue(np.all(np.abs(v) <= limits + 1e-12))

    def test_slow_callback_cannot_cause_unbounded_jump(self):
        q, v, _ = advance_velocity_limited_reference(
            np.zeros(1), np.zeros(1), np.ones(1), np.array([0.3]), 2.0,
        )
        np.testing.assert_allclose(q, [0.03])
        np.testing.assert_allclose(v, [0.3])

    def test_60hz_leader_return_has_no_second_order_settling_tail(self):
        q, v = np.array([0.5]), np.zeros(1)
        final_sample_at = math.ceil((0.5 / 0.8) * 60) / 60
        for tick in range(125):
            at = tick / 125
            sampled_at = math.floor(at * 60) / 60
            goal = np.array([max(0.0, 0.5 - 0.8 * sampled_at)])
            previous = q.copy()
            q, v, _ = advance_velocity_limited_reference(q, v, goal, np.ones(1), 1 / 125)
            self.assertGreaterEqual(q[0], 0.0)
            self.assertLessEqual(abs(q[0] - previous[0]), 1 / 125 + 1e-12)
            if at >= final_sample_at + 1 / 125:
                self.assertEqual(q[0], 0.0)
                self.assertEqual(v[0], 0.0)


if __name__ == '__main__':
    unittest.main()
