import math
from types import SimpleNamespace
import unittest

from rebotarmcontroller.leader_policy import LeaderMapping, TeleopLease, fresh_angles


class LeaderPolicyTests(unittest.TestCase):
    def test_wiki_direction_and_gripper_scale(self):
        mapping = LeaderMapping([0] * 7, [0] * 6, 0, absolute=True)
        mapping.map([0, 0, -20, 0, 0, 0, 20])
        targets, gripper = mapping.map([0, 0, -30, 0, 0, 0, 30])
        self.assertAlmostEqual(targets[2], math.pi / 6)
        self.assertAlmostEqual(gripper, math.pi)

    def test_relative_first_frame_preserves_follower_and_gripper(self):
        leader = [10, 20, -30, 5, 10, 0, 20]
        follower = [0.2, 0.5, 0.6, 0.1, 0.2, 0.3]
        mapping = LeaderMapping(leader, follower, 2.0)
        targets, gripper = mapping.map(leader)
        self.assertEqual(targets, follower)
        self.assertEqual(gripper, 2.0)
        moved = leader[:]
        moved[2] -= 5
        self.assertAlmostEqual(mapping.map(moved)[0][2], 0.6 + math.radians(5))

    def test_limits_nan_jump_and_missing_frames(self):
        mapping = LeaderMapping([0] * 7, [2.79, 0, 0, 0, 0, 0], 4.9)
        targets, grip = mapping.map([20, 0, 0, 0, 0, 0, 20])
        self.assertEqual(targets[0], 2.8)
        self.assertEqual(grip, 5.0)
        with self.assertRaises(ValueError):
            mapping.map([60, 0, 0, 0, 0, 0, 20])
        with self.assertRaises(ValueError):
            mapping.map([float('nan')] * 7)
        with self.assertRaises(ValueError):
            mapping.map([0] * 6)

    def test_sdk_monitor_angle_is_accepted(self):
        samples = {i: SimpleNamespace(angle_deg=i, reliable=True) for i in range(7)}
        self.assertEqual(fresh_angles(samples), list(range(7)))
        samples[3].reliable = False
        self.assertEqual(fresh_angles(samples), list(range(7)))
        del samples[3]
        with self.assertRaises(RuntimeError):
            fresh_angles(samples)

    def test_lease_rejects_wrong_owner_stale_and_replayed_frames(self):
        now = [10.0]
        lease = TeleopLease(lambda: now[0])
        session = lease.acquire()
        with self.assertRaises(RuntimeError):
            lease.acquire()
        with self.assertRaises(RuntimeError):
            lease.heartbeat('other')
        lease.sample(session, 1, 10.0)
        with self.assertRaises(RuntimeError):
            lease.sample(session, 1, 10.0)
        now[0] += 1.01
        with self.assertRaises(RuntimeError):
            lease.sample(session, 2, 10.0)
        now[0] += 0.1
        self.assertEqual(lease.expired(), 'leader sample timed out')

    def test_paused_lease_still_requires_browser_heartbeat(self):
        now = [1.0]
        lease = TeleopLease(lambda: now[0])
        session = lease.acquire()
        lease.paused = True
        now[0] += 0.5
        self.assertFalse(lease.expired())
        lease.heartbeat(session)
        now[0] += 3.01
        self.assertEqual(lease.expired(), 'browser heartbeat timed out')
        lease.release('timeout')
        with self.assertRaises(RuntimeError):
            lease.heartbeat(session)


if __name__ == '__main__':
    unittest.main()
