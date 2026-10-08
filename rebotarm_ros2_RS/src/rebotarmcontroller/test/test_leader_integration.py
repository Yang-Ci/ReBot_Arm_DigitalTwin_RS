"""Runs real subprocess/host code with lightweight ROS transport stubs on any OS."""
import importlib.util
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from rebotarmcontroller.fake_leader_hardware import FakeLeaderHardware
from rebotarmcontroller.leader_policy import DEFAULT_LIMITS, TeleopLease


class Message:
    def __init__(self):
        self.devices = []


def import_with_stubs(name, filename, stubs):
    spec = importlib.util.spec_from_file_location(name, filename)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


PACKAGE = Path(__file__).resolve().parents[1] / 'rebotarmcontroller'
host = import_with_stubs('rebotarmcontroller._test_leader_host', PACKAGE / 'leader_teleop.py', {
    'rclpy.callback_groups': SimpleNamespace(ReentrantCallbackGroup=object),
    'rebotarm_msgs.msg': SimpleNamespace(LeaderDevice=Message, LeaderStatus=Message),
    'rebotarm_msgs.srv': SimpleNamespace(LeaderControl=object),
})
driver = import_with_stubs('rebotarmcontroller._test_leader_driver', PACKAGE / 'hardware_manager.py', {
    'rebotarmcontroller.conversions': SimpleNamespace(fk_to_pose=lambda *a: None),
    'rebotarmcontroller.hardware_config': SimpleNamespace(resolve_hardware_config=lambda *a: None),
})


class FakeNode:
    def __init__(self):
        self._leader_lock = threading.RLock()
        self.positions, self.targets = [0.1, 0.4, 0.5, 0, 0, 0], [0] * 6
        self.velocities = [0] * 6
        self.gripper_position, self.gripper_velocity, self.gripper_target = 1.0, 0, 1.0
        self.gripper_open_position = 5.0
        self.enabled, self.state_machine = True, 'IDLE'
        self.params = {}
        self.messages = []

    def declare_parameter(self, name, value): self.params[name] = value
    def get_parameter(self, name): return SimpleNamespace(value=self.params[name])
    def create_publisher(self, *a, **k): return SimpleNamespace(publish=self.messages.append)
    def create_service(self, *a, **k): return object()
    def create_timer(self, *a, **k): return object()
    def get_clock(self): return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object()))


class LeaderIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.node = FakeNode()
        self.hw = FakeLeaderHardware(self.node)
        self.leader = host.LeaderTeleop(self.node, self.hw, 'test', allow_mock=True)

    def tearDown(self):
        self.leader.shutdown()

    def request(self, operation, **kwargs):
        args = dict(operation=operation, port='mock', session_id=self.leader.session,
                    speed=0.3, follow_gripper=True, absolute=False, confirm_zero=True)
        args.update(kwargs)
        return self.leader.control(SimpleNamespace(**args), Message())

    def wait_sample(self):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not self.leader.sample_at:
            time.sleep(0.01)
        self.assertTrue(self.leader.sample_at)

    def test_mock_worker_connect_calibrate_follow_pause_resume_stop(self):
        self.assertTrue(self.request('connect').success)
        self.wait_sample()
        self.assertEqual(self.leader.state, 'PREVIEW')
        self.assertFalse(self.request('start').success)
        self.assertTrue(self.request('unlock').success)
        self.wait_sample()
        self.assertFalse(self.leader.calibrated)
        self.assertFalse(self.request('calibrate', confirm_zero=False).success)
        self.assertTrue(self.request('calibrate').success)
        self.wait_sample()
        self.assertTrue(self.request('start').success)
        self.assertTrue(self.hw.teleop_owned)
        self.assertFalse(self.request('unlock').success)
        time.sleep(0.1)
        self.assertNotEqual(self.node.targets, self.node.positions)
        self.assertFalse(self.request('stop', session_id='another-page').success)
        self.assertTrue(self.request('pause').success)
        self.assertEqual(self.node.targets, self.node.positions)
        self.assertTrue(self.hw.teleop_owned)
        self.assertTrue(self.request('heartbeat').success)
        self.assertTrue(self.request('resume').success)
        self.assertTrue(self.request('stop').success)
        self.assertFalse(self.hw.teleop_owned)
        self.assertEqual(self.node.targets, self.node.positions)
        self.assertTrue(self.request('disconnect').success)

    def test_worker_loss_ends_follow_and_requires_recalibration(self):
        self.assertTrue(self.request('connect').success)
        self.assertTrue(self.request('calibrate').success)
        self.wait_sample()
        self.assertTrue(self.request('start').success)
        self.leader.worker.process.kill()
        time.sleep(0.15)
        self.leader.tick()
        self.assertFalse(self.hw.teleop_owned)
        self.assertFalse(self.leader.calibrated)
        self.assertEqual(self.leader.state, 'FAULT')
        self.assertFalse(self.request('resume').success)

    def test_paused_session_browser_loss_is_held_and_released(self):
        self.assertTrue(self.request('connect').success)
        self.assertTrue(self.request('calibrate').success)
        self.wait_sample()
        self.assertTrue(self.request('start').success)
        self.assertTrue(self.request('pause').success)
        self.hw._teleop_lease.last_heartbeat -= 3.1
        self.leader.tick()
        self.assertFalse(self.hw.teleop_owned)
        self.assertEqual(self.node.targets, self.node.positions)
        self.assertFalse(self.request('heartbeat').success)

    def test_absolute_mismatch_and_invalid_speed_rejected(self):
        self.assertTrue(self.request('connect').success)
        self.assertTrue(self.request('calibrate').success)
        self.wait_sample()
        self.assertFalse(self.request('start', absolute=True).success)
        self.assertFalse(self.request('start', speed=float('nan')).success)
        self.assertFalse(self.hw.teleop_owned)

    def test_fake_follower_accepts_simulation_speed_limit(self):
        self.assertTrue(self.request('connect').success)
        self.assertTrue(self.request('calibrate').success)
        self.wait_sample()
        self.assertFalse(self.request('start', speed=1.01).success)
        self.assertFalse(self.hw.teleop_owned)
        self.assertTrue(self.request('start', speed=1.0).success)
        self.assertEqual(self.node.max_joint_speed, 1.0)
        self.assertTrue(self.request('stop').success)

    def test_fake_gripper_preserves_slow_speed_scaling_and_caps_fast_follow(self):
        token = self.hw.teleop_acquire()
        for seq, (speed, expected) in enumerate([(0.1, 0.6), (0.3, 1.8), (1.0, 3.0)]):
            self.hw.teleop_send(token, seq, time.monotonic(), self.node.positions, 5.0, speed)
            self.assertAlmostEqual(self.node.max_gripper_speed, expected)

    def test_stale_queued_uart_frames_are_ignored_until_watchdog(self):
        self.assertTrue(self.request('connect').success)
        self.assertTrue(self.request('calibrate').success)
        self.wait_sample()
        self.assertTrue(self.request('start').success)
        self.assertEqual(self.leader.state, 'FOLLOWING')
        current_seq = self.leader.sample_seq
        # Inject a stale queued frame older than the max age threshold.
        stale = {
            'event': 'sample',
            'at': time.monotonic() - 1.1,
            'angles': list(self.leader.angles) if self.leader.angles else [0.0] * 7,
            'seq': current_seq + 1000,
        }
        self.leader.on_frame(self.leader.worker, stale)
        self.assertEqual(self.leader.state, 'FOLLOWING')
        self.assertEqual(self.leader.sample_seq, current_seq)
        # A subsequent fresh frame should still update state and keep following alive.
        fresh = {
            'event': 'sample',
            'at': time.monotonic(),
            'angles': list(self.leader.angles) if self.leader.angles else [0.0] * 7,
            'seq': current_seq + 1001,
        }
        self.leader.on_frame(self.leader.worker, fresh)
        self.assertEqual(self.leader.state, 'FOLLOWING')
        self.assertEqual(self.leader.sample_seq, current_seq + 1001)
        self.assertTrue(self.request('stop').success)

    def test_mock_scan_and_hardware_mock_rejection(self):
        result = self.request('scan')
        self.assertTrue(result.success)
        self.assertEqual(len(result.devices), 1)
        self.assertEqual(result.devices[0].responding_ids, list(range(7)))
        self.leader.allow_mock = False
        self.assertFalse(self.request('scan').success)
        self.assertFalse(self.request('connect').success)

    def test_expired_session_cannot_be_revived_by_pause(self):
        token = self.hw.teleop_acquire()
        self.hw._teleop_lease.last_sample -= 1.1
        with self.assertRaises(RuntimeError):
            self.hw.teleop_pause(token)
        self.assertFalse(self.hw.teleop_owned)

    def test_real_driver_complete_frame_and_rejection_are_atomic(self):
        hw = driver.HardwareManager.__new__(driver.HardwareManager)
        hw._cmd_lock = threading.RLock()
        hw._teleop_lease = TeleopLease()
        token = hw._teleop_lease.acquire()
        hw._state_machine = 'LOWLEVEL_STREAMING'
        hw._robot = SimpleNamespace(has_gripper=True)
        hw._feedback_refreshed_at = time.monotonic()
        hw.teleop_limits = DEFAULT_LIMITS
        hw.gripper_close_position, hw.gripper_open_position = 0, 5
        hw._mit_stream_vlim = np.ones(6)
        hw._leader_gripper_velocity_limit = 3.0
        targets = [0.1, 0.3, 0.4, 0.1, 0.1, 0.2]
        hw.teleop_send(token, 1, time.monotonic(), targets, 1.5, 0.1)
        np.testing.assert_equal(hw._mit_stream_target, targets)
        np.testing.assert_equal(hw._mit_stream_vlim, [0.1]*6)
        self.assertEqual(hw._mit_gripper_target, 1.5)
        self.assertAlmostEqual(hw._mit_gripper_vlim, 0.6)
        hw.teleop_send(token, 2, time.monotonic(), targets, 1.5, 1.0)
        np.testing.assert_equal(hw._mit_stream_vlim, [1.0]*6)
        self.assertEqual(hw._mit_gripper_vlim, 3.0)
        with self.assertRaises(ValueError):
            hw.teleop_send(token, 3, time.monotonic(), targets, 1.5, 1.01)
        np.testing.assert_equal(hw._mit_stream_vlim, [1.0]*6)
        for wrong in ([0.0]*5, [0, 0, 0, 9, 0, 0], [float('nan')]*6):
            with self.assertRaises(ValueError):
                hw.teleop_send(token, 2, time.monotonic(), wrong, 1, 0.1)
            np.testing.assert_equal(hw._mit_stream_target, targets)
            self.assertEqual(hw._mit_gripper_target, 1.5)

    def test_real_sender_uses_latest_leader_target_with_speed_limit(self):
        hw = driver.HardwareManager.__new__(driver.HardwareManager)
        hw._cmd_lock = threading.RLock()
        hw._teleop_lease = TeleopLease()
        hw._teleop_lease.acquire()
        hw._state_machine = 'LOWLEVEL_STREAMING'
        hw._robot = SimpleNamespace(has_gripper=False)
        hw._control_output_enabled = True
        hw._arm_control_mode = 'mit'
        hw._mit_stream_last_time = 10.0
        hw._mit_stream_target = np.array([0.0, 1.0])
        hw._mit_stream_vlim = np.array([1.0, 0.3])
        hw._mit_stream_velocity = np.array([-0.8, 0.0])
        hw._mit_stream_acceleration = np.zeros(2)
        hw._mit_gripper_target = None
        hw._cached_arm_position = np.array([0.005, 0.0])
        hw._endpos_ctrl = SimpleNamespace(_q_target=hw._cached_arm_position.copy(),
                                          _qd_target=np.zeros(2))
        hw._leader_mit_kp = np.array([50.0, 50.0])
        hw._leader_mit_kd = np.array([3.0, 5.0])
        sent = []
        hw._arm_group = SimpleNamespace(
            _mit_kp=np.ones(2), _mit_kd=np.ones(2),
            send_mit=lambda q, **kwargs: sent.append((q.copy(), {
                key: value.copy() for key, value in kwargs.items()
            })),
        )
        hw._gravity_comp_torque = lambda q: np.array([0.2, 1.0])
        with patch.object(driver.time, 'perf_counter', return_value=10.008):
            hw._endpos_loop_cb(hw._robot, 0.008)
        self.assertEqual(len(sent), 1)
        np.testing.assert_allclose(sent[0][0], [0.0, 0.0024], atol=1e-12)
        np.testing.assert_array_equal(sent[0][1]['vel'], [0.0, 0.0])
        np.testing.assert_array_equal(sent[0][1]['kp'], hw._leader_mit_kp)
        np.testing.assert_array_equal(sent[0][1]['kd'], hw._leader_mit_kd)
        np.testing.assert_array_equal(sent[0][1]['tau'], [0.2, 1.0])
        self.assertAlmostEqual(hw._endpos_ctrl._qd_target[1], 0.3)

        # Fast gripper opening advances at the new cap, then lands on the
        # latest target without overshooting. Its MIT gains stay unchanged.
        hw._robot.has_gripper = True
        hw._mit_gripper_target = 5.0
        hw._mit_gripper_vlim = 3.0
        hw._endpos_ctrl._gripper_target = 1.0
        hw._gripper_manual_free = hw._gripper_assist_active = False
        hw._gripper_mit_kp, hw._gripper_mit_kd = np.array([12.0]), np.array([0.05])
        gripper_sent = []
        hw._gripper_group = SimpleNamespace(
            send_mit=lambda q, **kwargs: gripper_sent.append((q.copy(), kwargs)),
        )
        with patch.object(driver.time, 'perf_counter', return_value=10.016):
            hw._endpos_loop_cb(hw._robot, 0.008)
        self.assertAlmostEqual(gripper_sent[-1][0][0], 1.024)
        np.testing.assert_array_equal(gripper_sent[-1][1]['kp'], [12.0])
        np.testing.assert_array_equal(gripper_sent[-1][1]['kd'], [0.05])
        hw._mit_gripper_target = 1.03
        with patch.object(driver.time, 'perf_counter', return_value=10.024):
            hw._endpos_loop_cb(hw._robot, 0.008)
        self.assertAlmostEqual(gripper_sent[-1][0][0], 1.03)
        hw._robot.has_gripper = False
        hw._mit_gripper_target = None

        # Once leader ownership ends, browser streams keep their existing
        # acceleration/jerk shaping instead of inheriting the leader path.
        hw._teleop_lease.release('test browser stream')
        hw._endpos_ctrl._q_target[:] = [0.005, 0.0]
        hw._mit_stream_velocity[:] = [-0.8, 0.0]
        hw._mit_stream_acceleration[:] = 0.0
        hw._mit_stream_last_time = 10.0
        hw._stream_acceleration_limit = 4.0
        hw._stream_jerk_limit = 30.0
        hw._stream_natural_frequency = 8.0
        with patch.object(driver.time, 'perf_counter', return_value=10.008):
            hw._endpos_loop_cb(hw._robot, 0.008)
        self.assertNotEqual(sent[-1][0][0], 0.0)
        self.assertLess(sent[-1][0][1], 0.0024)
        np.testing.assert_array_equal(sent[-1][1]['vel'], hw._endpos_ctrl._qd_target)
        np.testing.assert_array_equal(sent[-1][1]['kp'], [1.0, 1.0])
        np.testing.assert_array_equal(sent[-1][1]['kd'], [1.0, 1.0])

        # A paused lease still owns the arm, but uses regular hold gains.
        token = hw._teleop_lease.acquire()
        hw.teleop_pause(token)
        with patch.object(driver.time, 'perf_counter', return_value=10.016):
            hw._endpos_loop_cb(hw._robot, 0.008)
        np.testing.assert_array_equal(sent[-1][1]['kp'], [1.0, 1.0])
        np.testing.assert_array_equal(sent[-1][1]['kd'], [1.0, 1.0])
        np.testing.assert_array_equal(sent[-1][1]['vel'], [0.0, 0.0])

    def test_real_driver_watchdog_clears_all_stream_targets(self):
        hw = driver.HardwareManager.__new__(driver.HardwareManager)
        hw._cmd_lock = threading.RLock()
        hw._teleop_lease = TeleopLease()
        hw._teleop_lease.acquire()
        hw._teleop_lease.last_sample -= 1.1
        hw._state_machine = 'LOWLEVEL_STREAMING'
        hw._robot = SimpleNamespace(has_gripper=True)
        hw._mit_stream_target = np.ones(6)
        hw._lowlevel_pos_target = np.ones(6)
        hw._mit_stream_velocity = np.ones(6)
        hw._mit_stream_acceleration = np.ones(6)
        hw._mit_gripper_target, hw._gripper_target_position = 4, 4
        hw._cached_arm_position = np.arange(6) / 10
        hw._cached_gripper_position = 1.2
        hw._endpos_ctrl = SimpleNamespace(_q_target=np.ones(6), _qd_target=np.ones(6))
        self.assertEqual(hw.teleop_watchdog(), 'leader sample timed out')
        self.assertFalse(hw.teleop_owned)
        self.assertIsNone(hw._mit_stream_target)
        self.assertIsNone(hw._mit_gripper_target)
        np.testing.assert_equal(hw._endpos_ctrl._q_target, hw._cached_arm_position)
        np.testing.assert_equal(hw._endpos_ctrl._qd_target, np.zeros(6))
        self.assertEqual(hw._endpos_ctrl._gripper_target, 1.2)

    def test_real_driver_legacy_commands_rejected_before_io(self):
        hw = driver.HardwareManager.__new__(driver.HardwareManager)
        hw._cmd_lock = threading.RLock()
        hw._teleop_lease = TeleopLease()
        hw._teleop_lease.acquire()
        for action in (lambda: hw.send_joint_mit_cmd('joint1', 1, 1, 0, 0, 0),
                       lambda: hw.set_gripper_target(1), lambda: hw.begin_trajectory_stream(),
                       lambda: hw.safe_home(), lambda: hw.disable()):
            with self.assertRaises(RuntimeError): action()


if __name__ == '__main__':
    unittest.main()
