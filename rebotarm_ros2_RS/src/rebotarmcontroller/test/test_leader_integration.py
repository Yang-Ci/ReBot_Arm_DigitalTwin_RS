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
        self.hw._teleop_lease.last_heartbeat -= 2
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
        self.hw._teleop_lease.last_sample -= 1
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
        targets = [0.1, 0.3, 0.4, 0.1, 0.1, 0.2]
        hw.teleop_send(token, 1, time.monotonic(), targets, 1.5, 0.1)
        np.testing.assert_equal(hw._mit_stream_target, targets)
        np.testing.assert_equal(hw._mit_stream_vlim, [0.1]*6)
        self.assertEqual(hw._mit_gripper_target, 1.5)
        for wrong in ([0.0]*5, [0, 0, 0, 9, 0, 0], [float('nan')]*6):
            with self.assertRaises(ValueError):
                hw.teleop_send(token, 2, time.monotonic(), wrong, 1, 0.1)
            np.testing.assert_equal(hw._mit_stream_target, targets)
            self.assertEqual(hw._mit_gripper_target, 1.5)

    def test_real_driver_watchdog_clears_all_stream_targets(self):
        hw = driver.HardwareManager.__new__(driver.HardwareManager)
        hw._cmd_lock = threading.RLock()
        hw._teleop_lease = TeleopLease()
        hw._teleop_lease.acquire()
        hw._teleop_lease.last_sample -= 1
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
