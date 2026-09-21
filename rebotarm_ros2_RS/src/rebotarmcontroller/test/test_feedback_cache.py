import threading
from types import SimpleNamespace

import numpy as np

from rebotarmcontroller.hardware_manager import HardwareManager


def _manager_with_cache():
    manager = HardwareManager.__new__(HardwareManager)
    manager._arm_group = SimpleNamespace(joint_names=["joint1", "joint2"])
    manager._robot = SimpleNamespace(has_gripper=True)
    manager._cached_arm_position = np.array([1.0, 2.0])
    manager._cached_arm_velocity = np.array([3.0, 4.0])
    manager._cached_arm_torque = np.array([5.0, 6.0])
    manager._cached_gripper_position = 0.0
    manager._cached_gripper_velocity = 0.0
    manager._cached_gripper_torque = 0.0
    return manager


def test_cached_joint_state_does_not_touch_hardware():
    manager = _manager_with_cache()

    def fail_if_called(*args, **kwargs):
        raise AssertionError("cached read must not poll hardware")

    manager._robot_get_state = fail_if_called
    pos, vel, torque = manager._get_arm_state(request_feedback=False)

    np.testing.assert_allclose(pos, [1.0, 2.0])
    np.testing.assert_allclose(vel, [3.0, 4.0])
    np.testing.assert_allclose(torque, [5.0, 6.0])

    # Callers receive snapshots, not mutable references to the real-time cache.
    pos[0] = 99.0
    assert manager._cached_arm_position[0] == 1.0


def test_feedback_poll_refreshes_arm_and_gripper_cache():
    manager = _manager_with_cache()
    manager._robot_get_state = lambda request_feedback=True: (
        np.array([0.1, 0.2, 4.5]),
        np.array([0.3, 0.4, 0.5]),
        np.array([0.6, 0.7, 0.8]),
    )

    pos, vel, torque = manager._get_arm_state(request_feedback=True)

    np.testing.assert_allclose(pos, [0.1, 0.2])
    np.testing.assert_allclose(vel, [0.3, 0.4])
    np.testing.assert_allclose(torque, [0.6, 0.7])
    assert manager._cached_gripper_position == 4.5
    assert manager._cached_gripper_velocity == 0.5
    assert manager._cached_gripper_torque == 0.8


def test_cached_gripper_state_does_not_poll_hardware():
    manager = _manager_with_cache()
    manager._gripper_name = "gripper"
    manager._cached_gripper_position = 2.5
    manager._cached_gripper_velocity = -0.4
    manager._cached_gripper_torque = 0.7
    manager._robot._motor_map = {}

    def fail_if_called(*args, **kwargs):
        raise AssertionError("safe-home cached J7 read must not poll hardware")

    manager._robot_get_state = fail_if_called
    position, velocity, torque, status = manager.get_gripper_state(
        request_feedback=False
    )

    assert position == 2.5
    assert velocity == -0.4
    assert torque == 0.7
    assert status == 0


def test_gravity_torque_uses_configured_per_joint_scale():
    manager = HardwareManager.__new__(HardwareManager)
    manager._arm_group = SimpleNamespace(joint_names=["joint1", "joint2"])
    manager._gravity_comp_joint_direction = np.ones(2)
    manager._gravity_comp_tau_scale = np.array([1.0, 1.55])
    manager._gc_model = object()
    manager._gc_data = object()
    manager._pad_q_for_model = lambda _model, q, _size: q
    manager._gc_compute_generalized_gravity = (
        lambda _model, _q, _data: np.array([2.0, 3.0])
    )

    torque = manager._gravity_comp_torque(np.zeros(2))

    np.testing.assert_allclose(torque, [2.0, 4.65])


def test_gravity_torque_applies_optional_per_joint_limit():
    manager = HardwareManager.__new__(HardwareManager)
    manager._arm_group = SimpleNamespace(joint_names=["joint1", "joint2"])
    manager._gravity_comp_joint_direction = np.array([1.0, -1.0])
    manager._gravity_comp_tau_scale = np.array([1.0, 2.0])
    manager._gravity_comp_torque_limit = np.array([0.0, 3.0])
    manager._gc_model = object()
    manager._gc_data = object()
    manager._pad_q_for_model = lambda _model, q, _size: q
    manager._gc_compute_generalized_gravity = (
        lambda _model, _q, _data: np.array([2.0, 4.0])
    )

    torque = manager._gravity_comp_torque(np.zeros(2))

    np.testing.assert_allclose(torque, [2.0, -3.0])


def test_rs_gripper_assist_respects_rest_limits_and_speed_cutoff():
    manager = HardwareManager.__new__(HardwareManager)
    manager.gripper_close_position = 0.0
    manager.gripper_open_position = 5.0
    manager._gripper_assist_velocity_threshold = 0.02
    manager._gripper_assist_velocity_full = 0.22
    manager._gripper_assist_speed_limit = 0.8
    manager._gripper_assist_torque = 0.04
    manager._gripper_assist_breakaway_fraction = 0.30

    assert manager._gripper_assist_feedforward(2.5, 0.02) == 0.0
    assert manager._gripper_assist_feedforward(0.1, -0.2) == 0.0
    assert manager._gripper_assist_feedforward(4.9, 0.2) == 0.0
    assert manager._gripper_assist_feedforward(2.5, 0.9) == 0.0
    assert manager._gripper_assist_feedforward(2.5, 0.2) > 0.0
    assert manager._gripper_assist_feedforward(2.5, -0.2) < 0.0
    assert manager._gripper_assist_feedforward(2.5, 0.22) == 0.04
    assert manager._gripper_assist_feedforward(2.5, -0.021) == -0.012


def test_idempotent_gravity_stop_does_not_enable_a_disabled_gripper():
    manager = HardwareManager.__new__(HardwareManager)
    manager._cmd_lock = threading.RLock()
    manager._gravity_comp_active = False
    manager._enabled = False
    manager._robot = SimpleNamespace(has_gripper=True)
    manager._gripper_manual_free = True
    manager._gripper_assist_active = False
    manager._hold_gripper_current_locked = lambda: (_ for _ in ()).throw(
        AssertionError("inactive stop must not energize a disabled gripper")
    )

    manager.stop_gravity_compensation()


def test_repeated_gripper_mit_samples_do_not_repeat_hardware_mode_switch():
    manager = HardwareManager.__new__(HardwareManager)
    manager._gripper_group = SimpleNamespace(mode="mit")
    manager._enter_mode = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        AssertionError("an already-MIT gripper must not switch mode again")
    )

    manager._begin_gripper_lowlevel("mit")


def test_gripper_lowlevel_switches_mode_when_required():
    manager = HardwareManager.__new__(HardwareManager)
    manager._gripper_group = SimpleNamespace(mode="pos_vel")
    calls = []
    manager._enter_mode = lambda *args: calls.append(args)

    manager._begin_gripper_lowlevel("mit")

    assert calls == [(manager._gripper_group, "mit", "gripper")]


def test_gravity_transition_smoothly_reduces_hold_gains():
    manager = HardwareManager.__new__(HardwareManager)
    manager._gravity_comp_transition_duration = 0.5
    manager._arm_mit_kp = np.array([80.0, 150.0])
    manager._arm_mit_kd = np.array([5.0, 10.0])
    manager._gravity_comp_kp = np.array([2.0, 2.0])
    manager._gravity_comp_kd = np.array([1.0, 1.0])

    start_kp, start_kd = manager._gravity_comp_transition_gains(0.0)
    middle_kp, middle_kd = manager._gravity_comp_transition_gains(0.25)
    final_kp, final_kd = manager._gravity_comp_transition_gains(0.5)

    np.testing.assert_allclose(start_kp, manager._arm_mit_kp)
    np.testing.assert_allclose(start_kd, manager._arm_mit_kd)
    np.testing.assert_allclose(
        middle_kp, (manager._arm_mit_kp + manager._gravity_comp_kp) / 2.0
    )
    np.testing.assert_allclose(
        middle_kd, (manager._arm_mit_kd + manager._gravity_comp_kd) / 2.0
    )
    np.testing.assert_allclose(final_kp, manager._gravity_comp_kp)
    np.testing.assert_allclose(final_kd, manager._gravity_comp_kd)

    assert manager._gravity_comp_transition_blend(0.0) == 0.0
    assert manager._gravity_comp_transition_blend(0.25) == 0.5
    assert manager._gravity_comp_transition_blend(0.5) == 1.0
