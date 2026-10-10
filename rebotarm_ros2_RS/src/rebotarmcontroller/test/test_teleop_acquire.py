"""Exercise lease acquisition with real constructor state and no CAN IO."""
import time
from unittest.mock import patch

import numpy as np
import pytest

from rebotarmcontroller.hardware_config import resolve_hardware_config
from rebotarmcontroller.hardware_manager import HardwareManager


@pytest.fixture
def hardware():
    resolve_hardware_config(None, 'rs', 'can0')
    from reBotArm_control_py.actuator import RebotArm

    with patch.object(RebotArm, 'connect', side_effect=AssertionError('no CAN in tests')), \
            patch.object(RebotArm, '_setup_motors', side_effect=AssertionError('no motor IO in tests')):
        manager = HardwareManager(model='rs', channel='can0')
        manager._enabled = True
        manager._feedback_refreshed_at = time.monotonic()
        with patch.object(manager, '_begin_lowlevel_streaming') as arm_mode, \
                patch.object(manager, '_begin_gripper_lowlevel') as gripper_mode:
            yield manager, arm_mode, gripper_mode


def test_constructor_state_allows_leader_acquisition_and_release(hardware):
    manager, arm_mode, gripper_mode = hardware
    token = manager.teleop_acquire()
    assert token and manager.teleop_owned
    arm_mode.assert_called_once_with('mit')
    gripper_mode.assert_called_once_with('mit')
    np.testing.assert_allclose(manager._mit_stream_target, manager._cached_arm_position)
    manager.teleop_stop(token)
    assert not manager.teleop_owned
    assert manager._mit_stream_target is None and manager._mit_gripper_target is None


def test_assisted_gripper_still_blocks_acquisition_before_mode_entry(hardware):
    manager, arm_mode, gripper_mode = hardware
    manager._gripper_assist_active = True
    with pytest.raises(RuntimeError, match='position hold'):
        manager.teleop_acquire()
    assert not manager.teleop_owned
    arm_mode.assert_not_called()
    gripper_mode.assert_not_called()
