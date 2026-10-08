"""Keep RS leader gains independent from ordinary pose-hold gains."""
from pathlib import Path

import pytest

from rebotarmcontroller.hardware_config import _add_runtime_config, _load_ros_hardware_config


def load_rs_config():
    workspace = Path(__file__).resolve().parents[3]
    _, data = _load_ros_hardware_config(
        workspace / 'third_party/reBotArm_control_py',
        str(workspace / 'src/rebotarm_bringup/config/rebotarm_hardware.yaml'),
        'rs', 'can0',
    )
    return data


def test_leader_uses_official_gravity_gains_without_changing_hold_gains():
    control = load_rs_config()['_runtime']['control']
    assert control['leader_mit_kp'] == [50, 50, 50, 30, 50, 50]
    assert control['leader_mit_kd'] == [3, 5, 5, 3, 4, 4]
    assert control['mit_kp'] == [80, 150, 150, 50, 50, 50]
    assert control['mit_kd'] == [5, 10, 10, 5, 4, 4]


def test_configs_without_leader_gains_keep_their_existing_gains():
    data = load_rs_config()
    del data['control']['leader_mit_kp']
    del data['control']['leader_mit_kd']
    _add_runtime_config(data)
    control = data['_runtime']['control']
    assert control['leader_mit_kp'] == control['mit_kp']
    assert control['leader_mit_kd'] == control['mit_kd']


def test_leader_gripper_cap_is_separate_and_legacy_configs_keep_old_cap():
    data = load_rs_config()
    assert data['_runtime']['control']['leader_gripper_velocity_limit'] == 3.0
    del data['control']['leader_gripper_velocity_limit']
    _add_runtime_config(data)
    assert data['_runtime']['control']['leader_gripper_velocity_limit'] == 1.5


@pytest.mark.parametrize('invalid', [0, -1, float('nan'), float('inf')])
def test_invalid_leader_gripper_cap_is_rejected(invalid):
    data = load_rs_config()
    data['control']['leader_gripper_velocity_limit'] = invalid
    with pytest.raises(ValueError, match='leader_gripper_velocity_limit'):
        _add_runtime_config(data)


@pytest.mark.parametrize('gain', ['leader_mit_kp', 'leader_mit_kd'])
@pytest.mark.parametrize('invalid', [-1, float('nan'), [1, 2]])
def test_invalid_leader_gains_are_rejected(gain, invalid):
    data = load_rs_config()
    data['control'][gain] = invalid
    with pytest.raises(ValueError, match=gain):
        _add_runtime_config(data)
