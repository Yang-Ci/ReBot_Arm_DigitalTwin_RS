from types import SimpleNamespace

import pytest

from rebotarmcontroller.fake_rs_driver import FakeRsDriver


@pytest.mark.parametrize(('speed', 'expected'), [(0.0, 0.5), (1.0, 0.01), (2.0, 0.02)])
def test_fake_driver_speed_limit_can_be_disabled(speed, expected):
    class Stamp:
        def __sub__(self, other):
            return SimpleNamespace(nanoseconds=10_000_000)

    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=Stamp), last_time=Stamp(),
        positions=[0.0] * 6, targets=[0.5] * 6, velocities=[0.0] * 6,
        max_joint_speed=speed, gripper_position=0.0, gripper_target=1.0,
        gripper_velocity=0.0, max_gripper_speed=5.0,
        state_machine='LOWLEVEL_STREAMING', publish_status=lambda: None,
        _publish_states=lambda now: None, _step_towards=FakeRsDriver._step_towards,
    )
    FakeRsDriver._tick_locked(node)
    assert node.positions == pytest.approx([expected] * 6)
    assert node.gripper_position == pytest.approx(0.05)
