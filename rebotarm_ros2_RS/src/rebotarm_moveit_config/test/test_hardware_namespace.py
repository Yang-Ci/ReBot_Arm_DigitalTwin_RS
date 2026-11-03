"""Verify MoveIt feedback and trajectory routing use the same arm namespace."""

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("moveit_configs_utils")
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters


@pytest.mark.parametrize(
    ("configured", "expected"),
    [("rebot_arm_rs", "rebot_arm_rs"), ("/test_arm/", "test_arm")],
)
def test_moveit_controller_and_feedback_follow_namespace(configured, expected):
    launch_path = Path(__file__).resolve().parents[1] / "launch/hardware.launch.py"
    spec = importlib.util.spec_from_file_location("rs_hardware_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(
        model="rs", arm_namespace=configured, use_rviz="false", rviz_config="moveit.rviz"
    )

    _, robot_state_publisher, move_group, rviz, _ = module._launch_setup(context)
    parameters = evaluate_parameters(context, move_group._Node__parameters)[0]
    prefix = "moveit_simple_controller_manager"
    assert parameters[f"{prefix}.controller_names"] == (expected,)
    assert parameters[f"{prefix}.{expected}.action_ns"] == "follow_joint_trajectory"
    for node in (robot_state_publisher, move_group, rviz):
        remappings = [
            (perform_substitutions(context, source), perform_substitutions(context, target))
            for source, target in node._Node__remappings
        ]
        assert ("/joint_states", f"/{expected}/joint_states") in remappings
