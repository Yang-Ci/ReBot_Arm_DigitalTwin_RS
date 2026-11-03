import os
from importlib.machinery import SourceFileLoader
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from moveit_configs_utils import MoveItConfigsBuilder

moveit_launch_common = SourceFileLoader(
    "moveit_launch_common",
    os.path.join(os.path.dirname(__file__), "moveit_launch_common.py"),
).load_module()
apply_rviz_urdf_compat = moveit_launch_common.apply_rviz_urdf_compat
moveit_parameters = moveit_launch_common.moveit_parameters


def _default_model():
    try:
        path = Path(
            get_package_share_directory("rebotarm_bringup")
        ) / "config" / "rebotarm_hardware.yaml"
        if path.exists():
            with open(path, "r", encoding="utf-8") as f:
                return str((yaml.safe_load(f) or {}).get("default_model") or "rs")
    except (PackageNotFoundError, OSError):
        pass
    return "rs"


def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file in rebotarm_moveit_config/launch",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz with the MoveIt motion planning plugin",
    )
    arm_namespace_arg = DeclareLaunchArgument(
        "arm_namespace",
        default_value="rebot_arm_rs",
        description="Namespace used by an already-running reBotArmController",
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=_default_model(),
        description="RS model (this workspace only contains the RS robot)",
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            use_rviz_arg,
            arm_namespace_arg,
            model_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    model = LaunchConfiguration("model").perform(context).strip().lower()
    if model != "rs":
        raise RuntimeError("This workspace contains only the RS model; use model:=rs")
    arm_namespace = LaunchConfiguration("arm_namespace").perform(context).strip("/")
    if not arm_namespace:
        raise RuntimeError("arm_namespace must not be empty")

    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(
            file_path="config/rebotarm_rs.urdf.xacro"
        )
        .robot_description_semantic(
            file_path="config/rebotarm_rs.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_hardware_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    apply_rviz_urdf_compat(moveit_config)
    moveit_params = moveit_parameters(moveit_config)
    # The trajectory action must follow the same namespace as joint feedback.
    controllers = moveit_params["moveit_simple_controller_manager"]
    default_controller = controllers["controller_names"][0]
    if arm_namespace != default_controller:
        controllers[arm_namespace] = controllers.pop(default_controller)
        controllers["controller_names"] = [arm_namespace]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
        remappings=[("/joint_states", f"/{arm_namespace}/joint_states")],
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("rebotarm_moveit_config"),
            "launch",
            LaunchConfiguration("rviz_config"),
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[moveit_params],
        remappings=[("/joint_states", f"/{arm_namespace}/joint_states")],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
        remappings=[("/joint_states", f"/{arm_namespace}/joint_states")],
    )

    return [
        static_tf_node,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=move_group_node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="move_group exited"))
                ],
            )
        ),
    ]
