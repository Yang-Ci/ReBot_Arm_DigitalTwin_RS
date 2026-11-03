import os
import sys
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def _resolve_venv_pythonpath():
    """算出 venv site-packages 路径并拼到 PYTHONPATH 前面。

    reBotArmController 可执行文件用系统 /usr/bin/python3，而 motorbridge 等
    依赖只装在 .venv 里。这里自动定位 venv site-packages 并注入 PYTHONPATH，
    让直接 ros2 launch 也能找到这些包，无需手动 source rs_env.sh。
    """
    try:
        share_dir = get_package_share_directory("rebotarm_bringup")
        # share_dir 是 .../install/rebotarm_bringup/share/rebotarm_bringup
        # 往上四级到工作区根目录
        ws_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(share_dir)))
        )
        pyver = "python{}.{}".format(sys.version_info.major, sys.version_info.minor)
        venv_site = os.path.join(ws_root, ".venv", "lib", pyver, "site-packages")
        cmeel_site = os.path.join(
            venv_site, "cmeel.prefix", "lib", pyver, "site-packages"
        )
        paths = []
        if os.path.isdir(cmeel_site):
            paths.append(cmeel_site)
        if os.path.isdir(venv_site):
            paths.append(venv_site)
        if not paths:
            return ""
        prefix = ":".join(paths)
        current = os.environ.get("PYTHONPATH", "")
        return "{}:{}".format(prefix, current) if current else prefix
    except Exception:
        return ""


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    hardware_config = LaunchConfiguration("hardware_config")
    model = LaunchConfiguration("model")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    hardware_feedback_poll_rate = LaunchConfiguration("hardware_feedback_poll_rate")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
    disable_after_safe_home = LaunchConfiguration("disable_after_safe_home")
    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "ReBot_Arm_RS.urdf"]
    )
    rviz_urdf_compat = PathJoinSubstitution(
        [bringup_share, "launch", "rviz_urdf_compat.py"]
    )
    rviz_config = PathJoinSubstitution([bringup_share, "rviz", "rebotarm.rviz"])
    robot_description = ParameterValue(
        Command(["python3 ", rviz_urdf_compat, " ", urdf_file]), value_type=str
    )

    # 把 venv 的 site-packages 注入到控制器节点的 PYTHONPATH，
    # 这样不 source rs_env.sh 也能找到 motorbridge 等依赖
    controller_env = {}
    venv_pythonpath = _resolve_venv_pythonpath()
    if venv_pythonpath:
        controller_env["PYTHONPATH"] = venv_pythonpath

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "rebotarm_hardware.yaml"]
                ),
            ),
            DeclareLaunchArgument("model", default_value="rs"),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("hardware_feedback_poll_rate", default_value="100.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebot_arm_rs"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="gripper_end"),
            DeclareLaunchArgument("disable_after_safe_home", default_value="true"),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                on_exit=Shutdown(reason="reBotArmController exited"),
                additional_env=controller_env,
                parameters=[
                    {
                        "hardware_config": hardware_config,
                        "model": model,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "hardware_feedback_poll_rate": hardware_feedback_poll_rate,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "frame_id": frame_id,
                        "ee_frame_id": ee_frame_id,
                        "disable_after_safe_home": ParameterValue(
                            disable_after_safe_home,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
