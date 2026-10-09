from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm_rs"),
            DeclareLaunchArgument("input_topic", default_value=""),
            DeclareLaunchArgument("output_topic", default_value=""),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("wrist_camera_model", default_value="d405"),
            DeclareLaunchArgument("simulation_mode", default_value="kinematic"),
            DeclareLaunchArgument("update_rate", default_value="250.0"),
            DeclareLaunchArgument("smoothing_alpha", default_value="1.0"),
            DeclareLaunchArgument("stale_timeout", default_value="1.0"),
            DeclareLaunchArgument("use_viewer", default_value="false"),
            DeclareLaunchArgument("show_tcp_marker", default_value="false"),
            DeclareLaunchArgument("enable_camera_selector", default_value=LaunchConfiguration("use_viewer")),
            DeclareLaunchArgument("gripper_kp", default_value="1800.0"),
            DeclareLaunchArgument("gripper_kd", default_value="18.0"),
            DeclareLaunchArgument("gripper_tau_limit", default_value="64.0"),
            DeclareLaunchArgument("enable_task_tools", default_value="true"),
            DeclareLaunchArgument("enable_wrist_camera", default_value="true"),
            Node(
                package="rebotarm_mujoco_rs",
                executable="mujoco_rs_sync",
                name="rebotarm_rs_mujoco",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "model_path": LaunchConfiguration("model_path"),
                        "wrist_camera_model": LaunchConfiguration("wrist_camera_model"),
                        "simulation_mode": LaunchConfiguration("simulation_mode"),
                        "update_rate": ParameterValue(
                            LaunchConfiguration("update_rate"), value_type=float
                        ),
                        "smoothing_alpha": ParameterValue(
                            LaunchConfiguration("smoothing_alpha"), value_type=float
                        ),
                        "stale_timeout": ParameterValue(
                            LaunchConfiguration("stale_timeout"), value_type=float
                        ),
                        "use_viewer": ParameterValue(
                            LaunchConfiguration("use_viewer"), value_type=bool
                        ),
                        "show_tcp_marker": ParameterValue(
                            LaunchConfiguration("show_tcp_marker"), value_type=bool
                        ),
                        "gripper_kp": ParameterValue(
                            LaunchConfiguration("gripper_kp"), value_type=float
                        ),
                        "gripper_kd": ParameterValue(
                            LaunchConfiguration("gripper_kd"), value_type=float
                        ),
                        "gripper_tau_limit": ParameterValue(
                            LaunchConfiguration("gripper_tau_limit"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="rebotarm_mujoco_rs",
                executable="wrist_camera_selector",
                name="rebotarm_rs_wrist_camera_selector",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_camera_selector")),
                parameters=[{"arm_namespace": LaunchConfiguration("arm_namespace")}],
            ),
            Node(
                package="rebotarm_mujoco_rs",
                executable="rs_task_server",
                name="rebotarm_rs_task_server",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_task_tools")),
                parameters=[
                    {
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "model_path": LaunchConfiguration("model_path"),
                    }
                ],
            ),
            Node(
                package="rebotarm_mujoco_rs",
                executable="rs_scene_camera",
                name="rebotarm_rs_scene_camera",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_task_tools")),
                parameters=[
                    {
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "model_path": LaunchConfiguration("model_path"),
                        "wrist_camera_model": LaunchConfiguration("wrist_camera_model"),
                    }
                ],
            ),
            Node(
                package="rebotarm_mujoco_rs",
                executable="rs_scene_camera",
                name="rebotarm_rs_wrist_camera",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_wrist_camera")),
                parameters=[
                    {
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "model_path": LaunchConfiguration("model_path"),
                        "camera_name": "wrist_rgb",
                        "wrist_camera_model": LaunchConfiguration("wrist_camera_model"),
                        "frame_id": "wrist_rgb_frame",
                        "publish_hz": 12.0,
                    }
                ],
            ),
            Node(
                package="rebotarm_mujoco_rs",
                executable="rs_scene_detector",
                name="rebotarm_rs_scene_detector",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_task_tools")),
                parameters=[
                    {"arm_namespace": LaunchConfiguration("arm_namespace")}
                ],
            ),
        ]
    )
