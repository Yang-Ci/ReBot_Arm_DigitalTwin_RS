from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    arm_namespace = LaunchConfiguration("arm_namespace")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    max_joint_speed = LaunchConfiguration("max_joint_speed")
    max_gripper_speed = LaunchConfiguration("max_gripper_speed")
    start_enabled = LaunchConfiguration("start_enabled")
    use_rviz = LaunchConfiguration("use_rviz")

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

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebot_arm_rs_sim"),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("max_joint_speed", default_value="1.0",
                                  description="Fake driver joint speed in rad/s; 0 disables the limit"),
            DeclareLaunchArgument("max_gripper_speed", default_value="5.0"),
            DeclareLaunchArgument("start_enabled", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            Node(
                package="rebotarmcontroller",
                executable="FakeRsDriver",
                name="fake_rebotarm_rs_driver",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "joint_state_rate": joint_state_rate,
                        "max_joint_speed": max_joint_speed,
                        "max_gripper_speed": max_gripper_speed,
                        "start_enabled": ParameterValue(start_enabled, value_type=bool),
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="fake_rs_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="fake_rs_rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
