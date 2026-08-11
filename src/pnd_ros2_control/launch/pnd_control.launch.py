import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory("pnd_adam_description")
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_pkg,
                "launch",
                "robot_state_publisher.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("domain_id", default_value="1"),
            DeclareLaunchArgument("interface", default_value="lo"),
            DeclareLaunchArgument("control_rate", default_value="500.0"),
            DeclareLaunchArgument("state_publish_rate", default_value="100.0"),
            DeclareLaunchArgument("publish_hand_joints", default_value="true"),
            DeclareLaunchArgument("idle_kp", default_value="5.0"),
            DeclareLaunchArgument("idle_kd", default_value="1.0"),
            Node(
                package="pnd_ros2_control",
                executable="pnd_ros2_control_bridge",
                output="screen",
                parameters=[
                    {
                        "domain_id": LaunchConfiguration("domain_id"),
                        "interface": LaunchConfiguration("interface"),
                        "control_rate": LaunchConfiguration("control_rate"),
                        "idle_kp": LaunchConfiguration("idle_kp"),
                        "idle_kd": LaunchConfiguration("idle_kd"),
                    }
                ],
            ),
            Node(
                package="pnd_ros2_control",
                executable="pnd_joint_state_publisher",
                output="screen",
                parameters=[
                    {
                        "domain_id": LaunchConfiguration("domain_id"),
                        "interface": LaunchConfiguration("interface"),
                        "publish_rate": LaunchConfiguration(
                            "state_publish_rate"
                        ),
                        "publish_hand_joints": LaunchConfiguration(
                            "publish_hand_joints"
                        ),
                    }
                ],
            ),
            robot_state_publisher_launch,
        ]
    )
