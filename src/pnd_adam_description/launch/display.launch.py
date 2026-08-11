import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pnd_adam_description")
    urdf_file = os.path.join(pkg_share, "urdf", "pnd_adam.urdf")
    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description = f.read()
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    system_robot_state_publisher = os.path.join(
        "/opt/ros",
        ros_distro,
        "lib",
        "robot_state_publisher",
        "robot_state_publisher",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false"
            ),
            Node(
                executable=system_robot_state_publisher,
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"publish_frequency": 50.0},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0", "0", "0",
                    "0", "0", "0",
                    "world", "base_link",
                ],
                output="screen",
            ),
        ]
    )
