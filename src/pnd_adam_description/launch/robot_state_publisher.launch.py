import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory("pnd_adam_description"),
        "urdf",
        "pnd_adam.urdf",
    )
    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": False},
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
