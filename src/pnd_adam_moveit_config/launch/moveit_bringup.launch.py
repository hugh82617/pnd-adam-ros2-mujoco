import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "start_robot_state_publisher", default_value="true"
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    start_robot_state_publisher = LaunchConfiguration(
        "start_robot_state_publisher"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "adam_pro", package_name="pnd_adam_moveit_config"
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(moveit_manage_controllers=False)
        .planning_pipelines(default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    description_pkg = get_package_share_directory("pnd_adam_description")
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, "launch", "display.launch.py")
        ),
        condition=IfCondition(start_robot_state_publisher),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("pnd_adam_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        declared_args
        + [
            move_group_node,
            description_launch,
            rviz_node,
        ]
    )
