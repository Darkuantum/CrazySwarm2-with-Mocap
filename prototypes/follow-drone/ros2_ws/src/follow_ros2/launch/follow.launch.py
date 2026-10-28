"""Bring up all four follow_ros2 nodes with the shared config file.

Uses only launch/launch_ros APIs stable since Foxy (plain Node actions,
DeclareLaunchArgument, LaunchConfiguration) -- no post-Foxy launch features.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("follow_ros2"), "config", "follow_params.yaml"
    )

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the follow_ros2 parameters YAML file.",
    )
    config = LaunchConfiguration("config_file")

    nodes = [
        Node(
            package="follow_ros2",
            executable="follow_node",
            name="follow_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="follow_ros2",
            executable="detector_bridge",
            name="detector_bridge",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="follow_ros2",
            executable="uwb_node",
            name="uwb_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="follow_ros2",
            executable="mapper_bridge",
            name="mapper_bridge",
            output="screen",
            parameters=[config],
        ),
    ]

    return LaunchDescription([config_arg] + nodes)
