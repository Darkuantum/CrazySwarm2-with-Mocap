"""Bring up the crazyflie stack using THIS package's config.

Relocatable by construction: every path is resolved through
get_package_share_directory(), so the package works wherever it is dropped
into a workspace. Mirrors crazyflie_examples/launch/launch.py.

  ros2 launch crazyflie_shows show_launch.py                    # hardware
  ros2 launch crazyflie_shows show_launch.py backend:=sim
  ros2 launch crazyflie_shows show_launch.py backend:=sim show:=swarm_show
  ros2 launch crazyflie_shows show_launch.py backend:=sim rviz:=False

Note that crazyflies.yaml and motion_capture.yaml are overridable launch
arguments of crazyflie/launch.py, so this package ships its own copies rather
than editing the vendored ones. server.yaml and the URDF are NOT overridable
(hardcoded in that file's parse_yaml), so they still come from `crazyflie`.

`show:=` is empty by default: on hardware you want the stack up, the preflight
GUI checked, and only then the show run by hand with `ros2 run`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    backend = LaunchConfiguration('backend')
    show = LaunchConfiguration('show')
    rviz = LaunchConfiguration('rviz')

    pkg = get_package_share_directory('crazyflie_shows')

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crazyflie'), 'launch'), '/launch.py']),
        launch_arguments={
            'backend': backend,
            'crazyflies_yaml_file': os.path.join(
                pkg, 'config', 'crazyflies.yaml'),
            'motion_capture_yaml_file': os.path.join(
                pkg, 'config', 'motion_capture.yaml'),
            'rviz': rviz,
        }.items())

    show_node = Node(
        condition=IfCondition(PythonExpression(["'", show, "' != ''"])),
        package='crazyflie_shows',
        executable=show,
        name=show,
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(["'", backend, "' == 'sim'"]),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='cpp'),
        DeclareLaunchArgument('show', default_value=''),
        # Forwarded so a sim run can skip rviz2, which is the heaviest thing
        # in the stack and irrelevant when you only want the show's own
        # printout. server.yaml still decides whether the sim *publishes* the
        # visualisation markers; this only controls whether rviz2 is started.
        DeclareLaunchArgument('rviz', default_value='True'),
        stack,
        show_node,
    ])
