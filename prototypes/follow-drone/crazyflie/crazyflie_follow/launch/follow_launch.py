"""Bring up the crazyflie stack, then (optionally) follow_node.

Mirrors crazyflie_shows/launch/show_launch.py's structure and argument
style, with one deliberate difference: this package does NOT ship its own
crazyflies.yaml/motion_capture.yaml. crazyflie_shows ships copies because a
choreography wants its own drone roster and starting positions; follow_node
wants the opposite -- the SAME already-configured fleet the rest of this
rig uses (see CLAUDE.md), not a package-private copy that can drift out of
sync with it. So `crazyflie/launch.py`'s own `crazyflies_yaml_file` /
`motion_capture_yaml_file` launch arguments are left untouched here, which
makes them default to `crazyflie`'s own bundled config -- exactly what every
other node on this rig already flies against.

  ros2 launch crazyflie_follow follow_launch.py backend:=sim              # stack only
  ros2 launch crazyflie_follow follow_launch.py backend:=sim run_follow_node:=True
  ros2 launch crazyflie_follow follow_launch.py run_follow_node:=True     # hardware

`run_follow_node` defaults to False on purpose, the same reason
show_launch.py's `show:=` defaults to empty: follow_node arms and takes off
autonomously as soon as both the drone and the operator rigid bodies are
visible on /poses (see follow_node.py) -- on hardware you want the stack up
and `ros2 run crazyflie_follow check_safety` (or at least the preflight GUI)
checked *before* that can happen, not launched unconditionally as a side
effect of bringing the stack up.

keyboard_cmd is deliberately NOT started by this launch file -- it is an
interactive console (see keyboard_cmd.py) meant to be run by hand, in its own
terminal, so its key-list printout and prompt are visible.
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
    rviz = LaunchConfiguration('rviz')
    run_follow_node = LaunchConfiguration('run_follow_node')

    pkg = get_package_share_directory('crazyflie_follow')
    follow_config = os.path.join(pkg, 'config', 'follow.yaml')

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crazyflie'), 'launch'), '/launch.py']),
        launch_arguments={
            'backend': backend,
            'rviz': rviz,
        }.items())

    follow_node = Node(
        condition=IfCondition(run_follow_node),
        package='crazyflie_follow',
        executable='follow_node',
        name='follow_node',
        output='screen',
        parameters=[
            follow_config,
            {'use_sim_time': PythonExpression(["'", backend, "' == 'sim'"])},
        ])

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='cpp'),
        DeclareLaunchArgument('run_follow_node', default_value='False'),
        # Same reasoning as show_launch.py: rviz2 is the heaviest thing in
        # the stack and irrelevant to what this node prints on its own.
        DeclareLaunchArgument('rviz', default_value='True'),
        stack,
        follow_node,
    ])
