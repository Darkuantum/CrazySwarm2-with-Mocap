"""Bring up the crazyflie stack for a show, from the workspace's own config.

Relocatable by construction: every path is resolved through
get_package_share_directory(), so the package works wherever it is dropped
into a workspace. Mirrors crazyflie_examples/launch/launch.py.

  ros2 launch crazyflie_shows show_launch.py                    # hardware
  ros2 launch crazyflie_shows show_launch.py backend:=sim
  ros2 launch crazyflie_shows show_launch.py backend:=sim show:=swarm_show
  ros2 launch crazyflie_shows show_launch.py backend:=sim rviz:=False

**One fleet, one file.** crazyflies.yaml and motion_capture.yaml default to
the `crazyflie` package's -- the same files the server reads and
`sync_initial_positions.py` writes -- so adding a drone is one edit, not two.
This package used to ship its own copies (it had to: the show lived outside the
workspace), and on 2026-10-02 those copies had drifted: no cf8 in the show's
copy while the workspace yaml had it, i.e. the show was planned against one
fleet and flown with another. Both are still launch arguments, so a different
fleet is `crazyflies_yaml_file:=/path/to/other.yaml`. server.yaml and the URDF
were never overridable (hardcoded in launch.py's parse_yaml).

`show:=` is empty by default: on hardware you want the stack up, the preflight
GUI checked, and only then the show run by hand with `ros2 run`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def announce_fleet_yaml(context, *_):
    """Say out loud which crazyflies.yaml the stack is about to read.

    An override is legitimate but it silently decouples two things that must
    agree: the file the server seeds positions from, and the file
    sync_initial_positions.py writes (which defaults to the workspace's).
    Nothing can detect that from the script's side, so the launch says it here,
    with the command that would sync the right file.
    """
    path = os.path.realpath(
        LaunchConfiguration('crazyflies_yaml_file').perform(context))
    default = os.path.realpath(os.path.join(
        get_package_share_directory('crazyflie'), 'config', 'crazyflies.yaml'))
    if path != default:
        print('\n  *** FLEET YAML OVERRIDE ***\n'
              f'  reading   {path}\n'
              f'  NOT       {default}  (the workspace default)\n'
              '  Positions in that file are not kept in step by default:\n'
              f'    python3 scripts/sync_initial_positions.py --yaml {path}\n')
    return []


def generate_launch_description():
    backend = LaunchConfiguration('backend')
    show = LaunchConfiguration('show')
    rviz = LaunchConfiguration('rviz')

    pkg = get_package_share_directory('crazyflie_shows')
    crazyflies_yaml = LaunchConfiguration('crazyflies_yaml_file')
    mocap_yaml = LaunchConfiguration('motion_capture_yaml_file')

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crazyflie'), 'launch'), '/launch.py']),
        # No crazyflies_yaml_file / motion_capture_yaml_file override: the
        # stack's own config IS the config. This package shipped copies while
        # the show lived outside the workspace; keeping them inside it meant
        # editing the fleet twice and planning against whichever copy was
        # stale (2026-10-02: the copy had no cf8, the workspace yaml did).
        # To fly a different fleet, pass launch.py's own argument through:
        #   ros2 launch crazyflie_shows show_launch.py \
        #       crazyflies_yaml_file:=/path/to/other.yaml
        launch_arguments={
            'backend': backend,
            'crazyflies_yaml_file': crazyflies_yaml,
            'motion_capture_yaml_file': mocap_yaml,
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
        # Defaults deliberately EMPTY-less: they name the workspace's own
        # config, the single source the server reads. Pass another path to fly
        # a different fleet without editing anything.
        DeclareLaunchArgument(
            'crazyflies_yaml_file',
            default_value=os.path.join(get_package_share_directory('crazyflie'),
                                       'config', 'crazyflies.yaml')),
        DeclareLaunchArgument(
            'motion_capture_yaml_file',
            default_value=os.path.join(get_package_share_directory('crazyflie'),
                                       'config', 'motion_capture.yaml')),
        OpaqueFunction(function=announce_fleet_yaml),
        stack,
        show_node,
    ])
