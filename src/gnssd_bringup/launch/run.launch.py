"""
Run one complete experiment: simulation + route driver + recording.

Usage:
  ros2 launch gnssd_bringup run.launch.py scenario:=room_loop seed:=1
  ros2 launch gnssd_bringup run.launch.py scenario:=room_long seed:=1 laps:=10
  ros2 launch gnssd_bringup run.launch.py world:=warehouse scenario:=warehouse_loop seed:=1
  ros2 launch gnssd_bringup run.launch.py scenario:=room_loop seed:=2 gui:=true

One `seed` argument feeds the odometry noise, the Gazebo sensor noise and the
bag name, so the bag's run_info.yaml fully identifies the run. When the route
driver has finished its laps and exits, everything is shut down and the bag
is closed.

Reproducibility guard: the run refuses to start if the git repository has
uncommitted changes, because run_info.yaml could then not say which code
produced the bag. `allow_dirty:=true` overrides this (for quick tests); the
choice is recorded in run_info.yaml.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def repository_root():
    """Return the git repository that contains this launch file (via the --symlink-install link)."""
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    result = subprocess.run(['git', 'rev-parse', '--show-toplevel'], cwd=launch_dir,
                            capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else os.getcwd()


def uncommitted_changes(repo_dir):
    """Return the output of `git status --porcelain` (empty string = clean)."""
    result = subprocess.run(['git', 'status', '--porcelain'], cwd=repo_dir,
                            capture_output=True, text=True)
    return result.stdout.strip()


def launch_setup(context):
    allow_dirty = LaunchConfiguration('allow_dirty').perform(context).lower() == 'true'
    laps = LaunchConfiguration('laps').perform(context)
    world = LaunchConfiguration('world').perform(context)

    changes = uncommitted_changes(repository_root())
    if changes and not allow_dirty:
        get_logger('run.launch').error(
            'The repository has uncommitted changes; refusing to record a run that '
            'cannot be traced to a commit. Commit first, or pass allow_dirty:=true.\n'
            + changes)
        return [Shutdown(reason='uncommitted changes')]

    launch_dir = os.path.join(get_package_share_directory('gnssd_bringup'), 'launch')
    route_config = os.path.join(get_package_share_directory('gnssd_bringup'),
                                'config', f'route_{world}.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'ekf': LaunchConfiguration('ekf'),
            'odom_noise': LaunchConfiguration('odom_noise'),
            'seed': LaunchConfiguration('seed'),
            'drive': 'false',  # the driver is started below, so we can watch its exit
        }.items(),
    )

    recording = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'record.launch.py')),
        launch_arguments={
            'scenario': LaunchConfiguration('scenario'),
            'seed': LaunchConfiguration('seed'),
            'world': world,
            'allow_dirty': str(allow_dirty).lower(),
        }.items(),
    )

    # `laps` empty = use the value from route_<world>.yaml.
    driver_parameters = [route_config, {'use_sim_time': True}]
    if laps:
        driver_parameters.append({'laps': int(laps)})

    route_driver = Node(
        package='gnssd_tools',
        executable='route_driver_node',
        parameters=driver_parameters,
        output='screen',
    )

    stop_when_route_is_done = RegisterEventHandler(OnProcessExit(
        target_action=route_driver,
        on_exit=[
            LogInfo(msg='Route driver finished, shutting the experiment down'),
            Shutdown(reason='route finished'),
        ],
    ))

    return [simulation, recording, route_driver, stop_when_route_is_done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='room',
                              description='room or warehouse (world file and route)'),
        DeclareLaunchArgument('scenario', default_value='room_loop'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('laps', default_value='',
                              description='Number of laps (empty = value in route_<world>.yaml)'),
        DeclareLaunchArgument('odom_noise', default_value='true'),
        DeclareLaunchArgument('ekf', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('allow_dirty', default_value='false',
                              description='Record even with uncommitted changes in git'),
        OpaqueFunction(function=launch_setup),
    ])
