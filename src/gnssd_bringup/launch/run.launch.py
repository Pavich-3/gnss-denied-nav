"""
Run one complete experiment: simulation + route driver + recording.

Usage:
  ros2 launch gnssd_bringup run.launch.py scenario:=room_loop seed:=1
  ros2 launch gnssd_bringup run.launch.py scenario:=room_long seed:=1 laps:=10
  ros2 launch gnssd_bringup run.launch.py world:=warehouse scenario:=warehouse_loop seed:=1
  ros2 launch gnssd_bringup run.launch.py scenario:=room_loop seed:=2 gui:=true
  ros2 launch gnssd_bringup run.launch.py scenario:=room_dropout seed:=1 scan_dropout:="[[140,200]]"
  ros2 launch gnssd_bringup run.launch.py scenario:=room_map seed:=1 save_map_as:=room_s1
  ros2 launch gnssd_bringup run.launch.py scenario:=room_loc seed:=2 slam_mode:=localization slam_map:=room_s1

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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown)
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
    route = LaunchConfiguration('route').perform(context) or world
    save_map_as = LaunchConfiguration('save_map_as').perform(context)
    operator_recovery = LaunchConfiguration('operator_recovery').perform(context).lower() == 'true'

    changes = uncommitted_changes(repository_root())
    if changes and not allow_dirty:
        get_logger('run.launch').error(
            'The repository has uncommitted changes; refusing to record a run that '
            'cannot be traced to a commit. Commit first, or pass allow_dirty:=true.\n'
            + changes)
        return [Shutdown(reason='uncommitted changes')]

    launch_dir = os.path.join(get_package_share_directory('gnssd_bringup'), 'launch')
    route_config = os.path.join(get_package_share_directory('gnssd_bringup'),
                                'config', f'route_{route}.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'ekf': LaunchConfiguration('ekf'),
            'odom_noise': LaunchConfiguration('odom_noise'),
            'seed': LaunchConfiguration('seed'),
            'scan_dropout': LaunchConfiguration('scan_dropout'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'slam_mode': LaunchConfiguration('slam_mode'),
            'slam_map': LaunchConfiguration('slam_map'),
            'slam_x': LaunchConfiguration('slam_x'),
            'slam_y': LaunchConfiguration('slam_y'),
            'slam_yaw': LaunchConfiguration('slam_yaw'),
            'checkpoint_on_scan_lost': LaunchConfiguration('checkpoint_on_scan_lost'),
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

    shutdown = [LogInfo(msg='Route driver finished, shutting the experiment down'),
                Shutdown(reason='route finished')]
    if save_map_as:
        # FR-11: the "operator" saves the map at the end of the route, then shut down.
        save = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/map_manager_node/save_map',
                 'gnssd_interfaces/srv/SaveMap', f'{{name: {save_map_as}}}'],
            output='screen')
        on_route_done = [LogInfo(msg=f'Route finished: saving the map as {save_map_as}'), save,
                         RegisterEventHandler(OnProcessExit(target_action=save,
                                                            on_exit=shutdown))]
    else:
        on_route_done = shutdown
    stop_when_route_is_done = RegisterEventHandler(OnProcessExit(
        target_action=route_driver, on_exit=on_route_done))

    actions = [simulation, recording, route_driver, stop_when_route_is_done]
    if operator_recovery:
        actions.append(Node(
            package='gnssd_tools', executable='scripted_operator_node',
            parameters=[{'use_sim_time': True,
                         'reaction_delay': float(LaunchConfiguration('operator_delay')
                                                 .perform(context))}],
            output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='room',
                              description='room or warehouse (world file and route)'),
        DeclareLaunchArgument('scenario', default_value='room_loop'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('laps', default_value='',
                              description='Number of laps (empty = value in route_<world>.yaml)'),
        DeclareLaunchArgument('odom_noise', default_value='true'),
        DeclareLaunchArgument('scan_dropout', default_value='[]',
                              description='Lidar dropout windows "[[t_start,t_end], ...]"'),
        DeclareLaunchArgument('ekf', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('route', default_value='',
                              description='Route file config/route_<route>.yaml '
                                          '(empty = the world name)'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('slam_mode', default_value='mapping'),
        DeclareLaunchArgument('slam_map', default_value=''),
        DeclareLaunchArgument('slam_x', default_value='0.0'),
        DeclareLaunchArgument('slam_y', default_value='0.0'),
        DeclareLaunchArgument('slam_yaw', default_value='0.0'),
        DeclareLaunchArgument('checkpoint_on_scan_lost', default_value='false'),
        DeclareLaunchArgument('save_map_as', default_value='',
                              description='Save the map under this name when the route is done'),
        DeclareLaunchArgument('operator_recovery', default_value='false',
                              description='Start scripted_operator_node (Step 6 recovery test)'),
        DeclareLaunchArgument('operator_delay', default_value='3.0'),
        DeclareLaunchArgument('allow_dirty', default_value='false',
                              description='Record even with uncommitted changes in git'),
        OpaqueFunction(function=launch_setup),
    ])
