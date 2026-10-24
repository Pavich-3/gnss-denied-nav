"""
Run one complete experiment: simulation + route driver + recording.

Usage:
  ros2 launch gnssd_bringup run.launch.py scenario:=loop seed:=1
  ros2 launch gnssd_bringup run.launch.py scenario:=loop seed:=1 odom_noise:=false
  ros2 launch gnssd_bringup run.launch.py scenario:=loop seed:=2 gui:=true

One `seed` argument feeds both the odometry noise and the bag name, so the
bag's run_info.yaml fully identifies the run. When the route driver has
finished its laps and exits, everything is shut down and the bag is closed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('gnssd_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    route_config = os.path.join(bringup_dir, 'config', 'route.yaml')

    arguments = [
        DeclareLaunchArgument('scenario', default_value='loop'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('odom_noise', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
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
        }.items(),
    )

    route_driver = Node(
        package='gnssd_tools',
        executable='route_driver_node',
        parameters=[route_config, {'use_sim_time': True}],
        output='screen',
    )

    stop_when_route_is_done = RegisterEventHandler(OnProcessExit(
        target_action=route_driver,
        on_exit=[
            LogInfo(msg='Route driver finished, shutting the experiment down'),
            Shutdown(reason='route finished'),
        ],
    ))

    return LaunchDescription(arguments + [
        simulation,
        recording,
        route_driver,
        stop_when_route_is_done,
    ])
