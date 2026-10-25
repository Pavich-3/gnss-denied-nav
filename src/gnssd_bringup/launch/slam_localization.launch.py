"""
Correction layer in LOCALIZATION mode on a saved map: slam_toolbox localization_slam_toolbox_node.

Started (and stopped) by map_manager_node as a child process with
  map_file:=<maps folder>/<name>   (without extension; .posegraph/.data are read)
  x:= y:= yaw:=                    initial pose of the robot in the map frame
Config: config/slam_toolbox_localization.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def localization_node(context):
    config = os.path.join(get_package_share_directory('gnssd_bringup'),
                          'config', 'slam_toolbox_localization.yaml')
    start_pose = [float(LaunchConfiguration(name).perform(context)) for name in ('x', 'y', 'yaw')]
    return [Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[config, {
            'use_sim_time': True,
            'map_file_name': LaunchConfiguration('map_file').perform(context),
            'map_start_pose': start_pose,
        }],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_file', description='saved map without extension'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=localization_node),
    ])
