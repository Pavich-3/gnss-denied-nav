"""
Correction layer in MAPPING mode: slam_toolbox async_slam_toolbox_node.

Started (and stopped) by map_manager_node as a child process; not meant to be
started by hand next to sim.launch.py, which already runs map_manager_node.
Config: config/slam_toolbox.yaml (unchanged since Step 4).

Without arguments a new map is started. With
  map_file:=<maps folder>/<name>   (without extension; .posegraph/.data are read)
  x:= y:= yaw:=                    pose of the robot in the map frame
the saved pose graph is loaded and mapping CONTINUES on it from that pose
(slam_toolbox reads map_file_name + map_start_pose at start-up and
deserializes with START_AT_GIVEN_POSE; Step 6b recovery).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def mapping_node(context):
    config = os.path.join(get_package_share_directory('gnssd_bringup'),
                          'config', 'slam_toolbox.yaml')
    overrides = {'use_sim_time': True}
    map_file = LaunchConfiguration('map_file').perform(context)
    if map_file:
        overrides['map_file_name'] = map_file
        overrides['map_start_pose'] = [float(LaunchConfiguration(name).perform(context))
                                       for name in ('x', 'y', 'yaw')]
    return [Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[config, overrides],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value='',
                              description='saved map to continue (empty = new map)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=mapping_node),
    ])
