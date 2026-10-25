"""
Correction layer in MAPPING mode: slam_toolbox async_slam_toolbox_node.

Started (and stopped) by map_manager_node as a child process; not meant to be
started by hand next to sim.launch.py, which already runs map_manager_node.
Config: config/slam_toolbox.yaml (unchanged since Step 4).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('gnssd_bringup'),
                          'config', 'slam_toolbox.yaml')
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[config, {'use_sim_time': True}],
            output='screen',
        ),
    ])
