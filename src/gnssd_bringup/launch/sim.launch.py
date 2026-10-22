"""
Start the simulation stand: Gazebo world, robot, ROS-Gazebo bridge, odometry noise.

Usage:
  ros2 launch gnssd_bringup sim.launch.py                    # simulation only
  ros2 launch gnssd_bringup sim.launch.py rviz:=true         # ... plus RViz
  ros2 launch gnssd_bringup sim.launch.py odom_noise:=false  # /odom == /odom_raw (baseline)

What gets started:
  gz sim                 the room world, with the GUI
  robot_state_publisher  URDF -> /robot_description and TF of the robot's own links
  create                 spawns the robot into Gazebo from /robot_description
  parameter_bridge       topics listed in config/bridge.yaml
  odom_noise_node        /odom_raw -> /odom (noisy wheel odometry)
  rviz2                  only with rviz:=true

Nothing here publishes odom -> base_link; that is the EKF's job (later step).
RViz therefore uses base_link as its fixed frame.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('gnssd_description')
    bringup_dir = get_package_share_directory('gnssd_bringup')
    gz_sim_dir = get_package_share_directory('ros_gz_sim')

    robot_xacro = os.path.join(description_dir, 'urdf', 'robot.urdf.xacro')
    bridge_config = os.path.join(bringup_dir, 'config', 'bridge.yaml')
    odom_noise_config = os.path.join(bringup_dir, 'config', 'odom_noise.yaml')
    rviz_config = os.path.join(bringup_dir, 'rviz', 'sim.rviz')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(description_dir, 'worlds', 'room.sdf'),
            description='Gazebo world file'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz with config rviz/sim.rviz'),
        DeclareLaunchArgument(
            'odom_noise', default_value='true',
            description='Add noise to wheel odometry (false = /odom equals /odom_raw)'),
        # Start pose of the robot: the "S" corner of the room, see room.sdf.
        DeclareLaunchArgument('spawn_x', default_value='-3.5'),
        DeclareLaunchArgument('spawn_y', default_value='-2.5'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
    ]

    # The URDF is generated from xacro every time the launch file starts.
    robot_description = ParameterValue(Command(['xacro ', robot_xacro]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],  # -r: start running, not paused
            'on_exit_shutdown': 'true',  # closing the Gazebo window stops everything
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'gnssd_robot',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', '0.05',  # slightly above the floor; the robot drops onto its wheels
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    odom_noise = Node(
        package='gnssd_tools',
        executable='odom_noise_node',
        parameters=[
            odom_noise_config,
            {
                'use_sim_time': True,
                # The launch argument overrides `enabled` from the YAML file.
                'enabled': ParameterValue(LaunchConfiguration('odom_noise'), value_type=bool),
            },
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(arguments + [
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        odom_noise,
        rviz,
    ])
