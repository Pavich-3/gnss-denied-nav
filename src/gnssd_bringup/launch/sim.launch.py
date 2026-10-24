"""
Start the simulation stand: Gazebo world, robot, ROS-Gazebo bridge, odometry noise.

Usage:
  ros2 launch gnssd_bringup sim.launch.py                    # simulation with Gazebo GUI
  ros2 launch gnssd_bringup sim.launch.py rviz:=true         # ... plus RViz
  ros2 launch gnssd_bringup sim.launch.py drive:=true        # ... and drive the route
  ros2 launch gnssd_bringup sim.launch.py odom_noise:=false  # /odom == /odom_raw (baseline)
  ros2 launch gnssd_bringup sim.launch.py gui:=false         # no Gazebo window (batch runs)

What gets started:
  gz sim                 the room world (server, plus GUI unless gui:=false)
  robot_state_publisher  URDF -> /robot_description and TF of the robot's own links
  create                 spawns the robot into Gazebo from /robot_description
  parameter_bridge       topics listed in config/bridge.yaml
  odom_noise_node        /odom_raw -> /odom (noisy wheel odometry)
  route_driver_node      only with drive:=true, follows config/route.yaml
  rviz2                  only with rviz:=true

Nothing here publishes odom -> base_link; that is the EKF's job (later step).
RViz therefore uses base_link as its fixed frame.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def default_noise_seed(odom_noise_config):
    """Read the default seed from odom_noise.yaml, so the YAML stays the only place for it."""
    with open(odom_noise_config) as f:
        return str(yaml.safe_load(f)['odom_noise_node']['ros__parameters']['seed'])


def warn_if_not_at_origin(context):
    """Mapping runs must start at the world origin; warn if the spawn pose is different."""
    spawn = [float(LaunchConfiguration(name).perform(context))
             for name in ('spawn_x', 'spawn_y', 'spawn_yaw')]
    if any(value != 0.0 for value in spawn):
        get_logger('sim.launch').warning(
            f'Robot spawned at x={spawn[0]}, y={spawn[1]}, yaw={spawn[2]}, not at the origin. '
            'ATE without alignment requires start at origin for mapping runs.')
    return []


def gazebo_command(context):
    """Build the `gz sim` command: with or without the GUI."""
    world = LaunchConfiguration('world').perform(context)
    with_gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    command = ['gz', 'sim', '-r']  # -r: start running, not paused
    if not with_gui:
        # Server only. --headless-rendering lets the gpu_lidar render
        # without a window (EGL on the GPU).
        command += ['-s', '--headless-rendering']
    command.append(world)

    # Started directly, without a shell, so that Ctrl+C / shutdown reaches gz itself.
    return [ExecuteProcess(cmd=command, output='screen', on_exit=Shutdown())]


def generate_launch_description():
    description_dir = get_package_share_directory('gnssd_description')
    bringup_dir = get_package_share_directory('gnssd_bringup')

    robot_xacro = os.path.join(description_dir, 'urdf', 'robot.urdf.xacro')
    bridge_config = os.path.join(bringup_dir, 'config', 'bridge.yaml')
    odom_noise_config = os.path.join(bringup_dir, 'config', 'odom_noise.yaml')
    route_config = os.path.join(bringup_dir, 'config', 'route.yaml')
    rviz_config = os.path.join(bringup_dir, 'rviz', 'sim.rviz')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(description_dir, 'worlds', 'room.sdf'),
            description='Gazebo world file'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the Gazebo window (false = server only, for batch runs)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz with config rviz/sim.rviz'),
        DeclareLaunchArgument(
            'drive', default_value='false',
            description='Start route_driver_node (drives config/route.yaml)'),
        DeclareLaunchArgument(
            'odom_noise', default_value='true',
            description='Add noise to wheel odometry (false = /odom equals /odom_raw)'),
        DeclareLaunchArgument(
            'seed', default_value=default_noise_seed(odom_noise_config),
            description='Seed of the odometry noise (default: value in odom_noise.yaml)'),
        # Start pose. Must stay (0, 0, 0) for mapping runs, see CLAUDE.md.
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
    ]

    # The URDF is generated from xacro every time the launch file starts.
    robot_description = ParameterValue(Command(['xacro ', robot_xacro]), value_type=str)

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
                # Launch arguments override `enabled` and `seed` from the YAML file.
                'enabled': ParameterValue(LaunchConfiguration('odom_noise'), value_type=bool),
                'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
            },
        ],
        output='screen',
    )

    route_driver = Node(
        package='gnssd_tools',
        executable='route_driver_node',
        parameters=[route_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('drive')),
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
        OpaqueFunction(function=warn_if_not_at_origin),
        OpaqueFunction(function=gazebo_command),
        robot_state_publisher,
        spawn_robot,
        bridge,
        odom_noise,
        route_driver,
        rviz,
    ])
