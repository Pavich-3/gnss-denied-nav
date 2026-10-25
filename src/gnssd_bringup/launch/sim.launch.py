"""
Start the simulation stand: Gazebo world, robot, ROS-Gazebo bridge, odometry noise.

Usage:
  ros2 launch gnssd_bringup sim.launch.py                    # simulation with Gazebo GUI
  ros2 launch gnssd_bringup sim.launch.py rviz:=true         # ... plus RViz
  ros2 launch gnssd_bringup sim.launch.py drive:=true        # ... and drive the route
  ros2 launch gnssd_bringup sim.launch.py odom_noise:=false  # /odom == /odom_raw (baseline)
  ros2 launch gnssd_bringup sim.launch.py gui:=false         # no Gazebo window (batch runs)
  ros2 launch gnssd_bringup sim.launch.py world:=warehouse   # the 30 x 20 m warehouse
  ros2 launch gnssd_bringup sim.launch.py scan_dropout:="[[140,200]]"   # lidar off 140-200 s
  ros2 launch gnssd_bringup sim.launch.py ekf:=false         # without the EKF base layer (and SLAM)
  ros2 launch gnssd_bringup sim.launch.py slam:=false        # base layer only, no correction layer
  ros2 launch gnssd_bringup sim.launch.py slam_mode:=localization slam_map:=room_s1
                                                             # localize on maps/room_s1 (FR-12)

What gets started:
  gz sim                 worlds/<world>.sdf (server, plus GUI unless gui:=false)
  robot_state_publisher  URDF -> /robot_description and TF of the robot's own links
  create                 spawns the robot into Gazebo from /robot_description
  parameter_bridge       topics listed in config/bridge.yaml
  odom_noise_node        stand: /odom_raw -> /odom (noisy wheel odometry)
  imu_covariance_node    stand: /imu_raw -> /imu (adds the IMU covariances)
  scan_gate_node         stand: /scan_raw -> /scan, drops scans in the scan_dropout windows
                         (always in the chain; no windows = pass-through)
  sensor_monitor_node    system: reports lost sensor streams on /diagnostics
  ekf_filter_node        base layer: /odom + /imu -> /odometry/filtered and TF odom -> base_link
  map_manager_node       system: starts slam_toolbox (correction layer: /scan -> map and
                         TF map -> odom) in slam_mode, saves / loads maps on operator command
                         (started once the EKF process has started; it needs odom -> base_link)
  route_driver_node      only with drive:=true, follows config/route_<world>.yaml
  rviz2                  only with rviz:=true

The `seed` argument drives ALL randomness of a run: the odometry noise and,
through `gz sim --seed`, the sensor noise in Gazebo (e.g. the gyro bias).
"""

import math
import os
import subprocess
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.logging import get_logger
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def default_noise_seed(odom_noise_config):
    """Read the default seed from odom_noise.yaml, so the YAML stays the only place for it."""
    with open(odom_noise_config) as f:
        return str(yaml.safe_load(f)['odom_noise_node']['ros__parameters']['seed'])


def xacro_property(xacro_file, name):
    """
    Read the value of a <xacro:property> from a xacro file.

    Used to give nodes the same numbers as the robot model (e.g. the IMU
    noise sigmas) without copying them into a second file.
    """
    xacro_ns = '{http://www.ros.org/wiki/xacro}'
    for element in ElementTree.parse(xacro_file).getroot().iter(xacro_ns + 'property'):
        if element.get('name') == name:
            return float(element.get('value'))
    raise KeyError(f'xacro property {name} not found in {xacro_file}')


def repository_root():
    """Return the git repository that contains this launch file (via the --symlink-install link)."""
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    result = subprocess.run(['git', 'rev-parse', '--show-toplevel'], cwd=launch_dir,
                            capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else os.getcwd()


def warn_if_not_at_origin(context):
    """Mapping runs must start at the world origin; warn if the spawn pose is different."""
    spawn = [float(LaunchConfiguration(name).perform(context))
             for name in ('spawn_x', 'spawn_y', 'spawn_yaw')]
    mapping = LaunchConfiguration('slam_mode').perform(context) == 'mapping'
    if mapping and any(value != 0.0 for value in spawn):
        get_logger('sim.launch').warning(
            f'Robot spawned at x={spawn[0]}, y={spawn[1]}, yaw={spawn[2]}, not at the origin. '
            'ATE without alignment requires start at origin for mapping runs.')
    return []


def world_file(world_name):
    """Return worlds/<world_name>.sdf of gnssd_description; fail clearly for an unknown name."""
    path = os.path.join(get_package_share_directory('gnssd_description'),
                        'worlds', f'{world_name}.sdf')
    if not os.path.isfile(path):
        raise ValueError(f'Unknown world "{world_name}": {path} does not exist '
                         '(use world:=room or world:=warehouse)')
    return path


def gazebo_command(context):
    """Build the `gz sim` command: with or without the GUI, with a fixed seed."""
    world = world_file(LaunchConfiguration('world').perform(context))
    with_gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    seed = LaunchConfiguration('seed').perform(context)

    # -r: start running, not paused.
    # --seed: seeds Gazebo's random generator, which draws all sensor noise
    # (the gyro bias included), so the same seed gives the same sensor noise.
    command = ['gz', 'sim', '-r', '--seed', seed]
    if not with_gui:
        # Server only. --headless-rendering lets the gpu_lidar render
        # without a window (EGL on the GPU).
        command += ['-s', '--headless-rendering']
    command.append(world)

    # Started directly, without a shell, so that Ctrl+C / shutdown reaches gz itself.
    return [ExecuteProcess(cmd=command, output='screen', on_exit=Shutdown())]


def rviz_node(context, rviz_config):
    """
    Start RViz with a fixed frame that exists in this configuration.

    The config file uses `map` (published by slam_toolbox). Without SLAM there
    is no map frame, so RViz would drop every message; use `odom` instead.
    """
    with_slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    fixed_frame = 'map' if with_slam else 'odom'
    return [Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config, '-f', fixed_frame],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )]


def map_manager_node(context):
    """map_manager_node with the start-up mode, map and pose of the launch arguments."""
    if LaunchConfiguration('slam').perform(context).lower() != 'true':
        return []
    value = lambda name: LaunchConfiguration(name).perform(context)   # noqa: E731
    return [Node(
        package='gnssd_tools',
        executable='map_manager_node',
        parameters=[os.path.join(get_package_share_directory('gnssd_bringup'),
                                 'config', 'map_manager.yaml'), {
            'maps_dir': value('maps_dir'),
            'startup_mode': value('slam_mode'),
            'startup_map': value('slam_map'),
            'startup_pose': [float(value(name)) for name in ('slam_x', 'slam_y', 'slam_yaw')],
            'checkpoint_on_scan_lost': value('checkpoint_on_scan_lost').lower() == 'true',
        }],
        output='screen',
    )]


def generate_launch_description():
    description_dir = get_package_share_directory('gnssd_description')
    bringup_dir = get_package_share_directory('gnssd_bringup')

    robot_xacro = os.path.join(description_dir, 'urdf', 'robot.urdf.xacro')
    robot_params = os.path.join(description_dir, 'urdf', 'robot_params.xacro')
    bridge_config = os.path.join(bringup_dir, 'config', 'bridge.yaml')
    odom_noise_config = os.path.join(bringup_dir, 'config', 'odom_noise.yaml')
    # Each world has its own route: config/route_<world>.yaml.
    route_config = [os.path.join(bringup_dir, 'config', 'route_'),
                    LaunchConfiguration('world'), '.yaml']
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    sensor_monitor_config = os.path.join(bringup_dir, 'config', 'sensor_monitor.yaml')
    rviz_config = os.path.join(bringup_dir, 'rviz', 'sim.rviz')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value='room',
            description='World name: room or warehouse (worlds/<world>.sdf, '
                        'config/route_<world>.yaml)'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the Gazebo window (false = server only, for batch runs)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz with config rviz/sim.rviz'),
        DeclareLaunchArgument(
            'drive', default_value='false',
            description='Start route_driver_node (drives config/route_<world>.yaml)'),
        DeclareLaunchArgument(
            'ekf', default_value='true',
            description='Start the EKF base layer (robot_localization)'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Start the correction layer (map_manager_node + slam_toolbox; needs ekf)'),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            description='Start-up mode of the correction layer: mapping or localization'),
        DeclareLaunchArgument(
            'slam_map', default_value='',
            description='localization: saved map name in maps/ (FR-12)'),
        DeclareLaunchArgument('slam_x', default_value='0.0',
                              description='localization: initial pose in the map frame'),
        DeclareLaunchArgument('slam_y', default_value='0.0'),
        DeclareLaunchArgument('slam_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'checkpoint_on_scan_lost', default_value='false',
            description='Serialize the pose graph when the lidar is lost (safety checkpoint)'),
        DeclareLaunchArgument('maps_dir', default_value=os.path.join(repository_root(), 'maps'),
                              description='Folder of saved maps'),
        DeclareLaunchArgument(
            'sensor_monitor', default_value='true',
            description='Start sensor_monitor_node (system)'),
        DeclareLaunchArgument(
            'scan_dropout', default_value='[]',
            description='Lidar dropout windows, sim time: "[[t_start,t_end], ...]", '
                        't_end -1 = until the end'),
        DeclareLaunchArgument(
            'odom_noise', default_value='true',
            description='Add noise to wheel odometry (false = /odom equals /odom_raw)'),
        DeclareLaunchArgument(
            'seed', default_value=default_noise_seed(odom_noise_config),
            description='Seed of the odometry noise and of the Gazebo sensor noise '
                        '(default: value in odom_noise.yaml)'),
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

    imu_covariance = Node(
        package='gnssd_tools',
        executable='imu_covariance_node',
        parameters=[{
            'use_sim_time': True,
            # Same sigmas as the Gazebo IMU model.
            'gyro_noise_stddev': xacro_property(robot_params, 'imu_gyro_noise_stddev'),
            'accel_noise_stddev': xacro_property(robot_params, 'imu_accel_noise_stddev'),
        }],
        output='screen',
    )

    scan_gate = Node(
        package='gnssd_tools',
        executable='scan_gate_node',
        parameters=[{
            # Exception to "use_sim_time everywhere": the gate takes the sim
            # time from the scan header stamps and never reads the clock, so
            # it does not need to process the 1 kHz /clock (see its docstring).
            'use_sim_time': False,
            'dropout_windows': ParameterValue(LaunchConfiguration('scan_dropout'),
                                              value_type=str),
        }],
        output='screen',
    )

    sensor_monitor = Node(
        package='gnssd_tools',
        executable='sensor_monitor_node',
        # Steady clock instead of sim time (see the node's docstring): use_sim_time
        # would make it process the 1 kHz /clock.
        parameters=[sensor_monitor_config, {
            'use_sim_time': False,
            # Heading drift rate of the base layer for the correction consistency
            # check: the gyro bias of the sensor model (static and dynamic).
            'correction_check.drift_sigma': math.hypot(
                xacro_property(robot_params, 'imu_gyro_bias_stddev'),
                xacro_property(robot_params, 'imu_gyro_dynamic_bias_stddev')),
        }],
        condition=IfCondition(LaunchConfiguration('sensor_monitor')),
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('ekf')),
        output='screen',
    )

    map_manager = OpaqueFunction(function=map_manager_node)
    # The correction layer needs odom -> base_link from the EKF.
    start_slam_after_ekf = RegisterEventHandler(
        OnProcessStart(target_action=ekf, on_start=[map_manager]))

    route_driver = Node(
        package='gnssd_tools',
        executable='route_driver_node',
        parameters=[route_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('drive')),
        output='screen',
    )

    rviz = OpaqueFunction(function=rviz_node, args=[rviz_config])

    return LaunchDescription(arguments + [
        OpaqueFunction(function=warn_if_not_at_origin),
        OpaqueFunction(function=gazebo_command),
        robot_state_publisher,
        spawn_robot,
        bridge,
        odom_noise,
        imu_covariance,
        scan_gate,
        ekf,
        sensor_monitor,
        start_slam_after_ekf,
        route_driver,
        rviz,
    ])
