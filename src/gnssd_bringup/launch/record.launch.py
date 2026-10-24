"""
Record one experiment run with rosbag2.

Usage:
  ros2 launch gnssd_bringup record.launch.py scenario:=loop seed:=1
  ros2 launch gnssd_bringup record.launch.py scenario:=loop seed:=1 out_dir:=/tmp/bags

Creates  <out_dir>/<scenario>_seed<seed>_<YYYYmmdd_HHMMSS>/  containing
  - the rosbag2 recording (sqlite3 storage, read by `evo_* bag2`);
  - run_info.yaml written by run_info_writer_node (scenario, seed, noise
    parameters actually used, git commit, Gazebo version);
  - loop_closures.yaml written by loop_closure_monitor_node;
  - resources.csv written by resource_logger_node (CPU / RSS of the system nodes).

The seed given here is only a label for the bag. The seed that drives the
noise is passed to sim.launch.py; run.launch.py feeds both from one argument.
"""

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Everything needed to evaluate a run. Topics that appear later (e.g. /map
# once slam_toolbox has processed scans) are recorded as soon as they exist.
TOPICS = [
    '/clock', '/tf', '/tf_static',
    '/scan', '/imu_raw', '/imu', '/joint_states',
    '/odom_raw', '/odom', '/cmd_vel',
    '/ground_truth',
    '/odometry/filtered',
    '/map', '/pose', '/slam_toolbox/graph_visualization',
]

# sqlite3 is the rosbag2 default in Humble and is read by evo's `bag2` command.
STORAGE = 'sqlite3'


def repository_root():
    """
    Return the repository folder (the one that contains src/).

    With `colcon build --symlink-install` this file is a symlink into
    src/gnssd_bringup/launch/, so following the link leads back to the
    repository. Otherwise fall back to the current working directory.
    """
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    package_dir = os.path.dirname(launch_dir)
    src_dir = os.path.dirname(package_dir)
    if os.path.basename(src_dir) == 'src':
        return os.path.dirname(src_dir)
    return os.getcwd()


def start_recording(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    out_dir = LaunchConfiguration('out_dir').perform(context)
    allow_dirty = LaunchConfiguration('allow_dirty').perform(context).lower() == 'true'

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(out_dir, f'{scenario}_seed{seed}_{timestamp}')
    os.makedirs(out_dir, exist_ok=True)

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--storage', STORAGE, '-o', bag_path] + TOPICS,
        output='screen',
    )

    run_info_writer = Node(
        package='gnssd_tools',
        executable='run_info_writer_node',
        parameters=[{
            'output_file': os.path.join(bag_path, 'run_info.yaml'),
            'scenario': scenario,
            'seed': int(seed),
            'repo_dir': repository_root(),
            'allow_dirty': allow_dirty,
        }],
        output='screen',
    )

    loop_closure_monitor = Node(
        package='gnssd_tools',
        executable='loop_closure_monitor_node',
        parameters=[{
            'output_file': os.path.join(bag_path, 'loop_closures.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    resource_logger = Node(
        package='gnssd_tools',
        executable='resource_logger_node',
        parameters=[
            os.path.join(get_package_share_directory('gnssd_bringup'),
                         'config', 'resource_logger.yaml'),
            {'output_file': os.path.join(bag_path, 'resources.csv'), 'use_sim_time': True},
        ],
        output='screen',
    )

    return [LogInfo(msg=f'Recording to {bag_path}'), recorder, run_info_writer,
            loop_closure_monitor, resource_logger]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='test',
                              description='Name of the experiment scenario'),
        DeclareLaunchArgument('seed', default_value='0',
                              description='Noise seed of this run (used in the bag name)'),
        DeclareLaunchArgument('out_dir', default_value=os.path.join(repository_root(), 'bags'),
                              description='Folder where bag folders are created'),
        DeclareLaunchArgument('allow_dirty', default_value='false',
                              description='Stored in run_info.yaml (set by run.launch.py)'),
        OpaqueFunction(function=start_recording),
    ])
