"""
Step 8 (NFR-9): Nav2 as an EXTERNAL consumer of the localization system.

Starts, in one recorded run:
  sim.launch.py      the system and the stand, the system in LOCALIZATION mode on a saved map
                     (map_manager_node startup_mode localization, start at the origin);
                     no route driver - Nav2 drives through /cmd_vel
  record.launch.py   rosbag2 + run_info.yaml etc. (scenario nav2_<world>)
  navigation_launch.py of nav2_bringup 1.1.20 with config/nav2_params.yaml:
                     planner, controller, smoother, behaviors, bt_navigator, waypoint follower,
                     velocity smoother - NO amcl, NO map_server (localization and the map come
                     from the system)
  rviz2              rviz/nav2_<world>.rviz (nav2_default_view.rviz with the robot model on,
                     no AMCL display, whole world in view), window title gnssd_nav2_rviz
                     (for the screenshots)
  nav2_goal_sender_node (stand)  sends config/nav2_goals_<world>.yaml one after another and
                     writes docs/step8/goals_<world>.yaml and the RViz screenshots
                     docs/step8/rviz_<world>_goal<k>.xwd; when it exits, everything shuts down.

Like run.launch.py it refuses to run with uncommitted changes (allow_dirty:=true overrides).

Usage:
  ros2 launch gnssd_bringup nav2_demo.launch.py world:=room
  ros2 launch gnssd_bringup nav2_demo.launch.py world:=warehouse
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MAP = {'room': 'final_room_s1', 'warehouse': 'final_wh_s1'}  # saved in Step 7
NAV2_START_DELAY = 15.0   # s (wall): slam_toolbox has loaded the map and publishes map -> odom


def repository_root():
    """Return the git repository that contains this launch file (via the symlink install)."""
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
    world = LaunchConfiguration('world').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    slam_map = LaunchConfiguration('map').perform(context) or DEFAULT_MAP[world]
    allow_dirty = LaunchConfiguration('allow_dirty').perform(context).lower() == 'true'
    repo = repository_root()

    changes = uncommitted_changes(repo)
    if changes and not allow_dirty:
        get_logger('nav2_demo.launch').error(
            'The repository has uncommitted changes; refusing to record a run that '
            'cannot be traced to a commit. Commit first, or pass allow_dirty:=true.\n' + changes)
        return [Shutdown(reason='uncommitted changes')]

    bringup = get_package_share_directory('gnssd_bringup')
    nav2 = get_package_share_directory('nav2_bringup')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={'world': world, 'seed': seed, 'gui': 'false', 'rviz': 'false',
                          'drive': 'false', 'slam_mode': 'localization',
                          'slam_map': slam_map}.items())
    recording = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'record.launch.py')),
        launch_arguments={'scenario': f'nav2_{world}', 'seed': seed, 'world': world,
                          'out_dir': LaunchConfiguration('out_dir').perform(context)
                          or os.path.join(repo, 'bags'),
                          'allow_dirty': str(allow_dirty).lower()}.items())
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'autostart': 'true',
                          'params_file': os.path.join(bringup, 'config', 'nav2_params.yaml')
                          }.items())
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(bringup, 'rviz', f'nav2_{world}.rviz'),
                           '-t', 'gnssd_nav2_rviz'],
                parameters=[{'use_sim_time': True}])
    out_dir = LaunchConfiguration('results_dir').perform(context) or \
        os.path.join(repo, 'docs', 'step8')
    goal_sender = Node(
        package='gnssd_tools', executable='nav2_goal_sender_node', output='screen',
        parameters=[os.path.join(bringup, 'config', f'nav2_goals_{world}.yaml'),
                    {'use_sim_time': True,
                     'output_file': os.path.join(out_dir, f'goals_{world}.yaml'),
                     'screenshot_prefix': os.path.join(out_dir, f'rviz_{world}')}])
    stop = RegisterEventHandler(OnProcessExit(target_action=goal_sender, on_exit=[
        LogInfo(msg='All goals done, shutting the demonstration down'),
        Shutdown(reason='goals done')]))

    return [simulation, recording,
            TimerAction(period=NAV2_START_DELAY, actions=[navigation, rviz, goal_sender]),
            stop]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='room', description='room or warehouse'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('map', default_value='',
                              description='saved map (default: final_room_s1 / final_wh_s1)'),
        DeclareLaunchArgument('allow_dirty', default_value='false'),
        DeclareLaunchArgument('out_dir', default_value='',
                              description='bags folder (default: bags/)'),
        DeclareLaunchArgument('results_dir', default_value='',
                              description='goal table + screenshots (default: docs/step8/)'),
        OpaqueFunction(function=launch_setup),
    ])
