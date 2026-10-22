# CLAUDE.md — gnss-denied-nav

GNSS-denied localization and mapping subsystem for a differential-drive robot
(master's thesis). Base layer: robot_localization EKF (odometry + IMU).
Correction layer: slam_toolbox (2D lidar, async mode). Own nodes:
`sensor_monitor_node`, `map_manager_node` (and the odometry noise node).
Evaluation: rosbag2 + evo. Nav2 is an external consumer, not part of the system.

## Stack (fixed)
- Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic (gz-sim 8), ros_gz humble branch
  (`ros-humble-ros-gzharmonic*` from packages.osrfoundation.org).
- Never downgrade to Gazebo Classic or Fortress. Do not use `gazebo_ros`,
  `ros_ign*` or `ros-humble-ros-gz*` (Fortress) packages.

## Frames (REP 105)
- `map -> odom -> base_link`.
- `odom -> base_link` is published ONLY by the EKF node (robot_localization).
- `map -> odom` is published ONLY by slam_toolbox.
- No other node publishes TF for these frames. Gazebo plugins must not either
  (e.g. DiffDrive / OdometryPublisher TF output disabled or not bridged).

## Topics
| Topic | Type | Source |
|---|---|---|
| `/scan` | sensor_msgs/LaserScan | simulator lidar |
| `/imu` | sensor_msgs/Imu | simulator IMU |
| `/odom_raw` | nav_msgs/Odometry | directly from the simulator |
| `/odom` | nav_msgs/Odometry | odometry noise node (EKF input) |
| `/odometry/filtered` | nav_msgs/Odometry | EKF output |
| `/ground_truth` | nav_msgs/Odometry | simulator (evaluation only) |
| `/cmd_vel` | geometry_msgs/Twist | teleop / Nav2 |
| `/joint_states` | sensor_msgs/JointState | simulator (wheel joints) |
| `/clock` | rosgraph_msgs/Clock | simulator |

## GPU / rendering
- Host: hybrid laptop, `prime-select on-demand`, X11 session driven by the
  NVIDIA GTX 1650 Ti. Gazebo already renders on NVIDIA without any extra
  environment variables: `nvidia-smi` lists `gz sim server` (gpu_lidar) and
  `gz sim gui` as G-type processes (checked in Step 0/1, see docs/progress.md).
  Therefore sim.launch.py sets NO `__NV_PRIME_RENDER_OFFLOAD` /
  `__GLX_VENDOR_LIBRARY_NAME` / `__EGL_VENDOR_LIBRARY_FILENAMES`.
- If `nvidia-smi` ever stops listing the gz processes (e.g. after switching to
  Wayland or `prime-select intel`), add those three variables
  (`1`, `nvidia`, `/usr/share/glvnd/egl_vendor.d/10_nvidia.json`) to
  sim.launch.py with `SetEnvironmentVariable`.
- `libEGL warning: egl: failed to create dri2 screen` is harmless (Mesa EGL is
  probed first, then NVIDIA is used).

## Environment
- `~/.bashrc` sources `/opt/ros/humble` and also the user's other workspace
  `~/ros2_ws/install` (unrelated packages). No Noetic paths are present.

## Conventions
- `use_sim_time: true` everywhere (every node, every launch file).
- Own nodes are Python (rclpy), in `gnssd_tools`.
- All tunable values go into YAML under `src/gnssd_bringup/config/`; never
  hard-code them.
- Every step ends with a verification command whose output is pasted into
  that step's section of `docs/progress.md`.
- Keep changes minimal. Every file gets a one-line purpose entry in README.md.
- Ask before any destructive or system-wide action beyond package installs.
- Code must be readable for the thesis author: clear names, comments that
  explain why, module docstrings stating purpose, inputs and outputs.
- `gnssd_tools` must pass `ament_flake8` and `ament_pep257` (`colcon test`).
- The user makes git commits and pushes; prepare the commands, do not commit.

## Build
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
