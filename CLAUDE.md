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
| `/scan_raw` | sensor_msgs/LaserScan | simulator lidar (bridge) |
| `/scan` | sensor_msgs/LaserScan | scan_gate_node (stand): /scan_raw relayed unchanged, dropped in dropout windows |
| `/imu_raw` | sensor_msgs/Imu | simulator IMU, covariances all zero |
| `/imu` | sensor_msgs/Imu | imu_covariance_node: /imu_raw + covariances from robot_params.xacro (EKF input) |
| `/odom_raw` | nav_msgs/Odometry | directly from the simulator |
| `/odom` | nav_msgs/Odometry | odometry noise node (EKF input) |
| `/odometry/filtered` | nav_msgs/Odometry | EKF output (base layer) |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox |
| `/pose` | geometry_msgs/PoseWithCovarianceStamped | slam_toolbox (informational; the corrected estimate is defined by TF map -> base_link) |
| `/ground_truth` | nav_msgs/Odometry | simulator, gz OdometryPublisher, frame `world` (evaluation + route driver only) |
| `/cmd_vel` | geometry_msgs/Twist | teleop / Nav2 |
| `/joint_states` | sensor_msgs/JointState | simulator (wheel joints) |
| `/clock` | rosgraph_msgs/Clock | simulator |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | sensor_monitor_node (sensor states, correction consistency), ekf_filter_node |
| `/correction_valid` | std_msgs/Bool (transient local) | sensor_monitor_node: false once map->odom made an implausible heading jump (latched; reset: `ros2 service call /sensor_monitor_node/reset_correction_check std_srvs/srv/Trigger`) |
| `/map_manager/events` | std_msgs/String (transient local) | map_manager_node: one line per mode switch / map save / map load / checkpoint |

## Correction layer control (map_manager_node, Step 6)
- map_manager_node starts slam_toolbox as a child process (`slam_mapping.launch.py`
  or `slam_localization.launch.py`) and is the only thing that starts/stops it.
  Mode switch = stop (SIGINT -> SIGTERM -> SIGKILL to the process group) + start.
- Maps live in `maps/<name>.{posegraph,data,pgm,yaml}` (run outputs, ignored by git).
- Services: `/map_manager_node/save_map` (gnssd_interfaces/SaveMap, mapping only),
  `/map_manager_node/set_mode` (SetMode), `/map_manager_node/status` (GetStatus).
  set_mode: mapping (new map), mapping + map_name (continue that graph from the given
  or base pose, START_AT_GIVEN_POSE), localization + map_name.
- map_manager reads /tf and /odometry/filtered only at a switch with use_base_pose
  (temporary subscription); while INVALID the reference map->odom comes from
  sensor_monitor's diagnostics (last_valid_map_to_odom_*). sensor_monitor subscribes
  the sensor topics raw (arrival time only). Keep both nodes free of high-rate
  Python subscriptions they do not need (Step 6b: ~20 % -> ~1 % / ~24 % -> ~14 %).
- The occupancy grid is saved with nav2 `map_saver_cli` run by map_manager (timeout
  in map_manager.yaml): slam_toolbox's own save_map uses a 2 s timeout that fails here.
- slam_toolbox in localization mode refuses serialize_map but still answers
  result=0 (SUCCESS): check the files, never the result code.

## Evaluation conventions
- ATE is computed WITHOUT alignment: `evo_ape bag2 <bag> /ground_truth <est>`
  with no `-a`, `--align_origin` or `-s`.
- Why: alignment fits the estimate onto the reference and hides part of the
  drift that the thesis measures. Without alignment the estimate and the
  reference must share one frame from t = 0.
- Therefore every mapping run starts at the world origin (x = 0, y = 0,
  yaw = 0). room.sdf is built around that start point, the spawn defaults
  are 0, and sim.launch.py warns if they are changed. odom and map then
  start at the world origin, and /ground_truth (gz OdometryPublisher, world
  pose) is directly comparable.
- /ground_truth is used only for evaluation and by the route driver (stand),
  never by the estimators.
- Before every recorded run `ros2 node list` must be empty. A leftover node
  from an earlier session (e.g. a second parameter_bridge or odom_noise_node)
  publishes into the same topics and silently corrupts the bag (happened in
  Step 2).
- Corrected estimate = TF map -> base_link (slam_toolbox map -> odom composed with EKF
  odom -> base_link), extracted with `src/gnssd_tools/scripts/tf_to_tum.py <bag> map base_link`
  (evo in the venv cannot read /tf). Base layer = /odometry/filtered.
- Worlds: `world:=room|warehouse` selects `worlds/<world>.sdf` and
  `config/route_<world>.yaml`. Scenario names: room_loop, room_long, warehouse_loop.
  The system configuration (ekf.yaml, slam_toolbox.yaml, noise models) is the same in
  every world, so results are comparable; never tune per world.
- One run = one bag in `bags/<scenario>_seed<seed>_<timestamp>/` with
  `run_info.yaml`; started with `ros2 launch gnssd_bringup run.launch.py`.

## GPU / rendering
- Host: hybrid laptop, `prime-select on-demand`, X11 session driven by the
  NVIDIA GTX 1650 Ti. Gazebo already renders on NVIDIA without any extra
  environment variables: `nvidia-smi` lists `gz sim server` (gpu_lidar) and
  `gz sim gui` as G-type processes (checked in Step 0/1, see docs/progress.md).
  Headless runs (`gui:=false`, `gz sim -s --headless-rendering`) also render
  on NVIDIA through EGL (checked in Step 2).
  Therefore sim.launch.py sets NO `__NV_PRIME_RENDER_OFFLOAD` /
  `__GLX_VENDOR_LIBRARY_NAME` / `__EGL_VENDOR_LIBRARY_FILENAMES`.
- If `nvidia-smi` ever stops listing the gz processes (e.g. after switching to
  Wayland or `prime-select intel`), add those three variables
  (`1`, `nvidia`, `/usr/share/glvnd/egl_vendor.d/10_nvidia.json`) to
  sim.launch.py with `SetEnvironmentVariable`.
- `libEGL warning: egl: failed to create dri2 screen` is harmless (Mesa EGL is
  probed first, then NVIDIA is used).

## Environment
- evo 1.38.0 lives in its own venv: use `~/.venvs/evo/bin/evo_ape`,
  `~/.venvs/evo/bin/evo_traj` etc. The `pip --user` evo in ~/.local cannot
  plot: Ubuntu's python3-matplotlib (needed by rqt_plot) ships a regular
  `mpl_toolkits` package that shadows the one of pip's matplotlib 3.10.
  The venv does not need ROS (evo reads bags through `rosbags`).
- NEVER run `sudo apt autoremove`: only `ros-humble-ros-base` is marked as
  manually installed, so apt lists ~219 ROS packages (ros2topic, ros2run,
  rqt, tf2_tools, ...) as removable. Installed ROS packages can come from
  different repository snapshots: robot_localization 3.5.4 needed
  `ros-humble-diagnostic-updater` 4.0.7 (`apt-get install --only-upgrade`).
  If a node dies with "error while loading shared libraries", check for
  such a version mismatch first.
- `~/.bashrc` sources `/opt/ros/humble` and also the user's other workspace
  `~/ros2_ws/install` (unrelated packages). No Noetic paths are present.

## Nav2 (Step 8, external consumer)
- Nav2 is NOT part of the system: `nav2_demo.launch.py` starts nav2_bringup's
  navigation_launch.py only (no amcl, no map_server) with config/nav2_params.yaml.
- Nav2 consumes the system outputs unchanged: /map, TF map -> odom (slam_toolbox),
  TF odom -> base_link and /odometry/filtered (EKF), /scan; it publishes /cmd_vel.
- Never point Nav2 at /odom: here that is the stand's noisy wheel odometry.

## System vs stand
- SYSTEM (what would run on the real robot; measured by resource_logger):
  ekf_filter_node, slam_toolbox (async_slam_toolbox_node or
  localization_slam_toolbox_node), sensor_monitor_node, map_manager_node.
- STAND (simulation and evaluation only): gz sim, parameter_bridge,
  odom_noise_node, imu_covariance_node, scan_gate_node, route_driver_node,
  recorder, run_info_writer_node, loop_closure_monitor_node,
  resource_logger_node, node_census_node, scripted_operator_node,
  nav2_goal_sender_node, RViz.
- Every own node starts through gnssd_tools/node_runner.py (rclpy SIGINT
  handler off -> clean shutdown, no Traceback).

## Conventions
- `use_sim_time: true` everywhere (every node, every launch file). Exceptions
  (documented in the nodes): scan_gate_node takes sim time from the scan header
  stamps; sensor_monitor_node runs on the steady clock (as on the robot) and checks
  RTF ~ 1 from one /clock sample every 10 s; map_manager_node needs no ROS clock
  (it compares message stamps only).
- Gazebo publishes /clock at 1 kHz; every node with use_sim_time handles all of it:
  ~43 % of one core for an rclpy node, ~5.7 % for an rclcpp node (measured in
  Steps 5/5b). Simulation artifact (no /clock on the real robot): state it when
  reporting CPU of ekf_filter_node / slam_toolbox (C++, use_sim_time).
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
