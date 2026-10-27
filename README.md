# gnss-denied-nav

GNSS-denied localization and mapping subsystem for a differential-drive mobile
robot (master's thesis). It fuses wheel odometry and IMU in a robot_localization
EKF and corrects drift with slam_toolbox (2D lidar, async mode). Project nodes
monitor sensor health and manage maps. Runs are evaluated with rosbag2 + evo.
Targets Ubuntu 22.04, ROS 2 Humble and Gazebo Harmonic (gz-sim 8, `ros-humble-ros-gzharmonic`).
Build with `source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash`.

## Files
- `CLAUDE.md`: project conventions (stack, frames, topics, workflow rules).
- `LICENSE`: Apache-2.0 license text.
- `.gitignore`: excludes colcon output, bags, generated maps and evaluation results.
- `docs/progress.md`: per-step log with pasted verification output.
- `docs/step8/`: Step 8 goal tables and RViz screenshots of the Nav2 demonstration.
- `src/gnssd_description/`: robot URDF/xacro (`urdf/`), Gazebo worlds (`worlds/`) and sensor config (`config/`).
- `src/gnssd_interfaces/`: service definitions of map_manager_node (SaveMap, SetMode, GetStatus).
- `src/gnssd_bringup/`: launch files (`launch/`), parameter YAML (`config/`) and RViz configs (`rviz/`: sim.rviz, nav2_room.rviz, nav2_warehouse.rviz).
- `src/gnssd_description/urdf/robot_params.xacro`: all robot dimensions, masses and sensor/noise parameters.
- `src/gnssd_description/urdf/robot.urdf.xacro`: robot links, joints, Gazebo sensors (gpu_lidar, imu) and DiffDrive/JointStatePublisher plugins.
- `src/gnssd_description/worlds/room.sdf`: 10 x 8 m test room with a central island for a loop route.
- `src/gnssd_description/worlds/warehouse.sdf`: 30 x 20 m warehouse with four rack rows and a long repetitive aisle.
- `src/gnssd_bringup/launch/sim.launch.py`: starts Gazebo (seeded), robot, bridge, odometry noise, IMU covariance, EKF, sensor monitor, map_manager (which starts slam_toolbox), optional route driver and RViz.
- `src/gnssd_bringup/launch/nav2_demo.launch.py`: Step 8: system in localization mode + recording + Nav2 (no AMCL / map_server) + RViz + goal sender.
- `src/gnssd_bringup/launch/slam_mapping.launch.py`: slam_toolbox online async mapping, new or continuing a saved graph (started by map_manager_node).
- `src/gnssd_bringup/launch/slam_localization.launch.py`: slam_toolbox localization on a saved pose graph at a given initial pose (started by map_manager_node).
- `src/gnssd_bringup/launch/record.launch.py`: records one run (rosbag2, sqlite3) plus run_info.yaml, loop_closures.yaml, resources.csv.
- `src/gnssd_bringup/launch/run.launch.py`: one complete experiment (sim + route driver + recording), stops when the route is done; refuses to run with uncommitted changes.
- `src/gnssd_bringup/config/ekf.yaml`: base layer EKF (robot_localization): fuses vx, vy = 0, vyaw from /odom and vyaw from /imu; process noise derived in Step 6b.
- `src/gnssd_bringup/config/slam_toolbox.yaml`: correction layer (slam_toolbox online async mapping); package defaults + 3 marked changes.
- `src/gnssd_bringup/config/slam_toolbox_localization.yaml`: slam_toolbox localization mode; package defaults + marked changes.
- `src/gnssd_bringup/config/map_manager.yaml`: timeouts of map_manager_node (service calls, grid saving, stopping slam_toolbox).
- `src/gnssd_bringup/config/nav2_params.yaml`: Nav2 (external consumer, Step 8): nav2_bringup defaults + 4 marked changes (odometry topic, footprint).
- `src/gnssd_bringup/config/nav2_goals_room.yaml`, `nav2_goals_warehouse.yaml`: the 3 navigation goals per world for the Step 8 demonstration.
- `src/gnssd_bringup/config/route_room_fast.yaml`: room loop at 0.5 m/s (speed experiment, experiments-v1.1).
- `src/gnssd_bringup/config/route_room_east.yaml`: room loop starting at (7, 2) heading north (localization from a non-origin start).
- `src/gnssd_bringup/config/resource_logger.yaml`: list of system processes whose CPU/RSS is logged.
- `src/gnssd_bringup/config/sensor_monitor.yaml`: sensors, timeouts and rates watched by sensor_monitor_node.
- `src/gnssd_bringup/config/route_room.yaml`: route (waypoints + controller parameters) for world:=room.
- `src/gnssd_bringup/config/route_warehouse.yaml`: one 178.6 m lap through the warehouse for world:=warehouse.
- `src/gnssd_bringup/config/bridge.yaml`: topics bridged between Gazebo and ROS 2.
- `src/gnssd_bringup/config/odom_noise.yaml`: parameters of the odometry noise node.
- `src/gnssd_bringup/rviz/sim.rviz`: RViz view with robot model, TF and lidar scan.
- `src/gnssd_tools/`: Python (rclpy) project nodes and evaluation scripts.
- `src/gnssd_tools/gnssd_tools/odom_noise_node.py`: turns exact `/odom_raw` into drifting wheel odometry `/odom`.
- `src/gnssd_tools/gnssd_tools/route_driver_node.py`: pure-pursuit driver that repeats the same route in every run (steers on ground truth).
- `src/gnssd_tools/gnssd_tools/run_info_writer_node.py`: writes run_info.yaml (scenario, seed, noise parameters used, git commit, gz version).
- `src/gnssd_tools/gnssd_tools/imu_covariance_node.py`: republishes /imu_raw as /imu with covariances from robot_params.xacro.
- `src/gnssd_tools/gnssd_tools/loop_closure_monitor_node.py`: counts slam_toolbox loop closures (moved graph vertices) -> loop_closures.yaml.
- `src/gnssd_tools/gnssd_tools/resource_logger_node.py`: logs CPU % and RSS of the system processes at 1 Hz -> resources.csv.
- `src/gnssd_tools/gnssd_tools/sensor_monitor_node.py`: system node; reports lost sensor streams and implausible corrections on /diagnostics, publishes /correction_valid.
- `src/gnssd_tools/gnssd_tools/correction_consistency.py`: ROS-free check that flags implausible map->odom heading jumps (used by sensor_monitor_node).
- `src/gnssd_tools/gnssd_tools/sensor_health.py`: ROS-free sensor state machine (WAITING/OK/LOST) used by sensor_monitor_node.
- `src/gnssd_tools/gnssd_tools/map_manager_node.py`: system node; selects mapping/localization, saves and loads maps, supervises slam_toolbox.
- `src/gnssd_tools/gnssd_tools/map_manager_core.py`: ROS-free parts of map_manager_node (map files, command checks, child process stop).
- `src/gnssd_tools/gnssd_tools/scripted_operator_node.py`: stand node; plays the operator who switches to localization or mapping on the last checkpoint after an alarm.
- `src/gnssd_tools/gnssd_tools/nav2_goal_sender_node.py`: stand node; sends NavigateToPose goals in sequence, records result, time, path, errors, RViz screenshots.
- `src/gnssd_tools/gnssd_tools/scan_gate_node.py`: stand node; relays /scan_raw to /scan, drops scans in dropout windows.
- `src/gnssd_tools/gnssd_tools/node_census_node.py`: writes the list of running nodes every 5 s -> nodes.csv.
- `src/gnssd_tools/gnssd_tools/node_runner.py`: common start/clean-shutdown of all own nodes.
- `src/gnssd_tools/gnssd_tools/geometry.py`: small 2D helpers (yaw from quaternion, angle wrapping).
- `src/gnssd_tools/scripts/compare_odom.py`: plots `/odom` vs `/odom_raw` from a bag and reports the drift.
- `src/gnssd_tools/scripts/run_batch.sh`: runs one scenario for a range of seeds and writes bags/index.csv; skips runs already complete at the current commit.
- `src/gnssd_tools/scripts/run_final_series.sh`: the whole Step 7 series (all scenarios and seeds), resumable, keeps the laptop awake.
- `src/gnssd_tools/scripts/check_corrections.py`: replays the correction consistency check on recorded bags (offline validation).
- `src/gnssd_tools/scripts/dropout_analysis.py`: errors before/during/after a lidar dropout, re-convergence time, plot.
- `src/gnssd_tools/scripts/error_vs_distance.py`: position error vs distance from the start and vs travelled distance (TUM inputs).
- `src/gnssd_tools/scripts/ekf_consistency.py`: compares the EKF covariance with its real errors (fraction inside 2-sigma, sigmas at the end).
- `src/gnssd_tools/scripts/final_extract.py`: Step 7: reads every bag of the final series once into compact arrays (results/final_work/).
- `src/gnssd_tools/scripts/final_metrics.py`: Step 7: one row of metrics per run -> results/final/runs.csv.
- `src/gnssd_tools/scripts/final_summary.py`: Step 7: statistics, NFR evaluation and tables -> results/final/summary.md.
- `src/gnssd_tools/scripts/final_figures.py`: Step 7: the 10 thesis figures (Ukrainian labels) -> results/final/fig/.
- `src/gnssd_tools/scripts/loop_closure_error.py`: NFR-3 loop-closure error (difference of estimated and true displacement between two visits).
- `results/final/`: Step 7 results: runs.csv, summary.md, fig/ (PNG 300 dpi + PDF).
- `src/gnssd_tools/scripts/xwd_to_png.py`: converts xwd window dumps (RViz screenshots) to PNG.
- `src/gnssd_tools/scripts/estimate_gyro_bias.py`: estimates the per-run gyro z bias from a bag (/imu_raw vs /ground_truth).
- `src/gnssd_tools/scripts/map_to_png.py`: saves the last /map of a bag as a PNG (optionally with the ground truth).
- `src/gnssd_tools/scripts/summarize_resources.py`: mean/max CPU and RSS per process and system from resources.csv.
- `src/gnssd_tools/scripts/tf_to_tum.py`: extracts a TF trajectory (e.g. odom -> base_link) from a bag as a TUM file for evo.
- `src/gnssd_tools/test/`: flake8 and pep257 style tests and unit tests of the sensor state machine, the correction check and map_manager_core (`colcon test`).
- `docs/img/`: figures produced by the verification steps.
