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
- `src/gnssd_description/`: robot URDF/xacro (`urdf/`), Gazebo worlds (`worlds/`) and sensor config (`config/`).
- `src/gnssd_bringup/`: launch files (`launch/`), parameter YAML (`config/`) and RViz configs (`rviz/`).
- `src/gnssd_description/urdf/robot_params.xacro`: all robot dimensions, masses and sensor/noise parameters.
- `src/gnssd_description/urdf/robot.urdf.xacro`: robot links, joints, Gazebo sensors (gpu_lidar, imu) and DiffDrive/JointStatePublisher plugins.
- `src/gnssd_description/worlds/room.sdf`: 10 x 8 m test room with a central island for a loop route.
- `src/gnssd_bringup/launch/sim.launch.py`: starts Gazebo (seeded), robot, bridge, odometry noise, IMU covariance, EKF, optional route driver and RViz.
- `src/gnssd_bringup/launch/record.launch.py`: records one run (rosbag2, sqlite3) plus run_info.yaml.
- `src/gnssd_bringup/launch/run.launch.py`: one complete experiment (sim + route driver + recording), stops when the route is done; refuses to run with uncommitted changes.
- `src/gnssd_bringup/config/ekf.yaml`: base layer EKF (robot_localization): fuses vx, vyaw from /odom and vyaw from /imu.
- `src/gnssd_bringup/config/route.yaml`: waypoints and controller parameters of the route driver.
- `src/gnssd_bringup/config/bridge.yaml`: topics bridged between Gazebo and ROS 2.
- `src/gnssd_bringup/config/odom_noise.yaml`: parameters of the odometry noise node.
- `src/gnssd_bringup/rviz/sim.rviz`: RViz view with robot model, TF and lidar scan.
- `src/gnssd_tools/`: Python (rclpy) project nodes and evaluation scripts.
- `src/gnssd_tools/gnssd_tools/odom_noise_node.py`: turns exact `/odom_raw` into drifting wheel odometry `/odom`.
- `src/gnssd_tools/gnssd_tools/route_driver_node.py`: pure-pursuit driver that repeats the same route in every run (steers on ground truth).
- `src/gnssd_tools/gnssd_tools/run_info_writer_node.py`: writes run_info.yaml (scenario, seed, noise parameters used, git commit, gz version).
- `src/gnssd_tools/gnssd_tools/imu_covariance_node.py`: republishes /imu_raw as /imu with covariances from robot_params.xacro.
- `src/gnssd_tools/gnssd_tools/geometry.py`: small 2D helpers (yaw from quaternion, angle wrapping).
- `src/gnssd_tools/scripts/compare_odom.py`: plots `/odom` vs `/odom_raw` from a bag and reports the drift.
- `src/gnssd_tools/scripts/run_batch.sh`: runs one scenario for a range of seeds and writes bags/index.csv.
- `src/gnssd_tools/scripts/tf_to_tum.py`: extracts a TF trajectory (e.g. odom -> base_link) from a bag as a TUM file for evo.
- `src/gnssd_tools/test/`: flake8 and pep257 style tests (`colcon test`).
- `docs/img/`: figures produced by the verification steps.
