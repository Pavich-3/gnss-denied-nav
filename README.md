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
- `src/gnssd_bringup/launch/sim.launch.py`: starts Gazebo, robot, bridge, odometry noise node and optional RViz.
- `src/gnssd_bringup/config/bridge.yaml`: topics bridged between Gazebo and ROS 2.
- `src/gnssd_bringup/config/odom_noise.yaml`: parameters of the odometry noise node.
- `src/gnssd_bringup/rviz/sim.rviz`: RViz view with robot model, TF and lidar scan.
- `src/gnssd_tools/`: Python (rclpy) project nodes and evaluation scripts.
- `src/gnssd_tools/gnssd_tools/odom_noise_node.py`: turns exact `/odom_raw` into drifting wheel odometry `/odom`.
- `src/gnssd_tools/scripts/compare_odom.py`: plots `/odom` vs `/odom_raw` from a bag and reports the drift.
- `src/gnssd_tools/test/`: flake8 and pep257 style tests (`colcon test`).
- `docs/img/`: figures produced by the verification steps.
