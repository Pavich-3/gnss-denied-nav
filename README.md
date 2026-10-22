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
- `src/gnssd_tools/`: Python (rclpy) project nodes and evaluation scripts.
