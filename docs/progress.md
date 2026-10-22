# Progress log

Each step ends with a verification command; its output is pasted here.

## Step 0 — Host, toolchain and repository skeleton (2026-10-22)

### Host
```
$ lsb_release -ds; dpkg --print-architecture
Ubuntu 22.04.5 LTS
amd64
$ nvidia-smi
NVIDIA-SMI 535.309.01   Driver Version: 535.309.01   CUDA Version: 12.2
GPU 0: NVIDIA GeForce GTX 1650 Ti, 4096 MiB
$ ls /opt/ros
humble  noetic
$ glxinfo -B | grep renderer          # prime-select: on-demand
OpenGL renderer string: NVIDIA GeForce GTX 1650 Ti/PCIe/SSE2
```

### Installation notes
- Gazebo Classic 11.10 (Ubuntu repo) was installed and owned `/usr/bin/gz`.
  Followed gazebosim.org "Installing Gazebo11 side by side with new Gazebo":
  `ppa:openrobotics/gazebo11-gz-cli` -> gazebo11 11.14.0 (Classic CLI is now `gz11`;
  `gzserver`/`gzclient` unchanged), then Harmonic from packages.osrfoundation.org.
- Side effect: upgrading to the PPA gazebo11 removed `libgazebo-dev` and
  `ros-humble-gazebo-{dev,ros,plugins,ros-pkgs}` (Classic ROS bridge, not used here).
  The Harmonic install removed `gazebo11-gz-cli`, `gazebo` (transitional), `libignition-math6*`,
  `libignition-cmake-dev`.
- Package names: the official page lists only `ros-humble-ros-gzharmonic` (metapackage);
  it pulls `ros-humble-ros-gzharmonic-{bridge,sim,image,interfaces,sim-demos}` 0.244.12.
- evo 1.38.0 installed with `pip install --user "evo==1.38.0" "numpy<2"`.

Install commands (run with sudo):
```
add-apt-repository -y ppa:openrobotics/gazebo11-gz-cli && apt-get update && apt-get install gazebo11
curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/gazebo-stable.list
apt-get update && apt-get install gz-harmonic ros-humble-ros-gzharmonic
apt-get install ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-teleop-twist-keyboard ros-humble-rviz2 \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins ros-humble-rosbag2-storage-mcap \
  python3-colcon-common-extensions python3-rosdep
```

### Verification
```
$ gz sim --version
Gazebo Sim, version 8.15.0
Copyright (C) 2018 Open Source Robotics Foundation.
Released under the Apache 2.0 License.
```

```
$ timeout 30 gz sim -r shapes.sdf -v 3
[GUI] [Msg] Loading plugin [gz-rendering-ogre2]
[GUI] [Msg] Loaded plugin [MinimalScene] ...  (19 GUI plugins loaded, no errors)
libEGL warning: egl: failed to create dri2 screen      # warning only; GUI kept running
exit=124  (ran the full 30 s, closed by timeout)
Visual check: rendering works - confirmed by the Gazebo GUI screenshot of the Step 1 world
(docs/img/step1_gazebo.png, same GUI/ogre2 pipeline on NVIDIA, see addendum)
```

```
$ ros2 pkg list | grep -E 'ros_gz|robot_localization|slam_toolbox|nav2_bringup'
nav2_bringup
robot_localization
ros_gz
ros_gz_bridge
ros_gz_image
ros_gz_interfaces
ros_gz_sim
ros_gz_sim_demos
slam_toolbox
```

```
$ colcon build
Summary: 3 packages finished [1.29s]
  1 package had stderr output: gnssd_tools
# stderr = setuptools "byte-compiling is disabled, skipping." (benign)
```

```
$ evo_ape --help | head -5
/usr/lib/python3/dist-packages/scipy/__init__.py:146: UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy (detected version 1.26.4
usage: evo_ape [-h] {kitti,tum,euroc,bag,bag2,mcap} ...
$ pip show evo | grep Version
Version: 1.38.0
```
Note: the SciPy warning comes from a numpy 1.26.4 already present in `~/.local`
(pre-existing) next to Ubuntu's SciPy 1.8. evo runs; revisit if evo results look off.

```
$ git init -b main
Initialized empty Git repository in /home/pavich/Documents/diploma/.git/
```

### Step 0 addendum — environment and GPU check (2026-10-22)

```
$ printenv | grep ROS          # fresh interactive shell
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_LOCALHOST_ONLY=0
ROS_DISTRO=humble
AMENT_PREFIX_PATH=/home/pavich/ros2_ws/install/swarm_control:...:/opt/ros/humble
$ printenv | grep -c noetic
0
```
Only Humble, no Noetic paths. Note: `~/.bashrc` also sources the unrelated
workspace `~/ros2_ws/install` (swarm_control, simple_ros2_*); harmless, recorded in CLAUDE.md.

```
$ gz sim -r shapes.sdf &  nvidia-smi
|    0   N/A  N/A     29125      G   gz sim gui                                  239MiB |
# libraries mapped by the gz process: libGLX_nvidia, libEGL_nvidia, libnvidia-glcore, libnvidia-eglcore
```
Gazebo renders on the NVIDIA GPU with the default environment (`prime-select on-demand`,
X11 driven by NVIDIA), so no environment variables are added to the launch file.
Confirmed again in Step 1 with the gpu_lidar world: `gz sim server` is listed too.
Details and the fallback variables are in CLAUDE.md, section "GPU / rendering".

## Step 1 — Robot model, world, bridge, odometry noise node (2026-10-22)

### Robot parameters (`gnssd_description/urdf/robot_params.xacro`)

| Parameter | Value | Note |
|---|---|---|
| chassis (L x W x H) | 0.28 x 0.22 x 0.10 m | box |
| chassis mass | 2.0 kg | centre 0.04 m behind the axle |
| wheel radius | 0.035 m | same value in DiffDrive plugin |
| wheel width | 0.025 m | |
| wheel separation | 0.26 m | same value in DiffDrive plugin |
| wheel mass | 0.15 kg (each) | |
| caster | sphere r = 0.015 m, 0.05 kg, x = -0.11 m | friction 0 |
| total mass (Gazebo) | 2.53 kg | 2.23 kg lumped base + 2 wheels |
| footprint | ~0.28 x 0.285 m | |
| lidar mount | x = 0, z = 0.1125 m above base_link (0.1475 m above floor) | lidar_link |
| lidar | gpu_lidar, 360 deg, 360 samples (1 deg), 10 Hz, 0.12-12 m, range noise sigma 0.01 m | |
| IMU mount | x = 0, z = 0.02 m | imu_link |
| IMU | 100 Hz; gyro sigma 0.005 rad/s, bias mean 0.0005 rad/s (sd 0.0001); accel sigma 0.02 m/s^2, bias mean 0.01 m/s^2 (sd 0.001) | PLACEHOLDERS, to be replaced by datasheet values |
| DiffDrive odometry | 50 Hz, frame odom -> base_link, TF not bridged | |

base_link = middle of the wheel axle at axle height (0.035 m above the floor).

### Odometry noise model (`odom_noise_node`, `config/odom_noise.yaml`)

```
v_meas = v_true (1 + k_v) + N(0, sigma_v)      k_v = 0.02,  sigma_v = 0.02 m/s
w_meas = w_true (1 + k_w) + N(0, sigma_w)      k_w = 0.03,  sigma_w = 0.02 rad/s
x += v_meas cos(theta) dt;  y += v_meas sin(theta) dt;  theta += w_meas dt
```
Seed 42. Random noise is not added while the robot stands still
(|v| <= 0.001 m/s and |w| <= 0.001 rad/s), like real encoders reading zero.
Launch argument `odom_noise:=false` sets `enabled: false` (pass-through, still integrated).

### Design decisions
- No temporary static `odom -> base_link`: RViz uses `base_link` as fixed frame, so no
  TF other than robot_state_publisher's is needed in this step.
- Driving for the bag was done with a constant command
  (`v = 0.2 m/s, w = 0.2 rad/s`, circle of r = 1 m, ~2 loops in 60 s) instead of
  keyboard teleop, so the run is exactly repeatable. Teleop works the same way:
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

### Verification

```
$ ros2 launch gnssd_bringup sim.launch.py rviz:=true
[create-3] [INFO] [ros_gz_sim]: Requested creation of entity.
[odom_noise_node-5] [INFO] [odom_noise_node]: Odometry noise ON: linear scale error +0.0200,
    angular scale error +0.0300, noise stddev 0.02 m/s, 0.02 rad/s, seed 42
# no errors; only warnings: "libEGL ... dri2" and sdformat
# "XML Element[gz_frame_id] ... not defined in SDF. Copying" (expected: gz_frame_id is read by gz-sensors)
$ nvidia-smi
|    0   N/A  N/A     30170      G   /opt/ros/humble/lib/rviz2/rviz2               3MiB |
|    0   N/A  N/A     30221      G   gz sim server                                70MiB |
|    0   N/A  N/A     30222      G   gz sim gui                                  239MiB |
```
Gazebo screenshot: `docs/img/step1_gazebo.png` — room with central island, box, pillar, wall stub,
robot at the start corner; real-time factor 99.96 %.
RViz screenshot: `docs/img/step1_rviz.png` — Global Status Ok, robot model at the centre, /scan points outline the room walls and the corner of the central island (fixed frame base_link).

```
$ ros2 topic hz /scan    -> average rate: 9.984
$ ros2 topic hz /imu     -> average rate: 99.881
$ ros2 topic hz /odom_raw -> average rate: 49.921
$ ros2 topic hz /odom    -> average rate: 49.908
```
(Humble's `ros2 topic hz` takes one topic at a time; the four were measured in parallel.)

```
$ ros2 topic echo <topic> --once --field header.frame_id
/scan: lidar_link
/imu: imu_link
/odom_raw: odom   (child_frame_id: base_link)
$ ros2 topic echo /scan --once
angle_min: -3.1415927  angle_max: 3.1241393  angle_increment: 0.0174533  range_min: 0.12  range_max: 12.0
$ ros2 topic echo /odom --once
header: {stamp: {sec: 34, nanosec: 700000000}, frame_id: odom}
child_frame_id: base_link
pose: {pose: {position: {x: 1.9e-18, y: -2.9e-35, z: 0.0}, orientation: {z: -1.5e-17, w: 1.0}}
  covariance: [0.0004, ...]}
```

```
$ ros2 run tf2_tools view_frames        # while driving -> docs/img/step1_tf_frames.pdf
base_link -> caster_link, chassis_link, imu_link, lidar_link, left_wheel_link, right_wheel_link
(no odom frame at all)
$ ros2 topic info /tf -v ; ros2 topic info /tf_static -v
Publisher count: 1   Node name: robot_state_publisher      (both topics)
```

```
$ ros2 bag record -o bags/step1_noise_on /odom_raw /odom /cmd_vel /clock     # 60 s drive
$ python3 src/gnssd_tools/scripts/compare_odom.py bags/step1_noise_on -o docs/img/step1_compare_odom_noise_on.png
messages: /odom_raw 3520, /odom 3515
duration: 70.3 s
final drift: 0.414 m, max drift: 0.418 m

$ ros2 launch gnssd_bringup sim.launch.py odom_noise:=false
[odom_noise_node]: Odometry noise OFF: velocities pass through unchanged
$ python3 src/gnssd_tools/scripts/compare_odom.py bags/step1_noise_off -o docs/img/step1_compare_odom_noise_off.png
messages: /odom_raw 3193, /odom 3193
duration: 63.8 s
final drift: 0.001 m, max drift: 0.004 m
```
With noise the drift grows to ~0.41 m over ~14 m of path; without noise the
trajectories coincide. The remaining 1-4 mm come from integration: the node
integrates velocities with an Euler step, the DiffDrive plugin integrates wheel angles.
Plots: `docs/img/step1_compare_odom_noise_on.png`, `docs/img/step1_compare_odom_noise_off.png`.

```
$ colcon build --symlink-install
Summary: 3 packages finished [1.27s]
$ ament_flake8 src/gnssd_tools
6 files checked   (0 errors)
$ ament_pep257 src/gnssd_tools
No problems found
$ colcon test --packages-select gnssd_tools && colcon test-result --verbose
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```
