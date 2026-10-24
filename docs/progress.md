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

## Step 2 — Origin start, ground truth, route driver, rosbag2 profile (2026-10-22)

### What changed
- `room.sdf` translated by (+3.5, +2.5): the start point is now the world origin, in the
  south corridor, facing +x. Room x -1.5..8.5, y -1.5..6.5; island centre (3.5, 2.5).
  Grey materials on walls and obstacles. Spawn defaults 0/0/0; sim.launch.py logs a
  WARNING if they are changed (ATE without alignment needs the origin start).
- `/ground_truth`: gz `OdometryPublisher` on the robot, 50 Hz, dimensions 2, frame `world`
  -> `base_link`, no noise, its TF topic not bridged.
- `route_driver_node` (pure pursuit on /ground_truth, `config/route.yaml`): 8 waypoints around
  the island, 2 laps, 0.2 m/s, look-ahead 0.5 m; stops, logs and exits at the end.
- `record.launch.py` (rosbag2 sqlite3 + `run_info.yaml`), `run.launch.py` (sim + driver +
  recording, shuts down when the driver exits). `sim.launch.py`: new args `gui`, `drive`, `seed`.
- `odom_noise.yaml`: `random_scale: true` (scale errors drawn from +/-0.02 and +/-0.03 per seed).
  The node exposes the drawn values as read-only parameters `applied_*_scale_error`
  (0 when the noise is disabled); `run_info_writer_node` stores them.
- **Wheel collision changed from cylinder to sphere** (see "Problem found" below).
- evo moved to its own venv `~/.venvs/evo` (same version 1.38.0), see below.

### OdometryPublisher: world pose or relative to the start?
gz-sim 8 source (`src/systems/odometry_publisher/OdometryPublisher.cc`, branch gz-sim8):
`const math::Pose3d rawPose = worldPose(this->model.Entity(), _ecm);`
`math::Pose3d pose = rawPose * this->offset;` -> the published pose is the model's WORLD
pose (offset = xyz_offset/rpy_offset = 0); no initial pose is subtracted. Twist is a
finite difference smoothed with a rolling mean of 10 samples, in the body frame.
With the start at the origin both interpretations coincide:
```
$ ros2 topic echo /ground_truth --once      (t = 13.0 s, robot standing at the start)
frame_id: world   child_frame_id: base_link
position: x: 2.3e-10  y: -2.8e-19      orientation: z: -1.9e-19  w: 1.0
$ ros2 topic echo /odom_raw --once
frame_id: odom    child_frame_id: base_link
position: x: 7.6e-19  y: -4.0e-36      orientation: z: -5.7e-18  w: 1.0
# first samples in the recorded bag (t = 0.020 s): both (0.0000, 0.0000)
```

### Problem found and fixed: cylinder wheel contact
The first noise-off run gave ATE(/odom_raw) = 2.16 m RMSE instead of centimetres.
A spin/line test (command for 10 s, compare with /ground_truth) showed that the physics
did not follow the DiffDrive kinematics:

| test | command | true motion (cylinder collision) | true motion (sphere collision) | /odom_raw |
|---|---|---|---|---|
| spin in place, w = 0.5 rad/s | 5.000 rad | 5.529 rad (+10.6 %) | 5.000 rad | 5.000 rad |
| straight, v = 0.2 m/s | 2.000 m | 1.936 m (-3.2 %) | 1.998 m (-0.1 %) | 2.000 m |

A cylinder touches the floor along its rim and the contact ended up near the inner edge
(effective track ~0.23 m instead of 0.26 m). The wheel collision is now a sphere of the
wheel radius (visual stays a cylinder): a single contact point in the wheel mid-plane.
Step 1 results (/odom vs /odom_raw) are not affected: they compare two signals computed
from the same wheel velocities.

A second problem in the same session: two orphaned nodes from a manual test
(parameter_bridge, odom_noise_node) kept publishing into /odom_raw, /odom, ... and
corrupted three bags. They were deleted; the batch script now refuses to start while
`ros2 node list --no-daemon --spin-time 3` is not empty (rule added to CLAUDE.md).

### Storage format
Test bag with nav_msgs/Odometry recorded with `--storage sqlite3` and `--storage mcap`:
```
$ evo_traj bag2 bag_sqlite3 /test_odom   -> 58 poses        (works)
$ evo_traj bag2 bag_mcap /test_odom      -> 60 poses        (works)
$ evo_traj mcap bag_mcap/*.mcap /test_odom -> 60 poses      (works)
```
Both work with evo 1.38 `bag2` (it reads through the `rosbags` library). Chosen: **sqlite3**,
the rosbag2 default in Humble: no extra plugin, read by `ros2 bag info`, rosbag2_py and evo.

### evo installation change
`evo_traj --save_plot` crashed with the pip --user install:
`ImportError: cannot import name 'docstring' from 'matplotlib'` — Ubuntu's
python3-matplotlib 3.5 (required by rqt_plot) installs a regular `mpl_toolkits` package that
shadows the namespace package of pip's matplotlib 3.10. Fix without touching system packages:
```
python3 -m venv ~/.venvs/evo && ~/.venvs/evo/bin/pip install "evo==1.38.0"
# evo 1.38.0, matplotlib 3.10.9, numpy 2.2.6, scipy 1.15.3, rosbags 0.11.9
~/.venvs/evo/bin/evo_config set plot_backend Agg
```
All evo commands below use `~/.venvs/evo/bin/`.

### Verification

Runs (all `ros2 launch gnssd_bringup run.launch.py scenario:=loop seed:=N odom_noise:=...`,
gui:=false, headless rendering on NVIDIA EGL; every run exited by itself with code 0):

| bag | noise | seed | drawn scale errors (lin / ang) | driven | duration | max dist. from route |
|---|---|---|---|---|---|---|
| loop_seed1_20261022_230940 | off | 1 | 0 / 0 | 42.50 m | 213.1 s | 0.107 m |
| loop_seed1_20261022_231330 | on | 1 | -0.01463 / +0.02085 | 42.50 m | 213.1 s | 0.107 m |
| loop_seed2_20261022_231719 | on | 2 | +0.01824 / +0.02687 | 42.50 m | 213.1 s | 0.107 m |
| loop_seed3_20261022_232109 | on | 3 | -0.01048 / +0.00265 | 42.50 m | 213.1 s | 0.107 m |

Planned route 43.04 m (2 laps); the driver cuts the 45-degree corners slightly, hence 42.50 m.
The route is identical in every run (driver steers on ground truth); the final pose is
(0.000, 0.049) — the goal tolerance is 0.05 m. Max distance from the polyline 0.107 m:
with >= 0.8 m planned clearance minus 0.107 m deviation minus the 0.14 m half-width, the robot stayed > 0.5 m from every wall/obstacle: no collisions.

```
$ cat bags/loop_seed1_20261022_231330/run_info.yaml
scenario: loop
seed: 1
odom_noise:
  enabled: true
  random_scale: true
  linear_scale_error: 0.02
  angular_scale_error: 0.03
  applied_linear_scale_error: -0.014625430235503951
  applied_angular_scale_error: 0.02084602421623396
  linear_noise_stddev: 0.02
  angular_noise_stddev: 0.02
  standstill_linear_speed: 0.001
  standstill_angular_speed: 0.001
  seed: 1
git_commit: 3725d81edc4a10c6b842b2c6ddba23845644879b-dirty
gz_sim_version: 8.15.0
```

```
$ evo_traj bag2 bags/loop_seed1_20261022_231330 /odom_raw /odom --ref /ground_truth \
    --plot_mode xy --save_plot docs/img/step2_traj_seed1.png
name: /odom_raw      10839 poses, 42.500m path length, 216.760s duration
name: /odom          10838 poses, 41.829m path length, 216.740s duration
name: /ground_truth  10839 poses, 42.498m path length, 216.760s duration
```
Plot: `docs/img/step2_traj_seed1_trajectories.png` (and `_xyz.png`).

ATE, translation part, **not aligned** (`evo_ape bag2 <bag> /ground_truth <est>`, no -a,
no --align_origin):

| bag | estimate | RMSE | mean | max |
|---|---|---|---|---|
| noise off, seed 1 | /odom_raw | 0.0029 m | 0.0028 m | 0.0030 m |
| noise off, seed 1 | /odom | 0.0048 m | 0.0047 m | 0.0058 m |
| noise on, seed 1 | /odom_raw | 0.0029 m | 0.0028 m | 0.0030 m |
| noise on, seed 1 | /odom | **0.479 m** | 0.400 m | 1.009 m |
| noise on, seed 2 | /odom | **0.599 m** | 0.476 m | 1.408 m |
| noise on, seed 3 | /odom | **0.247 m** | 0.205 m | 0.466 m |

- /odom_raw: ~3 mm, constant. It appears within 40 ms of the start of motion (t = 4.06–4.08 s,
  velocity step 0 -> 0.2 m/s, brief wheel slip) and does not grow afterwards.
- /odom without noise: 4.8 mm (Euler integration in the node vs DiffDrive's own integration).
- /odom with noise: 0.25–0.60 m RMSE; the seed changes both the drawn scale errors and the
  result. Seed 3 drew a small angular error (+0.27 %) and has the smallest ATE, seed 2 the
  largest errors (+1.8 % / +2.7 %) and the largest ATE.

```
$ colcon build --symlink-install
Summary: 3 packages finished [1.49s]
$ ament_flake8 src/gnssd_tools
9 files checked   (0 errors)
$ ament_pep257 src/gnssd_tools
No problems found
$ colcon test --packages-select gnssd_tools && colcon test-result --verbose
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```
