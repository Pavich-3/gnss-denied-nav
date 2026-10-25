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

## Step 3 — IMU model, EKF base layer, reproducibility guards (2026-10-24)

Code commits: `f17a49f` (IMU model, EKF, guards, imu_covariance_node, tf_to_tum),
`68dff80` (run_batch.sh fix, gyro bias estimate). All bags below were recorded with
the clean tree at `68dff80` (`allow_dirty: false` in every run_info.yaml).

### IMU model (`robot_params.xacro`)
Calibrated MEMS gyro, Bosch BMI088 class (full reasoning in the xacro comment):

| parameter | value | reasoning |
|---|---|---|
| gyro white noise sigma | 0.0017 rad/s | 0.014 deg/s/sqrt(Hz) x sqrt(50 Hz) = 0.099 deg/s |
| gyro bias | mean 0, bias_stddev 1.0e-4 rad/s | residual after standstill calibration, drawn once per run (random sign) |
| gyro dynamic bias | 1.0e-5 rad/s, correlation time 1000 s | in-run instability ~2 deg/h; supported by gz-sensors 8 (first-order Gauss-Markov) |
| accelerometer | sigma 0.02 m/s^2, bias mean 0.01 (sd 0.001) | unchanged placeholders, NOT fused by the EKF |

gz-sensors 8 `GaussianNoiseModel.cc`: bias = `math::Rand::DblNormal(bias_mean, bias_stddev)`,
sign flipped with probability 0.5; `dynamic_bias_stddev` / `dynamic_bias_correlation_time`
are read and applied in `ApplyImpl`. All draws use `gz::math::Rand`.

### Seeding the Gazebo sensor noise
`gz sim --seed <n>` -> `ServerConfig::SetSeed()` -> `math::Rand::Seed(n)` (gz-sim 8
`src/ServerConfig.cc`). sim.launch.py passes the same `seed` argument as odom_noise_node;
run_info.yaml stores it as `gz_sim_seed`. Checked: gyro z samples at the same sim stamps
while standing still:
```
seed 7, run A: +0.002642@16.00, -0.001321@16.01, +0.000952@16.02 (mean over 400: +0.000034)
seed 7, run B: +0.002642@16.00, -0.001321@16.01, +0.000952@16.02 (mean over 400: +0.000034)
seed 8:        +0.000666@16.00, -0.001339@16.01, -0.001579@16.02 (mean over 400: -0.000004)
```
Same seed -> identical sensor noise; different seed -> different noise and bias.

### IMU covariances (`imu_covariance_node`)
Gazebo publishes the IMU with all covariances = 0; robot_localization would replace them with
1e-9 and treat the gyro as perfect. `imu_covariance_node` republishes `/imu_raw` as `/imu` with
angular_velocity_covariance diag = 0.0017^2 = 2.89e-6, linear_acceleration_covariance diag =
0.02^2 = 4.0e-4, orientation_covariance[0] = -1 (REP 145). The sigmas are read by sim.launch.py
from robot_params.xacro (no second copy). Checked in a bag: `/imu` 100.00 Hz,
`angular_velocity_covariance [2.89e-06, 0, 0, 0, 2.89e-06, 0, 0, 0, 2.89e-06]`,
`orientation_covariance[0] -1.0`.

**Weighting of the yaw rate in the EKF:** odometry w variance 0.02^2 = 4.0e-4 vs gyro
0.0017^2 = 2.89e-6 -> ratio 138 : 1, i.e. the gyro dominates the fused yaw rate.

### EKF (`config/ekf.yaml`, robot_localization 3.5.4)
frequency 30, two_d_mode, publish_tf, world_frame odom; odom0 = /odom fusing vx + vyaw;
imu0 = /imu fusing vyaw only; print_diagnostics true; process_noise_covariance not set.
`ros2 param get /ekf_filter_node process_noise_covariance` -> "Parameter not set", i.e. the
built-in default is used. Default diagonal (robot_localization `params/ekf.yaml`: "Defaults to
the matrix below if unspecified"), order x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az:
```
0.05 0.05 0.06 0.03 0.03 0.06 0.025 0.025 0.04 0.01 0.01 0.02 0.01 0.01 0.015   (off-diagonal 0)
```

Environment fix needed for the EKF: `ekf_node` died with `libdiagnostic_updater.so: cannot open
shared object file` — installed ros-humble-diagnostic-updater 4.0.6 was older than the one
robot_localization 3.5.4 was built against. Fixed with
`sudo apt-get install --only-upgrade ros-humble-diagnostic-updater` (-> 4.0.7). Recorded in CLAUDE.md
together with "never run apt autoremove" (219 ROS packages are listed as removable).

```
$ (bag) /odometry/filtered: 3323 msgs over one lap, 30.00 Hz (sim time); frame_id odom, child base_link
$ ros2 topic info /tf -v          (sim + EKF + driving)
Publisher count: 2   robot_state_publisher, ekf_filter_node
$ ros2 topic info /tf_static -v
Publisher count: 1   robot_state_publisher
$ ros2 run tf2_tools view_frames  -> docs/img/step3_tf_frames.pdf
odom -> base_link; base_link -> caster_link, chassis_link, imu_link, lidar_link, left/right_wheel_link
```
odom -> base_link comes only from ekf_filter_node (robot_state_publisher publishes only the
robot's own links).

### Reproducibility guards
- run.launch.py refuses to start with uncommitted changes unless `allow_dirty:=true`:
  ```
  $ ros2 launch gnssd_bringup run.launch.py scenario:=guardtest seed:=1 laps:=1
  [ERROR] [run.launch]: The repository has uncommitted changes; refusing to record a run that
  cannot be traced to a commit. Commit first, or pass allow_dirty:=true.
  # no process started, no bag created
  ```
  run_info.yaml records `allow_dirty`, `gz_sim_seed` and `laps` (read from route_driver_node).
- `src/gnssd_tools/scripts/run_batch.sh <scenario> <laps> <seed_from> <seed_to> [args]`:
  node check before every run, logs in bags/logs/, index in bags/index.csv:
  ```
  bag,scenario,laps,seed,commit,duration_s,extra_args
  loop_seed1_20261024_111448,loop,2,1,68dff80249bbf4a5d21aad1d687c0621088ec44c,219.2,
  loop_seed2_20261024_111837,loop,2,2,68dff80249bbf4a5d21aad1d687c0621088ec44c,221.3,
  loop_seed3_20261024_112228,loop,2,3,68dff80249bbf4a5d21aad1d687c0621088ec44c,219.2,
  long_seed1_20261024_112618,long,10,1,68dff80249bbf4a5d21aad1d687c0621088ec44c,1073.6,
  loop_seed1_20261024_114421,loop,2,1,68dff80249bbf4a5d21aad1d687c0621088ec44c,220.3,odom_noise:=false
  ```

### evo and /tf
```
$ ~/.venvs/evo/bin/evo_traj bag2 <bag> /tf:odom.base_link
ModuleNotFoundError: No module named 'tf2_py'
```
evo's TF reader needs ROS's tf2_py, which the venv does not have (and Humble's tf2_py is built
for the system numpy 1.x). Fix: `src/gnssd_tools/scripts/tf_to_tum.py` (run with the venv
Python, needs only rosbags + numpy + scipy) reads /tf and /tf_static, finds the chain
parent -> child, interpolates dynamic edges (linear / slerp, no extrapolation), composes and
writes TUM. Check against /odometry/filtered (same one-lap bag):
```
$ ~/.venvs/evo/bin/python src/gnssd_tools/scripts/tf_to_tum.py <bag> odom base_link tf.tum
odom -> base_link: 3323 poses written (time base: odom -> base_link, 3323 stamps)
$ evo_traj bag2 <bag> /odometry/filtered --save_as_tum
$ evo_ape tum odometry_filtered.tum tf.tum                      -> rmse 0.000000 m, max 0.000000 m
$ evo_ape tum odometry_filtered.tum tf.tum --pose_relation angle_deg -> rmse 0.000000 deg
$ tf_to_tum.py <bag> odom lidar_link    # dynamic + static edge: z = 0.112500, x/y/yaw = base_link
```

### gz shutdown note
When run.launch.py ends the experiment through the `Shutdown` action, `gz sim` sometimes does
not react to SIGINT/SIGTERM and is killed after 15 s (`escalating to 'SIGKILL'`). The recorder
gets its own SIGINT and closes the bag normally (metadata.yaml present, all bags readable).
A plain Ctrl+C / SIGINT of sim.launch.py stops gz cleanly. Not investigated further.

### Verification: ATE (translation, not aligned) and heading error vs /ground_truth

Gyro bias = mean(/imu_raw w_z - /ground_truth w_z) over the run
(`src/gnssd_tools/scripts/estimate_gyro_bias.py`); includes the dynamic bias.

Scenario `loop`, 2 laps (42.50 m, 213.1 s), odometry noise ON:

| seed | odom scale error lin / ang | gyro bias (est.) | estimate | RMSE | mean | max | heading RMSE | heading max |
|---|---|---|---|---|---|---|---|---|
| 1 | -1.46 % / +2.08 % | +2.14e-4 rad/s | /odometry/filtered | **0.109 m** | 0.098 m | 0.200 m | 1.25 deg | 2.49 deg |
| | | | /odom | 0.477 m | 0.398 m | 1.009 m | 7.91 deg | 13.95 deg |
| 2 | +1.82 % / +2.69 % | -2.5e-5 rad/s | /odometry/filtered | **0.110 m** | 0.099 m | 0.182 m | 0.27 deg | 0.75 deg |
| | | | /odom | 0.597 m | 0.472 m | 1.408 m | 9.90 deg | 17.69 deg |
| 3 | -1.05 % / +0.27 % | -1.14e-4 rad/s | /odometry/filtered | **0.069 m** | 0.062 m | 0.121 m | 0.89 deg | 1.70 deg |
| | | | /odom | 0.248 m | 0.205 m | 0.467 m | 2.56 deg | 4.67 deg |

Scenario `long`, 10 laps (212.12 m, 1061.2 s), seed 1, odometry noise ON
(scale errors -1.46 % / +2.08 %, gyro bias est. +2.59e-4 rad/s):

| estimate | RMSE | mean | max | heading RMSE | heading max |
|---|---|---|---|---|---|
| /odometry/filtered | **0.481 m** | 0.406 m | 1.013 m | 8.47 deg | 14.55 deg |
| /odom | 2.377 m | 2.023 m | 4.922 m | 45.53 deg | 80.82 deg |

ATE over time: `docs/img/step3_ape_over_time_long_seed1.png` (evo_res of both APE results);
trajectories: `docs/img/step3_traj_long_seed1_trajectories.png`.

Odometry noise OFF, gyro as configured (`loop`, 2 laps, seed 1, gyro bias est. +2.06e-4 rad/s):

| estimate | RMSE | mean | max | heading RMSE | heading max |
|---|---|---|---|---|---|
| /odometry/filtered | 0.069 m | 0.053 m | 0.162 m | 1.23 deg | 2.46 deg |
| /odom | 0.0050 m | 0.0049 m | 0.0061 m | 0.05 deg | 0.20 deg |

/odom_raw in all runs: 0.0029–0.0032 m (start-up slip, Step 2).

### Erratum (found in Step 4)
The gyro dynamic bias in this step was mis-scaled: gz-sensors' `dynamic_bias_stddev` is not the
stationary standard deviation (it is sigma_gz with stationary std sigma_gz * sqrt(tau/2)), so
1e-5 with tau = 1000 s gave a stationary drift of **2.2e-4 rad/s** instead of 1e-5. Also the bias
estimator used the smoothed /ground_truth yaw rate (timing-dependent error ~1e-4 rad/s).
Recomputed with the fixed estimator (heading change, Step 4) for the Step 3 bags:
loop seed 1: +1.97e-4, seed 2: -4.1e-5, seed 3: -1.20e-4, long seed 1: +2.35e-4,
noise-off seed 1: +1.99e-4 rad/s. The heading numbers of the EKF in this step therefore include
an unrealistically large gyro drift; the base-layer results with the corrected IMU model are in
the Step 4 tables (/odometry/filtered column).

### Comment: what the gyro does
- With noisy odometry the EKF reduces the heading error 2.9–37x (loop: 7.9 -> 1.25, 9.9 -> 0.27,
  2.6 -> 0.89 deg; long: 45.5 -> 8.5 deg) and the position error 3.6–5.4x
  (loop RMSE 0.25–0.60 -> 0.07–0.11 m; long 2.38 -> 0.48 m).
- Because the gyro is weighted 138x more than the odometry yaw rate, the EKF heading is
  practically the integrated gyro: its heading error is the same with and without odometry
  noise (1.25 vs 1.23 deg, seed 1) and grows with the gyro bias (long run: 2.59e-4 rad/s x
  1061 s = 15.7 deg, observed max 14.6 deg). The no-noise run therefore shows 6.9 cm instead of
  0.5 cm: the filter adds no error of its own, but it passes the gyro bias into the estimate.
- The remaining EKF position error comes mostly from the linear scale error of the odometry
  (only odometry measures v; seed 2 has +1.8 % and 0.27 deg heading error but still 0.11 m).
  Neither of the two drifts (gyro bias -> heading, scale -> distance) is observable without an
  absolute reference; removing them is the job of the correction layer (SLAM, Step 4).
- Seed 1 gyro bias (2.1–2.6e-4 rad/s) is ~2.5x the modelled bias_stddev; seeds 2 and 3 are
  within 1.1 sigma.

```
$ colcon build --symlink-install
Summary: 3 packages finished
$ ament_flake8 src/gnssd_tools
12 files checked   (0 errors)
$ ament_pep257 src/gnssd_tools
No problems found
$ colcon test --packages-select gnssd_tools && colcon test-result --verbose
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```

## Step 4 — Correction layer (slam_toolbox), resource logging (2026-10-24)

Code commits: `36ffb5b` (slam_toolbox, loop closure monitor, resource logger, tools),
`5ad701c` (gyro dynamic bias scaling fix, robust gyro bias estimate). All runs below:
clean tree at `5ad701c`, `allow_dirty: false`.

### slam_toolbox configuration (`config/slam_toolbox.yaml`)
Copy of `/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml`
(slam_toolbox 2.6.10) with these changes only:

| parameter | package value | project value | reason |
|---|---|---|---|
| use_sim_time | (not set) | true | simulation clock, like every node |
| base_frame | base_footprint | base_link | the robot has no base_footprint frame (REP 105 chain map -> odom -> base_link) |
| max_laser_range | 20.0 | 12.0 | equals the lidar model range_max |

Unchanged on purpose: mode mapping, odom_frame odom, map_frame map, scan_topic /scan,
resolution 0.05, do_loop_closing true and all loop thresholds, transform_publish_period 0.02,
map_update_interval 5.0, minimum_travel_distance 0.5, minimum_travel_heading 0.5.
Node: `async_slam_toolbox_node` (name slam_toolbox), started by an `OnProcessStart` handler
once the EKF process has started (it needs odom -> base_link). Launch arg `slam:=true`.
RViz: fixed frame map, /map display added. Startup warning (harmless):
`minimum laser range setting (0.0 m) exceeds the capabilities of the used Lidar (0.1 m)`.

### TF
```
$ ros2 topic info /tf -v        (sim + EKF + SLAM, driving)
Publisher count: 4   robot_state_publisher, ekf_filter_node, slam_toolbox, slam_toolbox
$ ros2 topic info /tf_static -v
Publisher count: 1   robot_state_publisher
$ ros2 run tf2_tools view_frames   -> docs/img/step4_tf_frames.pdf
map -> odom -> base_link -> {caster, chassis, imu, lidar, left_wheel, right_wheel}_link
```
(slam_toolbox owns two /tf publishers inside the one node.)

### Corrected estimate
Defined as TF map -> base_link = slam_toolbox map -> odom composed with EKF odom -> base_link,
extracted with `tf_to_tum.py <bag> map base_link`. Fix needed in tf_to_tum.py: slam_toolbox
re-sends map -> odom every 20 ms with the stamp of the last scan (repeated stamps); the script now
keeps the latest sample per stamp. Check: TF odom -> base_link vs /odometry/filtered still 0.000000 m.
At the start map -> odom is the identity in every run, e.g.
`first map->odom: t=0.300  0.000 0.000 0.000  q = (0, 0, 0, 1)`, so map = world at t = 0 and ATE is
computed without alignment.
slam_toolbox 2.6.10 also publishes `/pose` (geometry_msgs/PoseWithCovarianceStamped); it is
recorded, but the TF chain is the reference definition of the corrected estimate.

### Loop closures (`loop_closure_monitor_node` -> `<bag>/loop_closures.yaml`)
slam_toolbox does not publish or log automatic loop closures (Karto's "Loop closed!" goes only to
listeners). Karto runs the graph optimisation (CorrectPoses) only on a loop closure, and only an
optimisation moves already existing vertices. The monitor watches
/slam_toolbox/graph_visualization (vertex markers, ns "slam_toolbox") and counts an event when a
known vertex moves by more than 5 mm; it stores the count, the sim time and the largest vertex
shift. Time resolution = graph publication period (map_update_interval, 5 s).
Cross-check (2-lap test run with `debug_logging: true`, which prints a Ceres report per
optimisation): 3 Ceres reports, 3 events detected (t = 126.5, 151.2, 200.8 s).

### Resource logging (`resource_logger_node` -> `<bag>/resources.csv`)
1 Hz (wall clock), psutil, processes from `config/resource_logger.yaml`:
`[ekf_filter_node, async_slam_toolbox_node]` (stand processes excluded). Columns:
wall_time, sim_time, process, pid, cpu_core_pct (100 % = one core), cpu_total_pct
(/ 8 cores), rss_mb. Summary: `src/gnssd_tools/scripts/summarize_resources.py` (first sample of
each process skipped: psutil needs one interval).

### Gyro model fix (made during this step)
gz-sensors' dynamic bias update `b = exp(-dt/tau) b + N(0, sigma_gz^2 tau/2 (1 - exp(-2dt/tau)))`
has the stationary std sigma_gz * sqrt(tau/2). robot_params.xacro keeps the physical value
(1e-5 rad/s, ~2 deg/h) and robot.urdf.xacro passes sigma_gz = 1e-5 / sqrt(500) = 4.47e-7 to Gazebo.
Found because seed 1 gave different biases in Step 3 and Step 4; see the Step 3 erratum.
`estimate_gyro_bias.py` now uses (integral of gyro - true heading change) / T instead of the
smoothed /ground_truth yaw rate (two Step 3 runs with the same bias: 1.974e-4 and 1.989e-4).
With the fix, seed 1 gives +4.08e-5 (loop) and +3.67e-5 (long): the bias follows the seed.

### Verification: ATE (translation, NOT aligned) and heading error vs /ground_truth

Scenario `loop`, 2 laps (42.50 m, 213 s), odometry noise on:

| seed | scale err. lin / ang | gyro bias (est.) | estimate | RMSE | mean | max | heading RMSE | heading max | loop closures |
|---|---|---|---|---|---|---|---|---|---|
| 1 | -1.46 % / +2.08 % | +4.08e-5 | /odom | 0.477 m | 0.397 m | 1.009 m | 7.89 deg | 13.95 deg | |
| | | | /odometry/filtered | 0.077 m | 0.070 m | 0.121 m | 0.29 deg | 0.57 deg | |
| | | | **TF map->base_link** | **0.029 m** | 0.027 m | 0.056 m | 0.36 deg | 1.25 deg | 1 (t = 129.6 s) |
| 2 | +1.82 % / +2.69 % | +4.50e-5 | /odom | 0.598 m | 0.473 m | 1.406 m | 9.91 deg | 17.70 deg | |
| | | | /odometry/filtered | 0.105 m | 0.091 m | 0.185 m | 0.42 deg | 0.63 deg | |
| | | | **TF map->base_link** | **0.019 m** | 0.017 m | 0.038 m | 0.16 deg | 0.71 deg | 0 |
| 3 | -1.05 % / +0.27 % | -1.25e-4 | /odom | 0.247 m | 0.205 m | 0.465 m | 2.54 deg | 4.67 deg | |
| | | | /odometry/filtered | 0.071 m | 0.063 m | 0.128 m | 0.90 deg | 1.60 deg | |
| | | | **TF map->base_link** | **0.018 m** | 0.017 m | 0.034 m | 0.12 deg | 0.72 deg | 2 (t = 146.3, 198.8 s) |

Scenario `long`, 10 laps (212.11 m, 1061 s), seed 1 (scale -1.46 % / +2.08 %, gyro bias +3.67e-5):

| estimate | RMSE | mean | max | heading RMSE | heading max | loop closures |
|---|---|---|---|---|---|---|
| /odom | 2.380 m | 2.026 m | 4.926 m | 45.59 deg | 80.89 deg | |
| /odometry/filtered | 0.156 m | 0.135 m | 0.311 m | 1.71 deg | 2.64 deg | |
| **TF map->base_link** | **0.012 m** | 0.011 m | 0.045 m | 0.15 deg | 0.77 deg | 24 |

Loop closure times (long, s): 127.8 200.7 230.1 303.8 332.6 376.6 455.1 518.2 547.6 625.7 669.8
694.3 728.2 752.7 772.3 796.9 836.0 864.6 884.1 908.5 942.9 972.3 991.9 1049.8.
Largest vertex shift per closure: max 0.020 m, mean 0.010 m (long); 0.020 m (loop seed 1),
0.009 m (loop seed 3).

Figures (long, seed 1): trajectories of the three estimates + ground truth
`docs/img/step4_traj_long_seed1.png`; ATE over time, base vs corrected
`docs/img/step4_ape_over_time_long_seed1.png`; map at the end of the run
`docs/img/step4_map_long_seed1.png` (and `_with_gt.png` with the ground-truth path).

### Resources (system processes only; 8 cores; "1-core" = 100 % per core)

| run | process | CPU 1-core mean / max | CPU all cores mean / max | RSS mean / max |
|---|---|---|---|---|
| loop s1 | ekf_filter_node | 14.3 / 16.0 % | 1.79 / 2.00 % | 28.9 / 29.0 MiB |
| | async_slam_toolbox_node | 13.2 / 33.0 % | 1.65 / 4.12 % | 46.3 / 50.3 MiB |
| | **system** | **27.5 / 47.0 %** | **3.43 / 5.88 %** | **75.2 / 79.3 MiB** |
| loop s2 | ekf_filter_node | 12.0 / 16.0 % | 1.50 / 2.00 % | 29.1 / 29.1 MiB |
| | async_slam_toolbox_node | 11.1 / 37.1 % | 1.39 / 4.64 % | 45.9 / 50.8 MiB |
| | **system** | **23.1 / 45.1 %** | **2.88 / 5.64 %** | **75.0 / 79.9 MiB** |
| loop s3 | ekf_filter_node | 13.7 / 16.0 % | 1.72 / 2.00 % | 29.0 / 29.0 MiB |
| | async_slam_toolbox_node | 12.8 / 34.0 % | 1.59 / 4.25 % | 45.2 / 50.2 MiB |
| | **system** | **26.5 / 48.0 %** | **3.31 / 6.00 %** | **74.2 / 79.2 MiB** |
| long s1 | ekf_filter_node | 14.5 / 17.0 % | 1.81 / 2.12 % | 29.4 / 29.6 MiB |
| | async_slam_toolbox_node | 14.3 / 40.0 % | 1.79 / 5.00 % | 51.6 / 55.4 MiB |
| | **system** | **28.8 / 54.0 %** | **3.61 / 6.76 %** | **81.0 / 84.8 MiB** |

slam_toolbox RSS grows slowly with the map/graph (46 -> 52 MiB mean from 2 to 10 laps); the EKF
is constant.

### Comment: does the correction layer remove the two drifts of Step 3?
- **Heading (gyro bias):** yes. Base-layer heading error grows with time (long: 1.71 deg RMSE,
  2.64 deg max); corrected 0.15 deg RMSE, 0.77 deg max, not growing.
- **Distance (odometry scale error):** yes. The EKF position error grows lap by lap (long: up to
  0.31 m, periodic with the lap because the error direction rotates with the route); the
  corrected error stays below 4.5 cm for the whole 212 m (ATE-over-time plot). Corrected ATE is
  2.6–13x smaller than the base layer (loop 0.077–0.105 -> 0.018–0.029 m, long 0.156 -> 0.012 m).
- The correction does not depend on loop closures alone: seed 2 had none and still 1.9 cm, because
  slam_toolbox matches every new scan against the existing map (scan-to-map matching bounds the
  drift continuously); loop closures add small graph corrections (<= 2 cm vertex shifts).
- **Failures:** none observed. No wrong loop closure (largest vertex shift 2 cm), the map after
  10 laps has single, straight walls (no doubling or bending), map -> odom = identity at start in
  every run. One minor point: in loop seed 1 the corrected heading RMSE (0.36 deg) is slightly
  worse than the base layer (0.29 deg) — with the realistic gyro the base heading is already
  good over 2 laps and scan matching adds small heading jitter; position is still 2.6x better.

```
$ colcon build --symlink-install
Summary: 3 packages finished
$ ament_flake8 src/gnssd_tools
16 files checked   (0 errors)
$ ament_pep257 src/gnssd_tools
No problems found
$ colcon test --packages-select gnssd_tools && colcon test-result --verbose
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```

## Step 4b — Second world: warehouse (2026-10-24)

### Why
In the 10 x 8 m room a uniform odometry scale error only scales the trajectory about the start,
so the base-layer error stays bounded by ~k x room size and the base layer meets NFR-1 (0.2 m)
even without correction. A larger world with a route that goes far from the start shows how the
base-layer error grows with distance, and tests scan matching in long, repetitive aisles (a known
weak case for 2D lidar SLAM). The system configuration (ekf.yaml, slam_toolbox.yaml, noise
models, URDF, bridge) is unchanged since `5ad701c` (`git diff 5ad701c` on these files is empty).

Code commits: `2a9e94a` (warehouse world, route, world:= selection, error_vs_distance.py,
RViz fixed frame odom without SLAM), `9023ec4` (run_batch.sh per-world time limit). All runs at
the clean tree `9023ec4`, `allow_dirty: false`.

### World `worlds/warehouse.sdf`
Interior 30 x 20 m (x -2..28, y -2..18), walls 2.5 m. Start = world origin in the south main
aisle, facing +x. One static model, one link, 33 elements (`gz sdf -k`: Valid):
- 4 rack rows A–D (y 1.5–2.7, 5.1–6.3, 8.7–9.9, 12.3–13.5), each x 3–12.4 and 14.6–24, solid
  blocks 1.2 m high (the bottom pallet level is what the lidar at 0.1475 m sees); aisles 2.4 m;
  cross aisles at x -2..3, 12.4..14.6 (middle), 24..28.
- Rows B and C: identical 2.2 m pallet blocks with 0.2 m gaps -> the aisle between them
  (y 6.3..8.7, 21 m long) is repetitive on purpose. Rows A and D are solid with one irregular gap
  each (A: x 7.0–7.6, D: x 18.5–19.5).
- Distinct features: office box (NE corner), three dock platforms on the north wall, two columns
  in the middle cross aisle, a pallet stack near the start. Grey materials; same systems as room.sdf.
- Collision/visual names carry `_collision` / `_visual` suffixes (strict SDF check requires unique names).

### Route `config/route_warehouse.yaml`
One lap, 21 waypoints, 178.6 m: south main aisle east -> aisle 1 west -> aisle 2 (long repetitive)
east -> aisle 3 west -> north area east (far side, 29.7 m from the start) -> east cross aisle south
-> south main aisle west back to the start. Corners cut at 45 deg (0.5 m). Smallest distance from
the path centre line to any obstacle 1.20 m (1.06 m from the robot side). Controller parameters
identical to `route_room.yaml` (renamed from route.yaml; content unchanged).
`world:=room|warehouse` selects worlds/<world>.sdf and config/route_<world>.yaml; run.launch.py,
run_info.yaml (`world:`), run_batch.sh (`<world>` first argument) and bags/index.csv (`world`
column; older rows filled with `room`) carry it. Scenario names: room_loop, room_long, warehouse_loop.

### Simulation check
```
$ ros2 launch gnssd_bringup sim.launch.py world:=warehouse gui:=false drive:=true
$ gz topic -e -t /stats -n 1   (3 samples)   real_time_factor: 0.99997, 0.99993, 1.00085
$ ros2 topic hz  /scan 9.964   /imu 99.679   /odom 49.783   /odometry/filtered 29.882
```
RTF over each recorded run (sim time / wall time of /clock): seed 1 0.9946, seed 2 0.9933,
seed 3 0.9952.
Screenshots: Gazebo top view `docs/img/step4b_gazebo_top.png`, RViz with /scan
`docs/img/step4b_rviz_scan.png` — PENDING (taken by the user; automatic capture got another window).

Known harmless shutdown issue: odom_noise_node sometimes prints a Traceback
(`RuntimeError: Unable to convert call argument to Python object`) after SIGINT — a subscription
callback runs while rclpy shuts the context down. Data are not affected (recorder is separate).

Process note: the first batch attempt was stopped by run_batch.sh's time limit (it assumed a room
lap, 330 s); fixed in `9023ec4`. Seed 3 was refused once by the clean-tree guard because a figure
had been written into docs/img during the batch; rerun alone at the same commit.

### Verification: warehouse_loop, 1 lap (177.93 m, 890.3 s), odometry noise on

| seed | scale err. lin / ang | gyro bias (est.) | estimate | RMSE | mean | max | heading RMSE | heading max | loop closures |
|---|---|---|---|---|---|---|---|---|---|
| 1 | -1.46 % / +2.08 % | +4.26e-5 | /odom | 2.731 m | 2.303 m | 4.872 m | 4.31 deg | 10.08 deg | |
| | | | /odometry/filtered | **0.367 m** | 0.338 m | 0.717 m | 1.34 deg | 2.20 deg | |
| | | | TF map->base_link | **0.072 m** | 0.065 m | 0.163 m | 0.21 deg | 0.81 deg | 2 (t = 718.9, 753.7 s) |
| 2 | +1.82 % / +2.69 % | +2.01e-5 | /odom | 3.197 m | 2.711 m | 5.197 m | 3.89 deg | 9.48 deg | |
| | | | /odometry/filtered | **0.317 m** | 0.281 m | 0.566 m | 0.79 deg | 1.32 deg | |
| | | | TF map->base_link | **0.071 m** | 0.066 m | 0.131 m | 0.22 deg | 0.67 deg | 2 (t = 722.7, 757.5 s) |
| 3 | -1.05 % / +0.27 % | -9.51e-5 | /odom | 0.656 m | 0.587 m | 1.133 m | 2.28 deg | 4.31 deg | |
| | | | /odometry/filtered | **0.598 m** | 0.497 m | 1.378 m | 3.08 deg | 4.89 deg | |
| | | | TF map->base_link | **0.052 m** | 0.048 m | 0.105 m | 0.19 deg | 0.66 deg | 2 (t = 719.0, 748.8 s) |

map -> odom at the start = identity in all three runs. All loop closures happen when the route
returns along the east cross aisle (x = 26) to places seen 10 min earlier; the return along the
south main aisle to the start produced no closure.

Corrected trajectory, consecutive-sample jumps (estimate step minus true step): largest 4.7 cm
(seed 1, t = 718.6 s), 2.7 cm (seed 2, t = 718.4 s) — both at the first loop closure — and 1.3 cm
(seed 3). Largest corrected error: 0.163 m at (0.45, 0) t = 891.7 s (seed 1), 0.131 m at (0.05, 0)
t = 894.3 s (seed 2), 0.103 m at (5.96, 0) t = 863.2 s (seed 3): always on the final stretch of the
south main aisle.

Figures (seed 1): error vs distance from the start and vs travelled distance
`docs/img/step4b_error_vs_distance_seed1.png`; trajectories `docs/img/step4b_traj_seed1.png`;
ATE over time `docs/img/step4b_ape_over_time_seed1.png`; map `docs/img/step4b_map_seed1.png`
(604 x 406 cells) and `_with_gt.png`.

### Resources (system processes; 8 cores)

| run | process | CPU 1-core mean / max | CPU all cores mean / max | RSS mean / max |
|---|---|---|---|---|
| wh s1 | ekf_filter_node | 12.4 / 15.0 % | 1.55 / 1.88 % | 29.3 / 29.3 MiB |
| | async_slam_toolbox_node | 11.5 / 33.0 % | 1.44 / 4.12 % | 50.4 / 56.6 MiB |
| | **system** | **23.9 / 45.0 %** | **2.99 / 5.62 %** | **79.7 / 85.9 MiB** |
| wh s2 | ekf_filter_node | 12.0 / 15.0 % | 1.50 / 1.88 % | 29.2 / 29.3 MiB |
| | async_slam_toolbox_node | 11.1 / 31.0 % | 1.38 / 3.88 % | 50.5 / 56.9 MiB |
| | **system** | **23.0 / 43.0 %** | **2.88 / 5.38 %** | **79.8 / 86.2 MiB** |
| wh s3 | ekf_filter_node | 11.8 / 14.0 % | 1.47 / 1.75 % | 29.1 / 29.1 MiB |
| | async_slam_toolbox_node | 11.1 / 29.0 % | 1.39 / 3.62 % | 50.7 / 56.7 MiB |
| | **system** | **22.9 / 41.0 %** | **2.86 / 5.12 %** | **79.8 / 85.8 MiB** |

Max RSS of slam_toolbox: warehouse 56.6–56.9 MiB vs room 50.2–50.8 MiB (2 laps, 42.5 m) and
55.4 MiB (10 laps, 212 m). Memory follows the size of the pose graph / map rather than the world
area alone (warehouse map 604 x 406 cells vs 199 x 160).

### Comment
- **Base layer vs 0.2 m (NFR-1): no longer met.** ATE RMSE 0.32–0.60 m, max 0.57–1.38 m. The error
  grows with travelled distance (ATE-vs-distance plot: up to 0.67 m mid-route, 0.72 m at the end,
  not a simple function of |p_true|) — unlike the room, where it stayed below 0.16 m RMSE.
- **Gyro bias over a long run:** seed 3 (small odometry angular scale error +0.27 %, gyro bias
  -9.5e-5 rad/s = 4.9 deg in 893 s) ends with a WORSE heading in the EKF (3.08 deg RMSE) than in the
  raw odometry (2.28 deg): the gyro is weighted 138x more than the odometry yaw rate, so its bias
  passes straight into the heading. Not tuned (by design of this step); relevant for the later
  process-noise / covariance tuning.
- **Correction layer:** 0.05–0.07 m RMSE, max 0.10–0.16 m in all three seeds, 4.5–11.5x better than
  the base layer. Scan matching holds in the repetitive aisle 2: no jumps there (largest jumps
  2.7–4.7 cm, at loop closures), no wrong loop closure.
- **Weak spot found:** the south main aisle (solid wall on one side, solid row A on the other) —
  all three runs have their largest corrected error (0.10–0.16 m) on the final return along it;
  scan matching constrains x poorly in a featureless corridor, and the return produced no loop
  closure near the start. The map shows slightly blurred / doubled edges (a few cm) only at the
  west wall and the pallet stack, i.e. the places seen at the start and again at the end; no gross
  map doubling.
- Corrected error grows slowly while new areas are explored (up to ~0.1 m before the first loop
  closure at ~720 s), then the closure pulls it down by ~5 cm.

```
$ colcon build --symlink-install
Summary: 3 packages finished
$ ament_flake8 src/gnssd_tools
17 files checked   (0 errors)
$ ament_pep257 src/gnssd_tools
No problems found
$ colcon test --packages-select gnssd_tools && colcon test-result --verbose
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```

## Step 5 — sensor_monitor_node, lidar dropout gate, dropout scenario (2026-10-25)

Code commit for all runs: `71e8b64` (clean tree, `allow_dirty: false`). Fixes made after the runs
(shutdown, TF hold, rate warm-up; see "Fixes after the runs") do not change the recorded data;
all numbers below were computed with the fixed `tf_to_tum.py`. System configuration (ekf.yaml,
slam_toolbox.yaml, noise models, URDF) unchanged (`git diff 9023ec4` on these files is empty).

### sensor_monitor_node (system)
- Subscribes /odom, /imu, /scan with the sensor-data QoS (best effort; compatible with the
  reliable publishers of the stand). Arrival time = node clock (use_sim_time) in the callback.
- State machine in `gnssd_tools/sensor_health.py` (plain Python, no ROS): WAITING / OK / LOST,
  each transition returned once; WARN on OK->LOST (with the silence), INFO on LOST->OK (with the
  outage duration). 10 Hz check timer.
- Operator report: diagnostic_msgs/DiagnosticArray on /diagnostics (1 Hz and immediately on every
  transition), one status per sensor: level OK / WARN (rate below 50 % of expected) / ERROR (LOST)
  / STALE (WAITING), message, key-values last_age_s, timeout_s, expected_rate_hz,
  measured_rate_hz. Standard ROS health channel -> rqt_robot_monitor / `ros2 topic echo`.
- `config/sensor_monitor.yaml`: timeout = 5 x nominal period (odom 0.10 s, imu 0.05 s, scan
  0.50 s): five missing messages cannot be normal jitter, yet the loss is reported within
  timeout + 0.1 s. Launch arg `sensor_monitor:=true`; added to resource_logger's system list.
- Start-up fix found while testing: before the first /clock message the node clock reads 0, so
  early messages got time 0 and were reported LOST 1 s later; nothing is evaluated while the
  clock is 0.
- Unit tests `test/test_sensor_health.py`:
```
$ python3 -m pytest -q test/test_sensor_health.py
7 passed   (start, no message ever, timeout -> LOST once, recovery with outage duration,
            measured rate, rate known only after a full window, clock jump back)
```

### scan_gate_node (stand)
Bridge publishes /scan_raw; the gate relays it unchanged (same header, reliable depth 10 like the
bridge) to /scan except inside `scan_dropout` windows (YAML list of [t_start, t_end] in sim time,
t_end = -1 = until the end). Always in the chain (no windows = pass-through). Time base = the scan
header stamp, so the gate runs WITHOUT use_sim_time (documented exception, see below).
Pass-through measurement (probe subscribed to /scan_raw and /scan, wall-clock receive times per
header stamp, 40 s, first 5 s skipped):
```
added latency: mean 0.509 ms, median 0.433 ms, p95 1.029 ms, max 2.005 ms   (199 scans)
gate CPU (1 core = 100 %): mean 0.75 %, max 1.0 %; RSS 60.6 MiB
```
First version with use_sim_time: 45 % of a core — all of it from /clock (see next section).
run_info.yaml stores `scan_dropout_windows`. bags/index.csv quotes extra_args (they contain commas).

### /clock cost of Python nodes (simulation artifact)
Gazebo publishes /clock at 1 kHz; every rclpy node with use_sim_time handles all of it:
```
idle rclpy node with use_sim_time only:  43.1 % of one core (mean, 30 s)
sensor_monitor_node:                     59.8 % of one core
scan_gate_node (no use_sim_time):         0.7 %
```
So ~43 of sensor_monitor_node's ~60 % is /clock handling, which does not exist on the real robot;
its own work is ~17 % of a core. Resource tables below show the measured value; the
"without /clock" figure is measured minus 43 %. (Alternative for discussion: let the monitor use a
steady clock, which is what it would use on the robot.)

### Pass-through check: room_loop, seed 1 (gate in the chain, no windows)

| estimate | RMSE | mean | max | heading RMSE |
|---|---|---|---|---|
| /odom | 0.478 m | 0.399 m | 1.010 m | 7.91 deg |
| /odometry/filtered | 0.078 m | 0.070 m | 0.121 m | 0.31 deg |
| TF map->base_link | 0.013 m | 0.012 m | 0.036 m | 0.15 deg |

Step 4 seed 1 (no gate): 0.477 / 0.077 / 0.029 m; earlier seed-1 test runs gave 0.0125 and 0.0138 m
for the corrected estimate -> within run-to-run noise. 3 loop closures. Node census identical at
t = 5 s and t = 215 s.

### Dropout scenarios
room_dropout: route_room, 2 laps, `scan_dropout:=[[140,200]]` (60 s in the second lap; at 140 s the
robot is at ~(5.7, 0) moving east, at 200 s on the north side heading west).
warehouse_dropout: route_warehouse, `scan_dropout:=[[285,405]]` (120 s from just before the long
repetitive aisle 2 (entered at ~293 s) to its end).

#### Detection (all 6 runs identical)
| event | time |
|---|---|
| gate CLOSED | t1 = 140.00 s (room) / 285.00 s (warehouse) |
| sensor_monitor WARN "scan LOST" | t1 + 0.50 s, silence 0.60 s (last scan at t1 - 0.1 s; timeout 0.5 s + check period 0.1 s) |
| gate OPEN | t2 = 200.00 s / 405.00 s |
| sensor_monitor INFO "scan OK again" | t2 + 0.00–0.01 s, "outage of 60.10 s" / "120.10 s" |

/diagnostics excerpt (room_dropout seed 1, "sensor_monitor: scan"):
```
t= 140.00  level=OK     message="OK"                          last_age_s=0.096
t= 140.50  level=ERROR  message="LOST: no message for 0.60 s"  last_age_s=0.596
t= 199.00  level=ERROR  message="LOST: no message for 59.10 s" last_age_s=59.096
t= 200.00  level=WARN   message="rate low: 0.0 Hz of 10.0 Hz"  last_age_s=0.000
t= 201.00  level=OK     message="OK"                          last_age_s=0.097  measured_rate_hz=10.0
```
(The one "rate low" right after recovery — rate window not filled yet — is removed by the
rate warm-up fix below.)

#### Node liveness (FR-14, NFR-8)
In all 6 dropout runs: no system node exited or restarted; node census lines at t = 5 s and at the
last line before shutdown (215 s / 890 s) contain the same nodes (ekf_filter_node, slam_toolbox,
sensor_monitor_node and all stand nodes). The only difference ever seen was the ros2 CLI daemon
(`/_ros2cli_daemon_*`, left over from run_batch's node check) exiting after idling — not a project
node. /odometry/filtered during the outage: 1800 msgs / 60 s = **30.00 Hz** (room, all seeds),
3600 msgs / 120 s = **30.00 Hz** (warehouse, all seeds).

#### Accuracy before / during / after the outage (ATE, not aligned)

| run | estimate | before RMSE | during RMSE / max | after RMSE / max | re-convergence (< 0.1 m for 5 s) |
|---|---|---|---|---|---|
| room s1 | base | 0.063 m | 0.101 / 0.120 m | 0.057 / 0.086 m | |
| | corrected | 0.014 m | 0.084 / **0.126** m | 0.017 / 0.119 m | **0.2 s** |
| room s2 | base | 0.088 m | 0.143 / 0.185 m | 0.035 / 0.066 m | |
| | corrected | 0.014 m | 0.092 / **0.147** m | 0.029 / 0.147 m | **0.2 s** |
| room s3 | base | 0.059 m | 0.086 / 0.128 m | 0.082 / 0.116 m | |
| | corrected | 0.020 m | 0.062 / **0.088** m | 0.021 / 0.068 m | **0.0 s** (never above) |
| wh s1 | base | 0.257 m | 0.287 / 0.383 m | 0.436 / 0.726 m | |
| | corrected | 0.107 m | 0.114 / **0.203** m | **9.411 / 17.451 m** | **not recovered** |
| wh s2 | base | 0.280 m | 0.225 / 0.406 m | 0.355 / 0.567 m | |
| | corrected | 0.067 m | 0.264 / **0.449** m | **5.811 / 10.946 m** | **not recovered** |
| wh s3 | base | 0.248 m | 0.277 / 0.555 m | 0.766 / 1.373 m | |
| | corrected | 0.050 m | 0.092 / **0.198** m | **5.520 / 10.059 m** | **not recovered** |

Whole run (corrected): room 0.047 / 0.053 / 0.038 m RMSE; warehouse 7.02 / 4.33 / 4.12 m RMSE.
Loop closures: room 1 / 0 / 0, warehouse 0 / 1 / 1 (the warehouse closures at ~718 s are made
against the corrupted map).

During the outage map->odom stays frozen (e.g. warehouse s1: (0.138, 0.301, -0.6 deg) from 285.1 s
until 405.2 s), so the corrected estimate follows the base layer (its error grows by the base
layer's drift, max 0.09–0.45 m).

#### Failure: no re-localisation after the warehouse outage
At the first scan after the outage slam_toolbox registers it at a wrong place and map->odom jumps:
```
warehouse s1: map->odom  t=285.1 (0.138, 0.301, -0.6 deg)  ->  t=405.2 (10.222, -13.485, 39.4 deg)
warehouse s3: map->odom  t=284.0 (0.200, -0.338, 1.8 deg)   ->  t=405.3 (5.218, -9.339, 25.7 deg)
```
and stays there (s1 at 800 s: (10.0, -13.5, 38.8 deg)). The map built after 405 s is a second,
rotated (~40 deg) copy of the warehouse (`docs/img/step5_warehouse_dropout_seed1_map.png`).
Interpretation: during 120 s the robot moved ~24 m along the repetitive aisle into an area the map
had barely seen; the first new scan resembles several places of the warehouse and the scan matcher
picks a wrong one (perceptual aliasing). Not tuned (as required). In the room (60 s, second lap,
known area) the corrected error falls back below 0.1 m within 0.2 s.
Figures: `docs/img/step5_warehouse_dropout_seed1_error.png` (base vs corrected over time, outage
shaded), `docs/img/step5_room_dropout_seed1_error.png`, `docs/img/step5_warehouse_dropout_seed1_map.png`.

### Resources (system processes, 8 cores)

| run | process | CPU 1-core mean / max | RSS mean / max |
|---|---|---|---|
| room_loop s1 | ekf_filter_node | 14.4 / 16.0 % | 28.5 / 29.0 MiB |
| | async_slam_toolbox_node | 13.7 / 36.0 % | 46.6 / 50.8 MiB |
| | sensor_monitor_node | 62.3 / 66.0 % (≈19 % without /clock) | 62.4 / 62.4 MiB |
| | **system** | **90.4 / 111.0 %** (11.3 / 13.9 % of 8 cores) | **137.5 / 142.1 MiB** |
| room_dropout s1–s3 | system | 88.1–90.4 / 99–106 % | 137.1–137.6 / 138–139 MiB |
| warehouse_dropout s1–s3 | system | 87.4–88.7 / 100–109 % | 142.5–143.2 / 148.7–149.6 MiB |

(sensor_monitor_node 61–63 % mean, RSS 62 MiB in every run; ~43 % of it is /clock handling.)

### Fixes after the runs
1. **Clean shutdown, second iteration.** With rclpy's handler off, Python's default SIGINT
   handler can raise KeyboardInterrupt inside a pybind11 call -> the same RuntimeError
   (node_census_node in room_dropout s1), and scan_gate_node (no timers, no /clock) waited for a
   scan that never came and was SIGTERM-ed after 5 s. `node_runner.py` now installs its own
   SIGINT/SIGTERM handler that only sets a flag; nodes spin with spin_once(timeout 0.1 s) and check
   it. route_driver_node and run_info_writer_node use the same flag. Check (3 start/stop cycles of
   sim.launch.py with driving): tracebacks 0, escalations 0, processes died 0, "finished cleanly"
   11 of 11 in each cycle.
2. **tf_to_tum.py zero-order hold.** slam_toolbox re-sends map->odom with the stamp of the last
   scan while no scans arrive; the script interpolated across the gap and smeared the post-outage
   jump back over the outage (warehouse s1 "during" error 3.6 m instead of 0.20 m). Gaps longer
   than 1 s are now held. Normal runs are unaffected (map->odom stamps every 0.10 s, no gap > 1 s;
   room_loop s1 and warehouse_loop s1 ATE identical to 6 digits).
3. **Rate warm-up in sensor_monitor.** "rate low" is reported only after the sensor has been OK
   for a full rate window (2 s); removes the transient WARN at start-up and after recovery.

```
$ colcon build --symlink-install
Summary: 3 packages finished
$ ament_flake8 src/gnssd_tools
24 files checked   (0 errors)
$ ament_pep257 src/gnssd_tools
No problems found
$ colcon test --packages-select gnssd_tools && colcon test-result --verbose
Summary: 9 tests, 0 errors, 0 failures, 0 skipped
```

## Step 5b — Root cause of the post-outage failure, sensor_monitor clock (2026-10-25)

Commits: `f176b79` (Step 5 state; diagnostic run), `05646e9` (sensor_monitor steady clock, RTF check).
ekf.yaml, slam_toolbox.yaml and the noise models are unchanged (the diagnostic run flipped only
`debug_logging`, see below, and it was reverted: `git diff` empty).

### 1. Root cause (evidence)

**Source (slam_toolbox tag 2.6.10, `lib/karto_sdk/src/Mapper.cpp`):**
- `Mapper::Process`: the new scan's pose is predicted from the last processed scan
  (`Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());`
  `pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));`), then
  `if (m_pUseScanMatching->GetValue() && pLastScan != NULL)` it is matched with
  `m_pSequentialScanMatcher->MatchScan(pScan, ...GetRunningScans(...), bestPose, ...)` and
  `pScan->SetSensorPose(bestPose);` — **the response returned by MatchScan is discarded; the pose
  is accepted whatever the match quality** (minimum responses exist only for near-chain links and
  loop closures).
- Running scans (`AddRunningScan`): the last `scan_buffer_size` (10) scans, trimmed so that the
  first and last are at most `scan_buffer_maximum_scan_distance` (10 m) apart.
- `ScanMatcher::MatchScan`: search centred at the predicted pose; coarse window
  `correlation_search_space_dimension` 0.5 m (±0.25 m) and `coarse_search_angle_offset` 0.349 rad
  (±20°). **Response expansion**: `if (m_pUseResponseExpansion ...) { if (math::DoubleEqual(bestResponse, 0.0)) { ... newSearchAngleOffset += DegreesToRadians(20); ... (up to 3 times)`
  i.e. with a zero response the angular window grows to ±40°, ±60°, ±80°. Its message
  ("Mapper Info: Expanding response search space!") is inside `#ifdef KARTO_DEBUG`, which is commented
  out in 2.6.10 — the installed library contains no such strings (`strings libkartoSlamToolbox.so`),
  so `debug_logging: true` cannot show it (it only prints Ceres reports).
- Loop closure (`TryCloseLoop` -> `CorrectPoses()`) is the only path that optimises the graph.

**Diagnostic run** (warehouse_dropout seed 1, `debug_logging: true`, `allow_dirty: true`, otherwise
identical to `f176b79`): 2 Ceres reports in the whole run, none at 405 s; map->odom again jumped at
the first scan after the outage: t = 285.1 (0.045, 0.240, -0.6°) -> t = 405.2 (2.725, -6.298, 15.4°).
Same seed, different angle than in Step 5 (40°): the outcome depends on timing, i.e. on noise.

**Pose graph** (Step 5 bag, warehouse_dropout s1, /slam_toolbox/graph_visualization):
```
graph before: t=399.2, 62 vertices; after: t=409.0, 63 vertices
largest shift of an OLD vertex between the two graphs: 0.0000 m     -> no optimisation, no loop closure
last vertex before the outage: id 61 at (1.13, 5.63)
new vertex id 62: map pos (23.90, 7.55); edges to vertices [61]     -> sequential link only
true robot position at 405 s: (24.30, 7.50)
vertices 62..69: (23.90,7.55) (24.62,8.17) (24.81,9.07) (24.27,9.79) (23.65,10.48) (22.94,11.04) ...
  -> the chain turns ~40° too early: position of vertex 62 right, its heading wrong
```

**Overlap of the first post-outage scan with the running buffer** (world frame via ground truth,
last 10 scans before t1 vs the first scan at t2; points within 0.10 m):

| run | distance last scan -> first new scan | overlap |
|---|---|---|
| warehouse_dropout s1–s3 | 23.3 m | **0.3 %** (1 of 349 points) |
| room_dropout s1–s2 / s3 | 7.0 / 6.9 m | **39.7 % / 36.9 %** |

**Quantification (warehouse):**

| seed | outage path | EKF displacement (true) | EKF error at 405 s | map->odom jump |
|---|---|---|---|---|
| 1 | 24.0 m | 23.02 m (23.32 m) | 0.38 m, 1.06° | 17.08 m, **40.0°** |
| 2 | 24.0 m | 23.69 m (23.32 m) | 0.40 m, 0.78° | 10.78 m, **24.0°** |
| 3 | 24.0 m | 23.11 m (23.32 m) | 0.56 m, -2.57° | 10.30 m, **23.9°** |

The translation part of the jump is the rotation about the map origin: 2·24 m·sin(θ/2) = 16.4 m for
40°, 10.0 m for 24° — the corrected *position* at 405 s was almost right, the *heading* was not.

**Explanation.** During 120 s the robot drove 24 m; the graph got no vertex. The first new scan is
matched only against the running buffer (scans taken 23 m back, around (1, 6)), with which it shares
0.3 % of its points, so the correlation response is ~0. The best pose is then decided by noise inside
the ±20° window, and when the response is exactly 0 the window is widened to ±40…80°
(`use_response_expansion: true`); 24° and 40° are only reachable through the expansion, 15°
(diagnostic run) also without it. Because `Mapper::Process` accepts the matched pose regardless of its
response, a heading error of 15–40° enters the graph as a sequential edge (to vertex 61) and
every later vertex, scan and map cell is built rotated — the second, rotated copy of the warehouse
in the Step 5 map. No loop closure is involved (no vertex moved). Parameters that allowed it:
`scan_buffer_size` 10 / `scan_buffer_maximum_scan_distance` 10 m (buffer cannot reach back 23 m),
`coarse_search_angle_offset` 0.349 (±20°), `use_response_expansion` true (up to ±80°), and the absence
of any acceptance threshold for sequential matches. The room recovered because after 60 s the robot
was only 7 m from the last scans in a 10 x 8 m room the lidar sees almost entirely: 37–40 % overlap,
so the match is well defined.

### 2. sensor_monitor_node on the steady clock
Arrival times and timers on the monotonic clock (`time.monotonic()`, `ClockType.STEADY_TIME`),
`use_sim_time: false`. Real-time factor check: every 10 s one /clock message is taken (subscribe,
one message, unsubscribe) and RTF = Δsim/Δsteady between two samples; WARN once if |RTF-1| > 5 %
(`RealTimeFactorCheck` in sensor_health.py, unit-tested); RTF also published as a key-value.
Log lines no longer contain sim time; detection latency is measured from the ROS log stamps.

```
$ python3 -m pytest -q test/test_sensor_health.py      -> 8 passed (adds the RTF check test)
```

| sensor_monitor_node | CPU 1-core mean / max | RSS |
|---|---|---|
| Step 5 (use_sim_time, 1 kHz /clock) | 59.8–62.3 / 66 % | 62.4 MiB |
| Step 5b (steady clock), 30 s probe while driving | **11.1 / 12.0 %** | 62.1 MiB |
| Step 5b, room_dropout s1 rerun (resource_logger) | **13.3 / 15.0 %** | 62.5 MiB |

room_dropout seed 1 rerun at `05646e9`:
```
[scan_gate_node] Scan gate CLOSED (lidar dropout) at t = 140.00 s
[sensor_monitor_node] Sensor scan LOST: no message for 0.57 s (timeout 0.5 s)
[scan_gate_node] Scan gate OPEN (lidar back) at t = 200.00 s
[sensor_monitor_node] Sensor scan OK again after an outage of 60.84 s     (60.1 s sim at RTF ~0.99)
detection latency (log stamps): LOST 0.469 s after the gate closed; OK 0.001 s after it opened
real_time_factor in /diagnostics: 202 reports, min 0.975, max 0.993  (no RTF warning)
tracebacks 0, escalations 0, died 0; node census identical t = 5 s / 215 s; /odometry/filtered 30.00 Hz
corrected: before 0.012 m RMSE; during 0.081 / max 0.119 m; after 0.016 m; re-convergence 0.2 s
system CPU (ekf + slam + monitor): 40.1 / 58.0 % of one core (Step 5: 88–90 / 99–111 %)
```

### 3. /clock share of C++ nodes
`demo_nodes_cpp listener` (subscribed to an unused topic, otherwise idle) next to the running
simulation, 30 s, measured by the PID of the binary:
```
listener use_sim_time=true:  CPU 5.73 % (max 7.0 %), RSS 23.5 MiB
listener use_sim_time=false: CPU 0.00 %,             RSS 23.4 MiB
```
So ~5.7 % of one core of ekf_filter_node and of async_slam_toolbox_node (both C++, use_sim_time) is
/clock handling that does not exist on the real robot (Python: ~43 %).

### 4. Mitigation options (proposed, not implemented)

| option | where | what it does | warehouse case | normal runs | effort | ch. 2 architecture |
|---|---|---|---|---|---|---|
| **a) consistency check** | system node (extension of the monitoring function, or map_manager) | after a sensor outage (LOST->OK from sensor_monitor) compare the first change of map->odom with the uncertainty the base layer accumulated during the outage (EKF covariance from /odometry/filtered, e.g. 3σ + floor); on violation: ERROR on /diagnostics and mark the corrected estimate invalid (operator / Nav2 may fall back to the base layer) | detects all three runs (24–40° heading change vs EKF heading σ of a few degrees); does not stop the map from being corrupted | in-run corrections are ≤ 2 cm / < 1° -> no false alarm with a sensible floor | medium (new logic + thresholds, uses existing topics) | **yes**: observes only, like sensor_monitor |
| **b) slam_toolbox parameters** | slam_toolbox.yaml | `use_response_expansion: false` bounds a blind match to ±20°; larger `scan_buffer_maximum_scan_distance` / `scan_buffer_size` let the buffer reach further back | reduces the worst case (40° -> ≤ 20°) but a 15° wrong heading (diagnostic run) still occurs; the buffer cannot help when the new area was never mapped | expansion is a recovery feature for normal runs, disabling it may cost robustness after fast turns; must re-verify Steps 4/4b | low (config) + full re-verification | yes (parameters only) |
| **c) pause while LOST** | sensor_monitor -> slam_toolbox `/slam_toolbox/pause_new_measurements` service | stop feeding scans while the lidar is LOST; on resume keep the corrected estimate frozen until a scan overlaps the existing map (e.g. response / match against nearby graph nodes, Karto `ProcessAgainstNodesNearBy`), then resume mapping | prevents the rotated copy only if resumption waits for overlap; slam_toolbox 2.6 offers no "resume with constrained relocalisation" in mapping mode, so this needs custom code around it | no effect without outages | high | partly: requires sensor_monitor to act on slam_toolbox (it currently only observes) |
| **d) localisation on the saved map** | map_manager | at LOST save the map (serialize_map); after a long outage restart slam_toolbox in localization mode with the base-layer pose as initial pose | keeps the map uncorrupted; but the robot is in an area the map barely covers, so localisation has little to match until it re-enters mapped space | none without outages | high (mode switching, restarts) | yes (map_manager role), but changes the mapping workflow |

Recommendation: (a) first — cheap, purely observing, directly addresses FR-14/NFR-8 by making the
failure visible; then evaluate (b) `use_response_expansion: false` as a separate, fully re-verified
configuration change. (c)/(d) prevent the corruption but require control over slam_toolbox's modes.

```
$ colcon build --symlink-install
Summary: 3 packages finished
$ ament_flake8 src/gnssd_tools
24 files checked   (0 errors)
$ ament_pep257 src/gnssd_tools
No problems found
$ colcon test --packages-select gnssd_tools && colcon test-result --verbose
Summary: 10 tests, 0 errors, 0 failures, 0 skipped
```

## Step 5c — Correction consistency check (2026-10-25)

Commit for the live runs: `728b69c` (clean tree). ekf.yaml, slam_toolbox.yaml and the noise models
are unchanged.

### Design decision: drift bound from the sensor model, not from the EKF covariance
The base-layer pose covariance on /odometry/filtered is unusable as an uncertainty bound with the
default (untuned) process noise and no absolute measurement:
```
warehouse_dropout s1 /odometry/filtered pose covariance:
t=  10 s: sigma_yaw  44.4 deg, sigma_x 0.7 m,    sigma_y 7.7 m
t= 285 s: sigma_yaw 236.9 deg, sigma_x 4963 m,   sigma_y 18521 m
t= 405 s: sigma_yaw 282.4 deg
growth of sigma_yaw: 4.44 deg per map->odom update (0.1 s), 153.8 deg over the 120 s outage
```
A "k·sigma" threshold would be >= 154° after the outage (wrong corrections are 16–40°) and >= 4.4°
per normal update (legitimate ones are <= 0.67°): the two sets cannot be separated with this input.
Decision (agreed with the user): bound the heading the base layer can drift by its gyro model,
sigma_drift = sqrt(imu_gyro_bias_stddev^2 + imu_gyro_dynamic_bias_stddev^2) = 1.005e-4 rad/s, read
from robot_params.xacro (no duplicated numbers). Translation is reported but not tested: the shift of
the corrected position at a wrong update (0.09–0.36 m) overlaps with legitimate corrections (<= 0.15 m).

### Detector (`gnssd_tools/correction_consistency.py`, plain Python)
For every NEW map->odom update (new stamp; slam_toolbox's repeats of the same stamp are ignored):
```
change  = |yaw(map->odom) - yaw(reference)|,  reference = newest accepted state >= window old
                                                 (or the last accepted one after a gap)
allowed = floor + k * sigma_drift * dt          dt = time between the two
INVALID  if change > allowed   (latched)
```
Parameters (config/sensor_monitor.yaml): k = 3, floor = 3.0°, window = 1.0 s ->
3.02° for a normal update, 5.07° after a 120 s gap.
Latch: INVALID stays until the operator calls `~/reset_correction_check` (std_srvs/Trigger).
Justification: once a wrong correction is in slam_toolbox's graph, the map and every later correction
are built on it (Step 5 map: rotated copy of the warehouse), so the estimate cannot become right by
itself; an automatic clear would hide the failure. A future map_manager action (restart / relocalise)
is the natural place to trigger the reset.
The check does not use the lidar-outage signal; sensor_monitor only adds to the ERROR message whether
a scan LOST->OK happened within `outage_memory` (30 s).

Unit tests (`test/test_correction_consistency.py`):
```
6 passed: valid run with 0.2° quantised updates; slow drift (0.2°/s) accepted; legitimate 0.6° loop
closure accepted; 24° jump after 120 s detected (allowed 5.06°); INVALID latched until reset (then
the new heading is the reference); repeated stamp ignored
```

### Offline validation on all recorded bags (`scripts/check_corrections.py`, k 3, floor 3°, window 1 s)

| bag | updates | largest accepted change / allowed (ratio) | alarms |
|---|---|---|---|
| loop_seed1_20261024_122207 (room_loop) | 2176 | 0.400° / 3.02° (0.133) | 0 |
| loop_seed2_20261024_122604 | 2178 | 0.400° / 3.02° (0.133) | 0 |
| loop_seed3_20261024_123027 | 2167 | 0.400° / 3.02° (0.133) | 0 |
| long_seed1_20261024_123426 (room_long) | 10648 | 0.400° / 3.02° (0.133) | 0 |
| warehouse_loop s1 / s2 / s3 | 8949 / 8949 / 8931 | 0.584° / 0.508° / 0.400° vs 3.02° (0.194 / 0.168 / 0.133) | 0 / 0 / 0 |
| room_loop_seed1_20261025_120310 (Step 5 pass-through) | 2167 | 0.400° / 3.02° | 0 |
| room_dropout s1 / s2 / s3 (Step 5) | 1558 / 1567 / 1577 | 0.400° / 3.02° | 0 / 0 / 0 |
| room_dropout_seed1_20261025_190550 (Step 5b) | 1567 | 0.400° / 3.02° | 0 |
| **warehouse_dropout s1** | 7739 | (0.200° before the alarm) | **1: t = 405.2 s, 39.99° > 5.07° (dt 120.1 s)** |
| **warehouse_dropout s2** | 7739 | (0.200°) | **1: t = 405.2 s, 24.03° > 5.07°** |
| **warehouse_dropout s3** | 7739 | (0.400°) | **1: t = 405.2 s, 23.94° > 5.07°** |
| **warehouse_dropout_debug s1** (Step 5b) | 7749 | (0.200°) | **1: t = 405.2 s, 16.00° > 5.07°** |

(Step 3 bags have no correction layer and are skipped.)
Margins: largest legitimate change 0.584° vs threshold 3.02° -> **5.2x** below; smallest wrong
correction 16.0° vs threshold 5.07° -> **3.2x** above. Every failure is caught at the first update
after the outage; no false alarm in 12 normal runs (incl. the room outages, whose first post-outage
change was 0.0–0.2°).

### Integration in sensor_monitor_node (still observing only)
Subscribes /tf (map->odom) and /odometry/filtered (base pose, for the reported position shift).
Publishes /correction_valid (std_msgs/Bool, transient local; true at start, false on INVALID, true
after a reset) and the /diagnostics status "sensor_monitor: correction consistency" (OK "VALID" /
ERROR "INVALID since …" with state, last_heading_change_deg, last_position_shift_m, floor_deg, k,
drift_sigma_rad_s, alarm_heading_change_deg, alarm_allowed_deg). Live smoke test: /correction_valid
True, status VALID, reset service answers.

### Live runs at `728b69c`
warehouse_dropout seed 1:
```
[INFO]  [1792949892.662] scan_gate_node: Scan gate CLOSED (lidar dropout) at t = 285.00 s
[WARN]  [1792949893.093] sensor_monitor_node: Sensor scan LOST: no message for 0.53 s (timeout 0.5 s)
[INFO]  [1792950015.199] scan_gate_node: Scan gate OPEN (lidar back) at t = 405.00 s
[INFO]  [1792950015.201] sensor_monitor_node: Sensor scan OK again after an outage of 122.64 s
[ERROR] [1792950015.345] sensor_monitor_node: Correction INVALID at t = 405.30 s: map->odom heading
        changed by 39.99 deg in 120.2 s, allowed 5.08 deg (corrected position moved 0.22 m);
        preceded by a lidar outage
first map->odom heading jump > 3°: stamp 405.30 s, 39.99°, received (wall) 1792950015.344
/diagnostics "correction consistency": VALID from the start; ERROR "INVALID since t = 405.30 s:
heading jump 40.0 deg > allowed 5.1 deg" 0.002 s after the jump was received; INVALID until the end
(924 reports). /correction_valid: true -> false at 1792950015.345 (log; topic not recorded).
```
Alarm latency: 1 ms after the wrong map->odom update arrived (0.30 s after the lidar came back:
the first scan processed after the outage).
room_dropout seed 1: no map->odom heading change > 3°, status VALID in all 229 reports, no alarm;
scan LOST/OK detected as before (LOST 0.45 s after the gate closed, OK 0.001 s after it opened).
Both runs: tracebacks 0, escalations 0, processes died 0.

### Resources of sensor_monitor_node

| version | CPU 1-core mean / max | RSS |
|---|---|---|
| Step 5 (use_sim_time) | 59.8–62.3 / 66 % | 62.4 MiB |
| Step 5b (steady clock) | 11.1–13.3 / 15 % | 62.1–62.5 MiB |
| Step 5c smoke test (30 s probe, + /tf and /odometry/filtered) | 19.4 / 20.0 % | 63.2 MiB |
| Step 5c recorded runs (resource_logger) | **24.5–24.9 / 28 %** | 63.6–63.7 MiB |

The extra ~8–12 % is the Python handling of /tf (map->odom 50 Hz + odom->base_link 30 Hz +
robot_state_publisher) and /odometry/filtered (30 Hz). System total (ekf + slam + monitor):
51.9–52.7 / 67–72 % of one core.

Open item (done in Step 6): /correction_valid is not in record.launch's topic list (the timeline above comes from the
log and /diagnostics); add it when the recording profile is next changed.

```
$ colcon build --symlink-install
Summary: 3 packages finished
$ ament_flake8 src/gnssd_tools
27 files checked   (0 errors)
$ ament_pep257 src/gnssd_tools
No problems found
$ colcon test --packages-select gnssd_tools && colcon test-result --verbose
Summary: 16 tests, 0 errors, 0 failures, 0 skipped
```

## Step 6 — map_manager_node (2026-10-25)

Commits: code b53b1f0, fix 3dc6fe9 (one checkpoint per scan loss). ekf.yaml, noise models and
slam_toolbox.yaml (mapping) unchanged; slam_toolbox not tuned.

### What was added
- `gnssd_interfaces`: SaveMap (name), SetMode (mode, map_name, x, y, yaw, use_base_pose), GetStatus.
- `map_manager_node` (SYSTEM): `~/save_map`, `~/set_mode`, `~/status`; start-up mode/map/pose from
  launch args (FR-12); runs slam_toolbox as a child process (`slam_mapping.launch.py` /
  `slam_localization.launch.py`) in its own process group, stop = SIGINT -> SIGTERM -> SIGKILL;
  after every switch calls `/sensor_monitor_node/reset_correction_check`; events on
  `/map_manager/events` (recorded, transient local). Optional `checkpoint_on_scan_lost` (default false).
- `config/slam_toolbox_localization.yaml`: package `mapper_params_localization.yaml` with 4 changes,
  each documented in the file header: use_sim_time true (added), mode mapping -> localization
  (package file is a template), base_frame base_footprint -> base_link, max_laser_range 20 -> 12
  (lidar model). map_file_name / map_start_pose are passed at launch.
- `config/map_manager.yaml`: service_timeout 30 s, grid_save_timeout 10 s, stop_timeout 10 s.
- run.launch.py: `route`, `spawn_*`, `slam_mode`, `slam_map`, `slam_x/y/yaw`,
  `checkpoint_on_scan_lost`, `save_map_as` (save when the route is done), `operator_recovery`
  (scripted_operator_node, STAND). record.launch: + /correction_valid, /map_manager/events.
  resource_logger: + map_manager_node, localization_slam_toolbox_node. `maps/` is git-ignored
  (run outputs, like bags; otherwise the clean-tree guard fails after the first save).

Problems found while testing:
- slam_toolbox's `/slam_toolbox/save_map` starts `map_saver_cli` with a 2 s timeout -> "Failed to
  spin map subscription", no .pgm/.yaml. map_manager runs `map_saver_cli` itself with
  `save_map_timeout` 10 s (measured: ~3 s needed).
- map_manager with use_sim_time: 81 % of one core (1 kHz /clock). It needs no ROS clock (compares
  message stamps only) -> use_sim_time false, like scan_gate/sensor_monitor: 14.5 % in a probe.
- localization_slam_toolbox_node refuses serialize_map ("Cannot call serialize map in localization
  mode!") but answers result=0 (= RESULT_SUCCESS) -> map_manager checks the files, not the code.
- Two /diagnostics callbacks in parallel (multi-threaded executor) both saw "scan just LOST" -> the
  checkpoint was serialized twice (run 211640). Fixed with a lock (3dc6fe9); re-run: one checkpoint.

Smoke test (operator commands, headless sim):
```
save_map {name: smoke}            -> success=True, files .posegraph .data .pgm .yaml
save_map {name: ../bad}           -> success=False 'Invalid map name "../bad"'
set_mode localization nope        -> success=False 'Map "nope" not found ... (need .posegraph and .data)'
set_mode localization smoke       -> success=True 'localization on smoke'
save_map {name: smoke2} (loc.)    -> success=False 'Saving is only possible in mapping mode (now: localization)'
set_mode mapping                  -> success=True; one slam_toolbox process (async) afterwards
[event] mode mapping -> localization (operator); previous slam stopped by SIGINT
[event] mode localization -> mapping (operator); previous slam stopped by SIGINT
use_base_pose while driving: initial pose (-0.033, 3.300, -93.8 deg); 15 s later map->base_link
(0.908, -0.056) vs ground truth (0.894, -0.057)
```

### B. Verification runs (ATE without alignment; corrected = TF map -> base_link)

| run (bag) | mode | corrected RMSE / max | base RMSE |
|---|---|---|---|
| room_loop_seed1_20261025_203250, save_map_as:=room_s1 | mapping | 0.011 / 0.030 m | 0.077 m |
| room_loc_seed2_20261025_203708 (room_s1, start origin) | localization | 0.016 / 0.038 m | 0.106 m |
| room_loc_east_seed3_20261025_204103 (route room_east, spawn = initial pose (7, 2, 90°)) | localization | 0.010 / 0.032 m | n/a (odom starts at 0) |
| warehouse_loop_seed1_20261025_204547, save_map_as:=wh_s1 | mapping | 0.077 / 0.137 m | 0.368 m |
| warehouse_loc_seed2_20261025_210114 (wh_s1) | localization | 0.072 / 0.141 m | 0.314 m |

B1 map: results/step6/room_s1.png (walls, island and obstacles sharp, no doubling), wh_s1.png.

```
== room_loop_seed1_20261025_203250 (events from the bag)
     0.82 s  mode stopped -> mapping (start-up); previous slam stopped by none; new pid 258766
   230.32 s  save map=room_s1 ok=True files=room_s1.posegraph,room_s1.data,room_s1.pgm,room_s1.yaml
== room_loc_seed2_20261025_203708                               (B5 / FR-12: started in localization)
     0.71 s  load map=room_s1 pose=(0.000, 0.000, 0.00 deg)
     0.71 s  mode stopped -> localization (start-up); previous slam stopped by none; new pid 259414
== room_loc_east_seed3_20261025_204103
     1.38 s  load map=room_s1 pose=(7.000, 2.000, 90.00 deg)
     1.38 s  mode stopped -> localization (start-up); previous slam stopped by none; new pid 259905
```

B4 / FR-13 (sha256 and mtime before vs after both room localization runs; same for wh_s1):
```
5bbf2c6a86a784fcf3dadbe49f6610395f2a79894ea5731a17abc48a42dfa5ba  maps/room_s1.data
a1b2c72296aade8043cd24cc230265d761c42057cfc898109724b72be764d24e  maps/room_s1.pgm
7dd9d2e4d765ad06323fc850224ce15f3db13e22eb89d63b5e0bad36aa199129  maps/room_s1.posegraph
3d366065a0e1bcfe1f2b8781a40dabb3646fa3df746dd18d515f6e9ef8137591  maps/room_s1.yaml
$ diff room_s1_hashes_before.txt room_s1_hashes_after.txt  -> HASHES IDENTICAL, MTIME IDENTICAL
$ diff wh_s1_hashes_before.txt wh_s1_hashes_after.txt      -> WH HASHES IDENTICAL
```
No save or serialize event in any localization bag (only `load` + `mode`).

### C. Checkpoint on scan loss + operator recovery (warehouse_dropout seed 1, outage 285–405 s)

Why the checkpoint is an automatic safety action and the switch is an operator command: saving
the graph when the lidar is LOST changes nothing in the running estimate and protects the map
before any post-outage scan can corrupt it (Step 5b) — it is cheap and always safe, so it can be
automatic (switchable, default off). Switching to localization replaces the correction layer and
its map; whether that is right depends on the situation (e.g. is the robot in a mapped area?),
so it stays an operator decision. In the test a scripted "operator" (stand) calls
`set_mode(localization, last_checkpoint, use_base_pose=true)` 3 s after /correction_valid = false.

Run warehouse_dropout_recovery_seed1_20261025_231235 (commit 3dc6fe9), timeline (sim time):
```
 285.44 scan LOST: no message for 0.55 s
 285.44 EVENT checkpoint requested map=checkpoint_1792963055 (lidar LOST)
 285.46 EVENT save map=checkpoint_1792963055 ok=True files=checkpoint_1792963055.posegraph,.data
 405.14 correction_valid False: INVALID since t = 405.30 s: heading jump 40.2 deg > allowed 5.1 deg
 408.25 EVENT load map=checkpoint_1792963055 pose=(24.725, 7.588, 0.49 deg)
 408.25 EVENT mode mapping -> localization (operator (initial pose from the base layer));
        previous slam stopped by SIGINT; new pid 275152
 408.25 correction_valid True   (reset after the switch)
 745.95 correction_valid False  (see below)
```

| | corrected after the outage (405–894 s) RMSE / max | re-convergence (< 0.1 m for 5 s) |
|---|---|---|
| no recovery, same seed (Step 5c, warehouse_dropout_seed1_20261025_193318) | 9.399 / 17.369 m | not recovered |
| recovery, run 211640 (b53b1f0, duplicate checkpoint) | 0.924 / 2.126 m | 339.1 s |
| **recovery, run 231235 (3dc6fe9)** | **0.820 / 1.586 m** | 341.0 s |

Initial pose error (base layer propagated from the last valid map->odom): 0.236 m, 0.49°
(run 211640: 0.252 m, 0.46°). Time from the alarm to localization running: 3.1 s (the scripted
operator's reaction delay).

Corrected error after the switch (run 231235): t=412 s 0.36 m / heading +3.15°; 450 s 0.57 m;
600 s 1.29 m; 744 s 0.22 m / +3.40°; 747 s 0.08 m / -0.19°; 890 s 0.008 m
(run 211640: heading -5.4° from 411 s to 744 s, then 0.04 m).

Finding: the initial pose is good, but localization locks onto a heading 3–5° off right after the
switch and keeps it until t ≈ 745 s. From 408 s to 740 s the robot drives 3.6–11 m away from any
pose of the checkpoint graph (that part of the warehouse was driven only during the outage), so
there is no correct map to match against; it snaps back as soon as it is within ~1 m of the
pre-outage path (745 s). The consistency check flags this correct snap as a heading jump (it
cannot tell a correction from a corruption) -> INVALID at 745.95 s; the scripted operator acts only
once. slam_toolbox was not tuned (Step 6 rule).

Node census (run 211640): /slam_toolbox listed twice at 415–425 s. resources.csv shows the two
processes never alive together (last async sample 2 s before the first localization sample): the
second name is the stale DDS discovery entry of the stopped node, gone within ~20 s. No orphan
processes after any switch or shutdown (smoke tests and all runs: `ros2 node list` empty, no
slam_toolbox / gz / gnssd_tools process left).

### Resources of map_manager_node (resource_logger, recorded runs)

| run | map_manager CPU 1-core mean / max | RSS |
|---|---|---|
| room_loop s1 (mapping) | 22.2 / 26.1 % | 62.9 MiB |
| room_loc s2 | 20.3 / 24.9 % | 62.8 MiB |
| warehouse_loop s1 | 20.1 / 25.0 % | 63.4 MiB |
| warehouse_dropout_recovery s1 (3dc6fe9) | 22.1 / 28.1 % | 64.0 MiB |

Mostly Python handling of /tf (93 Hz) and /odometry/filtered (30 Hz), as for sensor_monitor_node.
localization_slam_toolbox_node: 13.4–13.9 / 33–39 %, 52–57 MiB (async in mapping: 12.6–13.7 %).

```
$ colcon build --symlink-install
Summary: 4 packages finished
$ ament_flake8 src/gnssd_tools
31 files checked   (0 errors)
$ ament_pep257 src/gnssd_tools
No problems found
$ colcon test --packages-select gnssd_tools && colcon test-result --all
Summary: 22 tests, 0 errors, 0 failures, 0 skipped
```
