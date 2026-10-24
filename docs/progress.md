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
