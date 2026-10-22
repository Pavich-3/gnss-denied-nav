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
Visual check (window shows shapes, rendering works): PENDING USER CONFIRMATION
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
