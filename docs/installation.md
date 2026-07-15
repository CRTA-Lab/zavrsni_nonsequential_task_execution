# Installation

## Required software

The project was developed using:

- Ubuntu 22.04
- ROS 2 Humble
- Universal Robots ROS 2 Driver
- CoppeliaSim
- RoboDK
- Python 3
- colcon
- rosdep

The validated operating system is Ubuntu 22.04 with ROS 2 Humble. RoboDK may
also be used on Windows for the standalone RoboDK stages; the physical-robot
integration was validated on Ubuntu.

## Install ROS 2 build tools

Install ROS 2 Humble from the official ROS 2 apt repository, then install the
workspace tools:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-numpy \
  build-essential \
  cmake \
  git \
  ripgrep \
  tree
```

Initialize rosdep once per computer:

```bash
sudo rosdep init
rosdep update
```

If `rosdep` was already initialized, do not repeat `sudo rosdep init`; run only
`rosdep update`.

## Clone the repository

After the repository has been reviewed and approved, clone it with:

```bash
git clone https://github.com/CRTA-Lab/zavrsni_nonsequential_task_execution.git
cd zavrsni_nonsequential_task_execution
```

## Universal Robots ROS 2 Driver

The Universal Robots ROS 2 Driver and its dependencies must be installed and
built before building this project workspace.

In the original development setup, the driver workspace was located at:

```text
~/ur_ws
```

Source ROS 2 and the UR driver workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ur_ws/install/setup.bash
```

The driver installation must provide at least these packages:

```bash
ros2 pkg prefix ur_robot_driver
ros2 pkg prefix ur_description
ros2 pkg prefix ur_client_library
```

Install the External Control URCap on both physical robots and configure it as
described in [Hardware and network setup](hardware_and_network_setup.md).

## CoppeliaSim requirements

Install a CoppeliaSim version compatible with the available `simROS2` plugin.
The tested scenes require:

- `simROS2`
- `simIK`
- `sensor_msgs/msg/JointState` support in `simROS2`

The development installation was started from:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"

cd "$HOME/CoppeliaSim"
./coppeliaSim.sh
```

## RoboDK requirements

Install RoboDK, Python 3 and the RoboDK Python API. The configuration-analysis
stage also uses NumPy and pandas:

```bash
python3 -m pip install --user robodk numpy pandas
```

## Install project dependencies

From the repository root:

```bash
cd ros2_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

## Build the project workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

For a clean rebuild, start a new terminal and source only ROS 2 and the UR
driver workspace before deleting generated directories:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"

cd "$HOME/zavrsni_nonsequential_task_execution/ros2_ws"
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## Verify the custom packages

```bash
ros2 pkg list | grep nonsequential_task
```

Expected packages:

```text
nonsequential_task_bringup
nonsequential_task_control
nonsequential_task_description
nonsequential_task_meta
```

## Verify the control scripts

```bash
ros2 pkg executables nonsequential_task_control
```

Expected executables:

```text
nonsequential_task_control coppelia_to_forward_position_robot1_ns.py
nonsequential_task_control coppelia_to_forward_position_robot2_ns.py
nonsequential_task_control dual_coppelia_record_then_joint_trajectory_wait_key.py
```

## Verify the launch package

```bash
export ROS_DOMAIN_ID=7

ros2 launch \
  nonsequential_task_bringup \
  dual_ns.launch.py \
  --show-args
```

The command should display the available launch arguments without reporting
missing launch files, configuration files or ROS 2 packages.

## Next steps

1. Complete [Hardware and network setup](hardware_and_network_setup.md).
2. Choose exactly one physical execution method:
   - [Forward-position start-up](startup_forward_position.md)
   - [Recorded JointTrajectory start-up](startup_joint_trajectory.md)
3. Use [Troubleshooting](troubleshooting.md) if a preflight check fails.
