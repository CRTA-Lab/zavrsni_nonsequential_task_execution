# Non-sequential Task Execution with Two Industrial Robots

This repository contains the simulation, control and documentation material
developed for the thesis **Non-sequential Task Execution with Two Industrial
Robots**. The project coordinates a UR3e and a UR5e through RoboDK,
CoppeliaSim, ROS 2 Humble and the Universal Robots ROS 2 Driver.

Author and primary developer: **Damjan Drobac**<br>
GitHub: [DamjanDrobac999](https://github.com/DamjanDrobac999)<br>
Institution: University of Zagreb, Faculty of Mechanical Engineering and Naval Architecture
Laboratory: CRTA – Regional Center of Excellence for Robotic Technology

## Reproducible project stages

1. [RoboDK dot following](robodk/dot_following/README.md)
2. [RoboDK path following and configuration analysis](robodk/path_following_configuration_analysis/README.md)
3. [ABB CoppeliaSim path following](coppeliasim/abb_path_following/README.md)
4. [Dual-UR forward-position execution](coppeliasim/dual_ur_forward_position/README.md)
5. [Dual-UR recorded JointTrajectory execution](coppeliasim/dual_ur_joint_trajectory/README.md)

## General documentation

- [Installation](docs/installation.md)
- [Hardware and network setup](docs/hardware_and_network_setup.md)
- [Forward-position start-up](docs/startup_forward_position.md)
- [JointTrajectory start-up](docs/startup_joint_trajectory.md)
- [URDF preparation](docs/urdf_models.md)
- [Troubleshooting](docs/troubleshooting.md)
- [ROS 2 workspace](ros2_ws/README.md)

## Validated hardware configuration

| Role | Robot | IP address | ROS namespace | External Control port |
| --- | --- | --- | --- | --- |
| Robot 1 | UR3e | `192.168.40.50` | `/robot1` | `50102` |
| Robot 2 | UR5e | `192.168.40.14` | `/robot2` | `50002` |
| Control computer | Ubuntu 22.04 | `192.168.40.51` | — | — |

All ROS 2 processes use `ROS_DOMAIN_ID=7`. See the
[hardware and network guide](docs/hardware_and_network_setup.md) before
connecting to physical robots.

## Quick verification

After installing the prerequisites, build and inspect the workspace:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"

cd "$HOME/zavrsni_nonsequential_task_execution/ros2_ws"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

ros2 pkg list | grep nonsequential_task
ros2 launch nonsequential_task_bringup dual_ns.launch.py --show-args
```

The physical-robot procedures are intentionally separated by controller:

- [live forward-position execution](docs/startup_forward_position.md)
- [recorded JointTrajectory execution](docs/startup_joint_trajectory.md)

Do not run both procedures at the same time.

## Repository contents

The repository contains:

- RoboDK stations and Python scripts
- CoppeliaSim scenes
- separate readable copies of embedded Lua scripts
- ROS 2 metapackage and project subpackages
- URDF files used during robot-model preparation
- bridge and trajectory-execution scripts
- Markdown installation and execution instructions
- [Final thesis PDF](thesis/Damjan_Drobac_Zavrsni_rad.pdf)

Generated recordings, ROS bags, plots, CSV files and videos are excluded from
Git by `.gitignore`. They are experimental outputs rather than required source
files.
