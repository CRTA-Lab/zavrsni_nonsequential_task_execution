# Hardware and network setup

## Purpose

This document defines the validated physical and network configuration for the
dual-robot experiments. Complete this setup before starting either control
method.

## Safety requirements

- Only trained personnel may operate the physical robots.
- Clear the shared workspace and keep both emergency stops accessible.
- Use reduced speed during initial validation.
- Verify that the simulated and physical joint positions agree before enabling
  command transfer.
- Never start the forward-position bridges or send a trajectory while a person
  is inside the robot workspace.
- Stop the CoppeliaSim simulation before changing controllers.

## Hardware mapping

| Setting | Robot 1 | Robot 2 |
| --- | --- | --- |
| Physical robot | UR3e | UR5e |
| Robot IP | `192.168.40.50` | `192.168.40.14` |
| ROS namespace | `/robot1` | `/robot2` |
| TF prefix | `robot1_` | `robot2_` |
| Reverse port | `50101` | `50001` |
| Script sender / External Control port | `50102` | `50002` |
| Trajectory port | `50103` | `50003` |
| Script command port | `50104` | `50004` |

The Ubuntu control computer uses `192.168.40.51` and all ROS 2 processes use
`ROS_DOMAIN_ID=7`.

## Configure the control-computer network

Configure the wired interface with a static IPv4 address:

```text
Address: 192.168.40.51
Netmask/prefix: 255.255.255.0 (/24)
```

A gateway and DNS server are not required on a dedicated robot network. Do not
assign the same address to another device.

Identify the wired interface and verify the assigned address:

```bash
ip -br link
ip -br address
```

Verify basic reachability:

```bash
ping -c 4 192.168.40.50
ping -c 4 192.168.40.14
```

Both robots must reply before the driver is launched.

## Configure External Control on the teach pendants

The Universal Robots External Control URCap must be installed on both robots.
Create one External Control installation/program for each robot.

### Robot 1 – UR3e

```text
Host IP: 192.168.40.51
Custom port: 50102
```

### Robot 2 – UR5e

```text
Host IP: 192.168.40.51
Custom port: 50002
```

The ports must match `script_sender_port` in the ROS 2 launch files. Load the
program containing the External Control node on each teach pendant. Start the
ROS 2 driver first, then press **Play from beginning** on both robots.

## ROS 2 environment

Use the same environment in every terminal:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"
```

The project launch files already contain the validated IP addresses,
namespaces, TF prefixes and non-conflicting port sets.

## Start and verify both drivers

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 launch nonsequential_task_bringup dual_ns.launch.py
```

After starting the External Control programs, verify the feedback topics:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 topic list | grep joint_states
ros2 topic echo --once /robot1/joint_states
ros2 topic echo --once /robot2/joint_states
```

Expected topics:

```text
/robot1/joint_states
/robot2/joint_states
```

The joint names are prefixed with `robot1_` and `robot2_` in driver feedback.
The CoppeliaSim Lua scripts map these names during synchronization.

## Mandatory synchronization rule

Before physical execution, CoppeliaSim must receive `/robot1/joint_states` and
`/robot2/joint_states` and align both simulated robots with their physical
positions. Starting from unrelated simulated positions previously caused a
teach-pendant tracking error. The included dual-UR Lua scripts use automatic
synchronization for three seconds before entering their motion modes.

Continue with exactly one procedure:

- [Forward-position start-up](startup_forward_position.md)
- [Recorded JointTrajectory start-up](startup_joint_trajectory.md)
