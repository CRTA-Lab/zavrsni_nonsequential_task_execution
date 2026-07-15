# Non-sequential task bringup

This ROS 2 package contains the launch and configuration files used to start
the two namespaced Universal Robots drivers for the thesis:

**Non-sequential Task Execution with Two Industrial Robots**

## Included launch files

- `launch/dual_ns.launch.py` – starts both namespaced robot drivers
- `launch/robot1_ns.launch.py` – configuration of Robot 1, the UR3e
- `launch/robot2_ns.launch.py` – configuration of Robot 2, the UR5e
- `launch/ur_control_ns.launch.py` – shared namespaced UR control launch logic

## Configuration files

The `launch/config/` directory contains the controller and robot configuration
files used by the launch system.

## Robot configuration

### Robot 1 – UR3e

- robot IP: `192.168.40.50`
- namespace: `/robot1`
- reverse port: `50101`
- script sender port: `50102`
- trajectory port: `50103`
- script command port: `50104`

### Robot 2 – UR5e

- robot IP: `192.168.40.14`
- namespace: `/robot2`
- reverse port: `50001`
- script sender port: `50002`
- trajectory port: `50003`
- script command port: `50004`

## Launch

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash

ros2 launch nonsequential_task_bringup dual_ns.launch.py
```

The launch files use local relative paths for the shared launch file and
controller configurations; they do not depend on the former `~/ur_ns`
directory. `scaled_joint_trajectory_controller` is the default active joint
controller. Use the project start-up guides when switching controllers:

- [Forward-position start-up](../../../docs/startup_forward_position.md)
- [Recorded JointTrajectory start-up](../../../docs/startup_joint_trajectory.md)

## Verification

```bash
ros2 topic echo --once /robot1/joint_states
ros2 topic echo --once /robot2/joint_states
ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager
```

## Author

Damjan Drobac
