# Forward-position start-up

## Purpose

This procedure runs the final dual-UR task live from CoppeliaSim. Each Lua
script publishes only the current six joint positions in a `JointState`.
Separate Python bridges reorder the joints and publish six-element
`Float64MultiArray` commands to the two namespaced
`forward_position_controller` instances.

This is the validated fallback workflow and must remain available even when
testing JointTrajectory execution.

## Required scene and controller

- Scene: `coppeliasim/dual_ur_forward_position/scene/dual_ur_forward_position.ttt`
- Required controller on both robots: `forward_position_controller`
- Coppelia topics: `/robot1/coppelia_joint_target` and
  `/robot2/coppelia_joint_target`
- Controller topics: `/robot1/forward_position_controller/commands` and
  `/robot2/forward_position_controller/commands`

Do not use the JointTrajectory scene for this procedure.

## 1. Start the two robot drivers

Terminal 1:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 launch nonsequential_task_bringup dual_ns.launch.py
```

On the teach pendants, run the External Control programs from the beginning:

- UR3e: host `192.168.40.51`, port `50102`
- UR5e: host `192.168.40.51`, port `50002`

## 2. Verify feedback

Terminal 2:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 topic echo --once /robot1/joint_states
ros2 topic echo --once /robot2/joint_states
```

Do not continue unless both commands return current joint data.

## 3. Open CoppeliaSim without starting the simulation

Terminal 3:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"

cd "$HOME/CoppeliaSim"
./coppeliaSim.sh
```

Open `dual_ur_forward_position.ttt`, but do not press **Play** yet. Confirm that
the two robot scripts use `MODE = 'auto'`.

## 4. Start both position bridges

Terminal 4:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 run nonsequential_task_control coppelia_to_forward_position_robot1_ns.py
```

Terminal 5:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 run nonsequential_task_control coppelia_to_forward_position_robot2_ns.py
```

## 5. Activate the required controllers

The driver starts with `scaled_joint_trajectory_controller` active. Switch each
robot separately:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 control switch_controllers \
  --controller-manager /robot1/controller_manager \
  --deactivate scaled_joint_trajectory_controller \
  --activate forward_position_controller

ros2 control switch_controllers \
  --controller-manager /robot2/controller_manager \
  --deactivate scaled_joint_trajectory_controller \
  --activate forward_position_controller
```

If the active controller is not `scaled_joint_trajectory_controller`, first
inspect the controller list and deactivate the controller that is actually
active.

Verify:

```bash
ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager
```

Expected state on both robots:

```text
forward_position_controller                 active
scaled_joint_trajectory_controller          inactive
```

## 6. Start the coordinated task

With the workspace clear and a hand near the stop control, press **Play** in
CoppeliaSim.

The automatic sequence is:

1. Both simulated robots synchronize from physical `/joint_states` for three
   seconds.
2. The simulated UR5e approaches the initial circular-path point.
3. UR5e sets `robot2_ready_for_robot1_run`.
4. UR3e moves through `target3` to `target10` and repeats the defined sequence.
5. UR5e follows the moving circular target while carrying the pipe.
6. Both bridges continuously forward the simulated joint positions.

Useful monitoring commands:

```bash
ros2 topic hz /robot1/coppelia_joint_target
ros2 topic hz /robot2/coppelia_joint_target
ros2 topic hz /robot1/forward_position_controller/commands
ros2 topic hz /robot2/forward_position_controller/commands
```

## Stop and return to the default controller

1. Stop the CoppeliaSim simulation.
2. Stop both bridge processes with `Ctrl+C`.
3. Switch each robot back to the default trajectory controller:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 control switch_controllers \
  --controller-manager /robot1/controller_manager \
  --deactivate forward_position_controller \
  --activate scaled_joint_trajectory_controller

ros2 control switch_controllers \
  --controller-manager /robot2/controller_manager \
  --deactivate forward_position_controller \
  --activate scaled_joint_trajectory_controller
```

Verify both controller states before closing the driver terminals.
