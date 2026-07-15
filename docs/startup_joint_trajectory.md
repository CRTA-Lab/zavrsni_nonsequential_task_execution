# Recorded JointTrajectory start-up

## Purpose

This procedure records the complete coordinated CoppeliaSim motion before any
physical movement. The recorded samples are converted into time-defined
trajectories containing joint positions, velocities, accelerations and
`time_from_start`. Both goals are sent through the
`FollowJointTrajectory` actions only after the operator presses `s`.

Unlike live one-point action streaming, the controller receives the complete
future trajectory and can interpolate it continuously.

## Required scene and controller

- Scene: `coppeliasim/dual_ur_joint_trajectory/scene/dual_ur_joint_trajectory.ttt`
- Required controller: `joint_trajectory_controller` on both robots
- Recorded topics: `/robot1/coppelia_joint_target` and
  `/robot2/coppelia_joint_target`
- Action servers: `/robot1/joint_trajectory_controller/follow_joint_trajectory`
  and `/robot2/joint_trajectory_controller/follow_joint_trajectory`

The Lua scripts transport acceleration in `JointState.effort` because
`sensor_msgs/msg/JointState` has no acceleration field. In this experiment the
field therefore means joint acceleration in `rad/s²`; it does not mean torque.

## Self-contained package implementation

The operator entry point and the validated recorder/trajectory builder are both
included in `nonsequential_task_control`. The base implementation is installed
next to the wait-key script as an internal support file. No Python file in the
user's home directory is required.

After building, verify both installed files:

```bash
test -s "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/nonsequential_task_control/lib/nonsequential_task_control/dual_coppelia_record_then_joint_trajectory_wait_key.py"
test -s "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/nonsequential_task_control/lib/nonsequential_task_control/dual_coppelia_record_then_joint_trajectory.py"
```

## 1. Start the two robot drivers

Terminal 1:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 launch nonsequential_task_bringup dual_ns.launch.py
```

Start the External Control programs from the beginning:

- UR3e: host `192.168.40.51`, port `50102`
- UR5e: host `192.168.40.51`, port `50002`

Verify both feedback topics before continuing:

```bash
ros2 topic echo --once /robot1/joint_states
ros2 topic echo --once /robot2/joint_states
```

## 2. Open the JointTrajectory scene

Terminal 2:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"

cd "$HOME/CoppeliaSim"
./coppeliaSim.sh
```

Open `dual_ur_joint_trajectory.ttt`, but do not press **Play** yet. This must be
the copied JointTrajectory experiment scene, not the validated forward-position
fallback scene.

## 3. Activate `joint_trajectory_controller`

From the default driver state:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 control switch_controllers \
  --controller-manager /robot1/controller_manager \
  --deactivate scaled_joint_trajectory_controller \
  --activate joint_trajectory_controller

ros2 control switch_controllers \
  --controller-manager /robot2/controller_manager \
  --deactivate scaled_joint_trajectory_controller \
  --activate joint_trajectory_controller

ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager
```

If `forward_position_controller` is active, deactivate it instead of
`scaled_joint_trajectory_controller`. Never leave two joint-command controllers
active for the same robot.

Expected state:

```text
joint_trajectory_controller                 active
scaled_joint_trajectory_controller          inactive
forward_position_controller                 inactive
```

Verify both action servers:

```bash
ros2 action list | grep joint_trajectory_controller/follow_joint_trajectory
```

## 4. Start recording

Terminal 3:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 run nonsequential_task_control \
  dual_coppelia_record_then_joint_trajectory_wait_key.py \
  --record-seconds 120.0 \
  --time-scale 1.0 \
  --downsample 1 \
  --filter-alpha 0.30 \
  --max-start-error 0.25 \
  --max-velocity 5.0 \
  --max-acceleration 5.0
```

These are the successfully validated final parameters. `--downsample 1` keeps
all samples and `--filter-alpha 0.30` smooths the calculated derivatives. A
`time-scale` of `1.0` preserves the processed recording timing.

Immediately after the recorder is ready, press **Play** in CoppeliaSim and let
the full simulation execute. Do not move either physical robot while recording;
the physical robots have not yet received a trajectory.

## 5. Inspect and authorize execution

After recording, the script prints the point count and duration for both
robots. It also checks the difference between the first trajectory point and
the current physical joint state. If the start error exceeds `0.25 rad`, do not
bypass the check; synchronize again and repeat the recording.

The prompt accepts one key:

```text
s = send both trajectories and start
q = cancel without sending
```

Press `s` only after confirming:

- both controllers are active;
- both robots are at the recorded start positions;
- the workspace is clear;
- both emergency stops are accessible.

The script waits two seconds and submits both goals. Successful execution ends
with accepted goals and action results with `error_code: 0`.

## Stop, cancel and restore the fallback

- Before sending: press `q` to exit without physical motion.
- During execution: use the robot stop control if motion becomes unsafe.
- After the test, stop CoppeliaSim and restore the normal default controller:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 control switch_controllers \
  --controller-manager /robot1/controller_manager \
  --deactivate joint_trajectory_controller \
  --activate scaled_joint_trajectory_controller

ros2 control switch_controllers \
  --controller-manager /robot2/controller_manager \
  --deactivate joint_trajectory_controller \
  --activate scaled_joint_trajectory_controller
```

The original forward-position method remains available by following
[Forward-position start-up](startup_forward_position.md).
