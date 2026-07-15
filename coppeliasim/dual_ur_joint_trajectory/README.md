# Dual-UR recorded JointTrajectory execution

## Purpose

This experiment uses the final coordinated dual-robot task. The simulated
motion is first recorded and then converted into complete time-defined
JointTrajectory commands.

Each trajectory point contains:

- joint positions
- joint velocities
- joint accelerations
- `time_from_start`

The complete trajectory is sent to `joint_trajectory_controller` before
physical execution.

## Included files

- `scene/dual_ur_joint_trajectory.ttt`
- `lua/robot1_ur3e_vel_acc.lua`
- `lua/robot2_ur5e_vel_acc.lua`
- `lua/path_target.lua`

The Lua scripts are embedded in the `.ttt` scene. Separate copies are included
for direct source-code inspection.

## Lua script roles

- `robot1_ur3e_vel_acc.lua` – publishes UR3e joint positions, velocities and accelerations
- `robot2_ur5e_vel_acc.lua` – publishes UR5e joint positions, velocities and accelerations
- `path_target.lua` – moves the target along the circular path while the simulated trajectory is recorded

## Execution sequence

1. Start both UR drivers.
2. Start External Control on both robots.
3. Synchronize simulated and physical joint positions.
4. Activate `joint_trajectory_controller`.
5. Start the record-then-execute Python script.
6. Start the complete CoppeliaSim motion.
7. Allow the script to record the complete motion.
8. Confirm physical execution when prompted.
9. The complete trajectories are sent to both physical robots.

## Validated parameters

- recording duration: `120.0 s`
- time scale: `1.0`
- downsample: `1`
- filter alpha: `0.30`
- maximum start error: `0.25 rad`
- maximum velocity: `5.0 rad/s`
- maximum acceleration: `5.0 rad/s²`

## Complete instructions

See:

[JointTrajectory start-up](../../docs/startup_joint_trajectory.md)

## Expected behaviour

Both physical robots execute the complete coordinated trajectory without
visible pipe vibration.

## Detailed operating notes

CoppeliaSim transports joint acceleration in `JointState.effort` because
`JointState` has no acceleration field. In this experiment `effort` therefore
means `rad/s²`, not torque. No physical command is sent during recording; the
operator must press `s` to send both complete goals or `q` to cancel.

The wait-key entry point and validated base implementation are both installed
by `nonsequential_task_control`; no external script in `$HOME` is required.

Live high-frequency `FollowJointTrajectory` goals were not retained because
new goals repeatedly preempted unfinished goals. Sending the complete recorded
trajectory gives the controller positions, derivatives and timing in advance
and produced the smoothest validated execution.
