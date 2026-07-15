# Dual-UR forward-position execution

## Purpose

This scene represents the first functional dual-robot simulation-to-real
execution.

The simulated UR3e generates the moving reference and executes target-to-target
motion. The simulated UR5e carries the pipe and follows the moving circular path.

The joint targets are transferred through ROS 2 to the physical robots using
`forward_position_controller`.

## Included files

- `scene/dual_ur_forward_position.ttt`
- `lua/robot1_ur3e_run.lua`
- `lua/robot2_ur5e_track.lua`
- `lua/path_target.lua`

The Lua scripts are embedded in the `.ttt` scene. Separate copies are included
for direct source-code inspection.

## Lua script roles

- `robot1_ur3e_run.lua` – synchronizes the UR3e model and executes target-to-target motion
- `robot2_ur5e_track.lua` – synchronizes the UR5e model, approaches the initial point and follows the moving path
- `path_target.lua` – moves the target along the circular path and updates its pose

## Robot mapping

- Robot 1: UR3e
- Robot 2: UR5e

## Main ROS 2 topics

- `/robot1/joint_states`
- `/robot2/joint_states`
- `/robot1/coppelia_joint_target`
- `/robot2/coppelia_joint_target`
- `/robot1/forward_position_controller/commands`
- `/robot2/forward_position_controller/commands`

## Operating sequence

1. Start both UR drivers.
2. Start External Control on both robots.
3. Verify both joint-state topics.
4. Start CoppeliaSim with `ROS_DOMAIN_ID=7`.
5. Open `scene/dual_ur_forward_position.ttt`.
6. Start both forward-position bridge scripts.
7. Activate `forward_position_controller` for both robots.
8. Start the simulation.
9. Automatic mode synchronizes both simulated robots.
10. The UR5e approaches the initial path point.
11. The UR5e sends the ready signal.
12. The UR3e begins the target-to-target motion.
13. The UR5e follows the circular moving target.

## Complete instructions

See:

[Forward-position start-up](../../docs/startup_forward_position.md)

## Safety

The physical work area must be clear. Both simulated robots must be
synchronized with their corresponding physical robots before command transfer.

## Expected behaviour

The physical UR3e follows the simulated target-to-target motion, while the
physical UR5e follows the moving circular path and carries the pipe around the
UR3e arm.

## Detailed operating notes

The historical CoppeliaSim object names `/UR3e1` and `/UR3e2` do not by
themselves identify the physical robot type. The ROS mapping in this document
and the Lua joint lists are authoritative. Do not rename scene objects without
updating the embedded scripts and their readable copies.

Open the scene without pressing Play, start both bridges, activate both
`forward_position_controller` instances, verify the controller states, and
only then start the simulation. Automatic mode performs three seconds of
synchronization before motion.

The Lua messages contain positions only; `velocity` and `effort` are empty.
The bridges do not send velocities, accelerations, future points or an
interpolation horizon. Network and teach-pendant setup is documented in
[Hardware and network setup](../../docs/hardware_and_network_setup.md).
