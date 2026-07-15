# Non-sequential task control

This ROS 2 package contains the custom simulation-to-real control scripts
developed for the thesis:

**Non-sequential Task Execution with Two Industrial Robots**

## Included scripts

### Forward-position execution

- `scripts/coppelia_to_forward_position_robot1_ns.py`
- `scripts/coppelia_to_forward_position_robot2_ns.py`

These scripts subscribe to the simulated joint targets published from
CoppeliaSim and forward the ordered joint-position commands to the namespaced
`forward_position_controller` instances.

### Recorded JointTrajectory execution

- `scripts/dual_coppelia_record_then_joint_trajectory_wait_key.py`

This script records the simulated motion of both robots, generates complete
time-defined trajectories and sends them to the corresponding
`joint_trajectory_controller` instances.

Each generated trajectory contains:

- joint positions
- joint velocities
- joint accelerations
- `time_from_start`

## Robot namespaces

- Robot 1 – UR3e: `/robot1`
- Robot 2 – UR5e: `/robot2`

## Final methods

The repository contains only the final validated methods:

- live execution through `forward_position_controller`
- recorded execution through `joint_trajectory_controller`

Experimental live JointTrajectory methods are not included.

## Run commands

After building and sourcing the project workspace:

```bash
ros2 run nonsequential_task_control coppelia_to_forward_position_robot1_ns.py
ros2 run nonsequential_task_control coppelia_to_forward_position_robot2_ns.py
```

The validated recorded-trajectory command and controller preparation are in
[Recorded JointTrajectory start-up](../../../docs/startup_joint_trajectory.md).

## JointTrajectory base implementation

`dual_coppelia_record_then_joint_trajectory_wait_key.py` is the supported
operator-confirmation entry point. It imports the recorder and trajectory
builder from `dual_coppelia_record_then_joint_trajectory.py`, installed next to
it inside this package. No external `$HOME` script is required.

The base implementation is installed as a non-entry-point support file. Run
the wait-key executable so the two trajectories are sent only after an explicit
`s` confirmation.

## Author

Damjan Drobac
