# Troubleshooting

## No `/robot1/joint_states` or `/robot2/joint_states`

Check the shared ROS domain and driver environment in every terminal:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 topic list | grep joint_states
```

Also verify that the robot IPs answer `ping`, the drivers are still running,
and both teach-pendant programs are playing their External Control nodes.

## External Control does not connect

Verify the host and port on each teach pendant:

| Robot | Host | Port |
| --- | --- | --- |
| UR3e | `192.168.40.51` | `50102` |
| UR5e | `192.168.40.51` | `50002` |

These values must match `script_sender_port` in `robot1_ns.launch.py` and
`robot2_ns.launch.py`. The other ports must also remain unique between robots.

## CoppeliaSim has no ROS 2 topics

Start CoppeliaSim from a terminal that contains `ROS_DOMAIN_ID=7` and the ROS 2
Humble environment. Confirm that the `simROS2` and `simIK` plugins load without
errors. The embedded scripts require `sensor_msgs/msg/JointState` support.

## CoppeliaSim reports an unsupported message type

The validated scripts publish `sensor_msgs/msg/JointState`. They do not publish
`std_msgs/msg/Float64MultiArray` directly from CoppeliaSim because the tested
plugin reported that type as unsupported. The Python forward-position bridges
perform the conversion to `Float64MultiArray`.

If `JointState` is also unsupported, rebuild the simROS2 plugin version that
matches the installed CoppeliaSim release and include:

```text
sensor_msgs/msg/JointState
std_msgs/msg/Header
builtin_interfaces/msg/Time
```

in the plugin interface list.

## Simulated robot jumps or the teach pendant reports a tracking error

The simulated and physical robots were not synchronized before commands were
enabled. Stop the simulation and command bridges, restore a trajectory
controller, then restart the complete procedure. Do not manually skip the
three-second automatic synchronization phase.

For recorded JointTrajectory execution, also respect `--max-start-error 0.25`.
Never increase this threshold merely to force an unsafe start.

## Controller switch fails

Inspect the actual state before switching:

```bash
ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager
```

Deactivate only the controller that is currently active and activate exactly
one new joint-command controller. The two methods require:

| Method | Active controller |
| --- | --- |
| Live position | `forward_position_controller` |
| Recorded trajectory | `joint_trajectory_controller` |
| Normal fallback | `scaled_joint_trajectory_controller` |

## Bridge reports missing expected joint names

CoppeliaSim publishes the six unprefixed names
`shoulder_pan_joint` through `wrist_3_joint`. The forward-position bridge orders
those values for the controller. Physical feedback uses `robot1_` or `robot2_`
prefixes. Do not rename only one side of this mapping.

## Forward-position motion visibly vibrates in live real-time mode

`forward_position_controller` receives only current joint positions: no
velocities, accelerations, future points or interpolation horizon. It therefore
behaves like a sample-and-hold reference.

In the experiment, CoppeliaSim real-time mode produced a less dense or less
regular live command stream in wall time and the pipe vibrated visibly. With
real-time mode disabled, the same simulated motion ran approximately three
times faster relative to wall time and generated denser setpoints, which looked
smoother despite the faster physical execution. The robots were not faster
than the simulation; they followed the simulation's different wall-time rate.

Recorded forward-position replay reduced this timing problem by publishing
interpolated positions at a regular rate. Small tactile vibration could remain
because the command still contains positions only.

## Recorded JointTrajectory is smooth but has a different duration

The complete trajectory contains positions, velocities, accelerations and
`time_from_start`, so execution is no longer tied to live CoppeliaSim callback
timing. The CoppeliaSim real-time toggle still changes how quickly the original
samples are generated relative to wall time. `--time-scale` then scales the
processed trajectory timing. The validated final run used `--time-scale 1.0`.

## Live `FollowJointTrajectory` goals do not track correctly

Do not stream a new short action goal for every incoming sample. Each new goal
preempts or replaces the preceding goal before it finishes. In testing, higher
goal rates made tracking worse: the robots jerked, followed intermittently or
moved only to the final point after streaming stopped. Use the validated
record-then-send-complete-trajectory method.

## Velocity-only control enters an unsafe configuration

`forward_velocity_controller` proved that joint velocities can be sent, but
velocity-only tracking did not preserve the desired IK configuration and could
drive the robot toward self-collision. It is an experimental result, not a
supported final execution method in this repository.

## Wait-key JointTrajectory script cannot find its internal implementation

The operator entry point loads
`dual_coppelia_record_then_joint_trajectory.py` from the same installed package
directory. If the file is missing, rebuild the current source tree and source
the new installation:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"

cd "$HOME/zavrsni_nonsequential_task_execution/ros2_ws"
colcon build --symlink-install --packages-select nonsequential_task_control
source install/setup.bash
```

Do not copy a second implementation into `$HOME`; the package is self-contained.

## URDF imports with joints but without geometry

CoppeliaSim did not resolve `package://ur_description/...` mesh URIs. Use the
prepared URDF files or replace the package prefix with:

```text
file:///opt/ros/humble/share/ur_description
```

See [URDF model preparation](urdf_models.md).

## Clean build prints stale `AMENT_PREFIX_PATH` warnings

Warnings about deleted paths can appear if the old project `install/setup.bash`
was sourced before `build`, `install` and `log` were removed. Open a clean
terminal, source only ROS 2 and `~/ur_ws`, then build again:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"

cd "$HOME/zavrsni_nonsequential_task_execution/ros2_ws"
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

If all four packages finish, the earlier prefix warnings were environmental,
not build failures.

## RoboDK scripts cannot find CSV files

Some original scripts contain development-machine Windows paths for recordings
and analysis input. Before running them, update the documented `CSV_FILE`,
`pattern`, `INPUT_CSV` or `INPUT_BEST_CSV` constant to a valid local path. The
generated CSV files are intentionally excluded from Git.
