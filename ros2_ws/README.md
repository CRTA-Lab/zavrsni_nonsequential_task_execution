# ROS 2 workspace

## Purpose

This colcon workspace contains the project-specific ROS 2 packages for the
namespaced UR3e/UR5e system. It depends on a separate Universal Robots driver
workspace, expected at `~/ur_ws` in the validated setup.

## Packages

| Package | Purpose |
| --- | --- |
| `nonsequential_task_bringup` | Dual namespaced UR driver launch files and controller YAML files |
| `nonsequential_task_control` | Forward-position bridges and recorded JointTrajectory entry point |
| `nonsequential_task_description` | Prepared UR3e and UR5e URDF files with visible-mesh paths |
| `nonsequential_task_meta` | Runtime grouping of the three project packages |

## Build

From a fresh terminal:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"

cd "$HOME/zavrsni_nonsequential_task_execution/ros2_ws"

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Expected result:

```text
Summary: 4 packages finished
```

Verify discovery:

```bash
ros2 pkg list | grep nonsequential_task
ros2 pkg executables nonsequential_task_control
```

## Dual-driver launch

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source "$HOME/ur_ws/install/setup.bash"
source "$HOME/zavrsni_nonsequential_task_execution/ros2_ws/install/setup.bash"

ros2 launch nonsequential_task_bringup dual_ns.launch.py
```

The launch package starts:

| Namespace | Robot | IP | TF prefix | Port range |
| --- | --- | --- | --- | --- |
| `/robot1` | UR3e | `192.168.40.50` | `robot1_` | `50101`–`50104` |
| `/robot2` | UR5e | `192.168.40.14` | `robot2_` | `50001`–`50004` |

RViz is disabled by the two robot wrapper launch files. The default initial
joint controller is `scaled_joint_trajectory_controller`.

## Controller selection

List controllers:

```bash
ros2 control list_controllers -c /robot1/controller_manager
ros2 control list_controllers -c /robot2/controller_manager
```

Only one joint-command controller may be active for each robot:

| Workflow | Required active controller |
| --- | --- |
| Live CoppeliaSim position stream | `forward_position_controller` |
| Recorded complete trajectory | `joint_trajectory_controller` |
| Default/fallback | `scaled_joint_trajectory_controller` |

Use the complete switch commands in:

- [Forward-position start-up](../docs/startup_forward_position.md)
- [Recorded JointTrajectory start-up](../docs/startup_joint_trajectory.md)

## Installed control executables

### Forward-position bridges

```bash
ros2 run nonsequential_task_control coppelia_to_forward_position_robot1_ns.py
ros2 run nonsequential_task_control coppelia_to_forward_position_robot2_ns.py
```

Each bridge subscribes to its CoppeliaSim `JointState`, orders the six
unprefixed joint values and publishes a `Float64MultiArray` to the corresponding
position controller.

### Recorded JointTrajectory entry point

```bash
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

The operator entry point imports the validated base implementation installed
next to it inside `nonsequential_task_control`. The base file is intentionally
not exposed as a separate `ros2 run` executable because its original main
function sends goals automatically after a countdown. Use the wait-key entry
point, which requires an explicit `s` confirmation. See the complete
JointTrajectory guide for the safety and start-up sequence.

## Topic and action map

| Direction | Robot 1 | Robot 2 |
| --- | --- | --- |
| Driver feedback | `/robot1/joint_states` | `/robot2/joint_states` |
| Coppelia target | `/robot1/coppelia_joint_target` | `/robot2/coppelia_joint_target` |
| Position command | `/robot1/forward_position_controller/commands` | `/robot2/forward_position_controller/commands` |
| Trajectory action | `/robot1/joint_trajectory_controller/follow_joint_trajectory` | `/robot2/joint_trajectory_controller/follow_joint_trajectory` |

## Source layout

```text
src/
├── nonsequential_task_bringup/
│   └── launch/
│       ├── config/
│       ├── dual_ns.launch.py
│       ├── robot1_ns.launch.py
│       ├── robot2_ns.launch.py
│       └── ur_control_ns.launch.py
├── nonsequential_task_control/
│   └── scripts/
├── nonsequential_task_description/
│   └── urdf/
└── nonsequential_task_meta/
```

`build/`, `install/` and `log/` are generated locally and must not be committed.
