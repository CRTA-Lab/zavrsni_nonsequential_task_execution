cmake_minimum_required(VERSION 3.8)
project(nonsequential_task_control)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/coppelia_to_forward_position_robot1_ns.py
    scripts/coppelia_to_forward_position_robot2_ns.py
    scripts/dual_coppelia_record_then_joint_trajectory_wait_key.py
  DESTINATION lib/${PROJECT_NAME}
)

# Internal implementation imported by the operator-confirmation entry point.
# It is installed as a regular file so it is not exposed as a separate
# auto-sending ros2 run executable.
install(
  FILES
    scripts/dual_coppelia_record_then_joint_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
