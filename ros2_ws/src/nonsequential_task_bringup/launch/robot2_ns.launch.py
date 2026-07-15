import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            PushRosNamespace("robot2"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(os.path.dirname(os.path.abspath(__file__)), "ur_control_ns.launch.py")
                ),
                launch_arguments={
                    "ur_type": "ur5e",
                    "robot_ip": "192.168.40.14",
                    "launch_rviz": "false",
                    "tf_prefix": "robot2_",
                    "controllers_file": os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "robot2_ur_controllers.yaml"),
                    "reverse_port": "50001",
                    "script_sender_port": "50002",
                    "trajectory_port": "50003",
                    "script_command_port": "50004",
                    "controller_spawner_timeout": "30",
                }.items(),
            ),
        ])
    ])
