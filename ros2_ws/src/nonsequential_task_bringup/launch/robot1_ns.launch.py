import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            PushRosNamespace("robot1"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(os.path.dirname(os.path.abspath(__file__)), "ur_control_ns.launch.py")
                ),
                launch_arguments={
                    "ur_type": "ur3e",
                    "robot_ip": "192.168.40.50",
                    "launch_rviz": "false",
                    "tf_prefix": "robot1_",
                    "controllers_file": os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "robot1_ur_controllers.yaml"),
                    "reverse_port": "50101",
                    "script_sender_port": "50102",
                    "trajectory_port": "50103",
                    "script_command_port": "50104",
                    "controller_spawner_timeout": "30",
                }.items(),
            ),
        ])
    ])
