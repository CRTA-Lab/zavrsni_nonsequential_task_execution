from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot1_ns.launch.py"))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot2_ns.launch.py"))
        ),
    ])
