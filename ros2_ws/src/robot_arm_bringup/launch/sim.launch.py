import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable



def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory("robot_arm_gazebo"),
        "launch",
        "simulation.launch.py",
    )
    return LaunchDescription([
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        )
    ])