import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_arm_bringup")
    driver_share = get_package_share_directory("robot_arm_driver")
    moveit_share = get_package_share_directory("robot_arm_moveit_config")

    backend = LaunchConfiguration("backend")
    params_file = LaunchConfiguration("params_file")
    allow_motion = LaunchConfiguration("allow_motion")

    legacy_launch = os.path.join(bringup_share, "launch", "real.launch.py")
    moveit_launch = os.path.join(moveit_share, "launch", "real_moveit.launch.py")
    default_driver_params = os.path.join(
        driver_share, "config", "arm_driver.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="legacy",
                choices=["legacy", "moveit"],
                description="IK/planning backend to launch",
            ),
            DeclareLaunchArgument(
                "params_file", default_value=default_driver_params
            ),
            DeclareLaunchArgument("allow_motion", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(legacy_launch),
                launch_arguments={
                    "params_file": params_file,
                    "allow_motion": allow_motion,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", backend, "' == 'legacy'"])
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={
                    "params_file": params_file,
                    "allow_motion": allow_motion,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", backend, "' == 'moveit'"])
                ),
            ),
        ]
    )
