import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robot_chassis")
    default_params = os.path.join(share, "config", "chassis.yaml")
    default_tree = os.path.join(share, "config", "mission_tree.xml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("tree_file", default_value=default_tree),
            Node(
                package="robot_chassis",
                executable="base_serial_node",
                name="base_serial_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                package="robot_chassis",
                executable="ultrasonic_node",
                name="ultrasonic_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                package="robot_chassis",
                executable="row_navigation_node",
                name="row_navigation_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                package="robot_chassis",
                executable="mission_behavior_node",
                name="mission_behavior_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"tree_file": LaunchConfiguration("tree_file")},
                ],
            ),
        ]
    )
