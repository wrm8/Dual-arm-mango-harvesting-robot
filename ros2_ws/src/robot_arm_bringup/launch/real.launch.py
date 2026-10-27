import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("robot_arm_description")
    driver_share = get_package_share_directory("robot_arm_driver")

    default_description = os.path.join(
        description_share, "urdf", "imported", "arm_urdf_v1.urdf"
    )
    default_driver_params = os.path.join(
        driver_share, "config", "arm_driver.yaml"
    )

    with open(default_description, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=default_driver_params
            ),
            DeclareLaunchArgument("allow_motion", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="robot_arm_driver",
                executable="robot_arm_hx_driver_node",
                name="robot_arm_driver",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "allow_motion": ParameterValue(
                            LaunchConfiguration("allow_motion"), value_type=bool
                        )
                    },
                ],
            ),
            Node(
                package="robot_arm_node",
                executable="ik_pick_node",
                name="ik_pick_node",
                output="screen",
                parameters=[
                    {
                        "base_link": "base_link",
                        "tip_link": "tcp_link",
                    }
                ],
            ),
        ]
    )
