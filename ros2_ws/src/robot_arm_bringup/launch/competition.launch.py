import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_arm_bringup")
    chassis_share = get_package_share_directory("robot_chassis")
    sim_launch = os.path.join(bringup_share, "launch", "sim.launch.py")
    real_launch = os.path.join(bringup_share, "launch", "real.launch.py")
    chassis_launch = os.path.join(chassis_share, "launch", "chassis.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim",
                default_value="true",
                description="Use Gazebo arm bringup when true; use the guarded real driver otherwise",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                condition=IfCondition(LaunchConfiguration("use_sim")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_launch),
                condition=UnlessCondition(LaunchConfiguration("use_sim")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(chassis_launch),
            ),
            # sim.launch.py starts robot_state_publisher but not the IK action
            # server. The real launch already starts it, so this node is only
            # enabled for simulation.
            Node(
                package="robot_arm_node",
                executable="ik_pick_node",
                name="ik_pick_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_sim")),
                parameters=[{"base_link": "base_link", "tip_link": "tcp_link"}],
            ),
        ]
    )
