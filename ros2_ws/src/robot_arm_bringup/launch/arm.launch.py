import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    driver_share = get_package_share_directory('robot_arm_driver')
    default_config = os.path.join(driver_share, 'config', 'ik_driver.yaml')
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_config),
            Node(
                package='robot_arm_driver',
                executable='robot_arm_ik_node',
                name='robot_arm_ik_driver',
                output='screen',
                parameters=[LaunchConfiguration('params_file')],
            ),
        ]
    )
