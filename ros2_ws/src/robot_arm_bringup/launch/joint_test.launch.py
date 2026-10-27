import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_arm_driver'), 'config', 'arm_driver.yaml')
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_config),
            DeclareLaunchArgument('allow_motion', default_value='false'),
            Node(
                package='robot_arm_driver',
                executable='robot_arm_hx_driver_node',
                name='robot_arm_driver',
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'allow_motion': ParameterValue(
                            LaunchConfiguration('allow_motion'), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
