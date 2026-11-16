from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    defaults = {'host': '127.0.0.1', 'port': '8765', 'log_dir': '~/.ros/robot_monitor',
                'chassis_config': '', 'start_threshold': '0.03', 'delay_warning_ms': '100'}
    declarations = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    arguments = []
    for key in defaults:
        arguments.extend(['--' + key.replace('_', '-'), LaunchConfiguration(key)])
    return LaunchDescription(declarations + [Node(
        package='robot_monitor', executable='robot_monitor.py', name='robot_monitor',
        output='screen', arguments=arguments)])
