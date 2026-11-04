import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robot_vision')
    defaults = {'params_file': os.path.join(share, 'config', 'vision.yaml'),
                'weights': os.path.join(share, 'models', 'best.pt'),
                'source': 'realsense', 'device': '0',
                'publish_targets': 'false', 'calibration': '', 'decision_config': ''}
    arguments = [DeclareLaunchArgument(key, default_value=value)
                 for key, value in defaults.items()]
    overrides = {key: ParameterValue(LaunchConfiguration(key), value_type=str)
                 for key in ('weights', 'source', 'device', 'calibration', 'decision_config')}
    overrides['publish_targets'] = ParameterValue(
        LaunchConfiguration('publish_targets'), value_type=bool)
    return LaunchDescription(arguments + [Node(
        package='robot_vision', executable='mango_vision_node.py',
        name='mango_vision_node', output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides])])
