import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_model = PathJoinSubstitution(
        [FindPackageShare('robot_arm_description'), 'urdf', 'imported', 'arm_urdf_v1.urdf']
    )

    return LaunchDescription(
        [   #这两个节点在 ROS 官方生态里就是一套标准的“模型可视化搭档”，专门用来在 RViz 里把 URDF 模型正确地显示出来，并让它动起来。
            #只有在不使用标准仿真器、纯做 URDF 可视化调试时，才需要 joint_state_publisher 或 GUI 这类临时节点来“冒充”关节状态源。
            #一旦进入真正的仿真流程，就应该把这类调试工具移除，换成仿真器自带的状态发布机制
            DeclareLaunchArgument('model', default_value=default_model),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {
                        'robot_description': ParameterValue(
                            Command(['cat ', LaunchConfiguration('model')]),
                            value_type=str,
                        )
                    }
                ],
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
            ),
            # Node(
            #     package='joint_state_publisher',
            #     executable='joint_state_publisher',
            #     name='joint_state_publisher',
            #     output='screen',
            # ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d',
                    os.path.join(
                        get_package_share_directory('robot_arm_description'),
                        'rviz',
                        'robot_arm.rviz',
                    ),
                ],
            ),
        ]
    )
