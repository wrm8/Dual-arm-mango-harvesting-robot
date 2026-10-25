import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import AppendEnvironmentVariable
from launch.actions import TimerAction


def generate_launch_description():
    description_share = get_package_share_directory("robot_arm_description")
    gazebo_share = get_package_share_directory("robot_arm_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    model_path = os.path.join(
        description_share, "urdf", "sim", "arm_urdf_v1_sim.urdf"
    )
    world_path = os.path.join(gazebo_share, "worlds", "arm_test.sdf")
    gz_launch_path = os.path.join(
        ros_gz_sim_share, "launch", "gz_sim.launch.py"
    )

    # 通用替换所有 $(find xxx)
    def replace_find(text):
        return re.sub(
            r"\$\(find ([^)]+)\)",
            lambda m: get_package_share_directory(m.group(1)),
            text,
        )

    robot_description = replace_find(open(model_path).read())

    # # 读 URDF 后做替换
    # robot_description_str = open(model_path).read()
    # robot_description_str = robot_description_str.replace(
    #     "$(find robot_arm_gazebo)",
    #     gazebo_share,
    # )
    # robot_description = robot_description_str
    # # robot_description = ParameterValue(
    # #     Command(["cat ", model_path]),
    # #     value_type=str,
    # # )

    env_var = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',  # 或 GZ_SIM_RESOURCE_PATH
        os.path.dirname(description_share),  # 指向 /home/wrm/robot_arm_jetson/ros2_ws/install/robot_arm_description/share
    )

    return LaunchDescription([
        env_var,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={"gz_args": ["-r -v 4 ", world_path]}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True,
            }],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            output="screen",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_robot_arm",
            arguments=[
                "-topic", "robot_description",
                "-name", "robot_arm",
                "-x", "0.0", "-y", "0.0", "-z", "0.1",
            ],
            output="screen",
        ),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_controller"],
                    output="screen",
                ),
            ],
        ),
    ])
