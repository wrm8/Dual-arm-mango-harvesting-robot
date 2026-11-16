import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _base_serial_node(context):
    # Standalone launch keeps the YAML switches. The system entry can override
    # both chassis and arm interlocks together, including a dry run.
    overrides = {}
    for name in ("dry_run", "allow_motion"):
        value = LaunchConfiguration(name).perform(context).strip().lower()
        if value:
            if value not in ("true", "false"):
                raise ValueError(f"{name} must be true or false")
            overrides[name] = value == "true"
    return [Node(
        package="robot_chassis", executable="base_serial_node",
        name="base_serial_node", output="screen",
        parameters=[LaunchConfiguration("params_file"), overrides],
    )]


def generate_launch_description():
    share = get_package_share_directory("robot_chassis")
    sensors_share = get_package_share_directory("robot_sensors")
    default_params = os.path.join(share, "config", "chassis.yaml")
    default_tree = os.path.join(share, "config", "mission_tree.xml")
    default_sensor_params = os.path.join(sensors_share, "config", "ultrasonic.yaml")
    default_imu_params = os.path.join(sensors_share, "config", "wit_imu.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("tree_file", default_value=default_tree),
            DeclareLaunchArgument("sensor_params_file", default_value=default_sensor_params),
            DeclareLaunchArgument("imu_params_file", default_value=default_imu_params),
            DeclareLaunchArgument("dry_run", default_value="",
                                 description="Override base_serial_node YAML dry_run when nonempty"),
            DeclareLaunchArgument("allow_motion", default_value="",
                                 description="Override base_serial_node YAML allow_motion when nonempty"),
            DeclareLaunchArgument(
                "serial_enabled",
                default_value="true",
                description="Read DYP-A21 sensors from CH9344 UART devices",
            ),
            DeclareLaunchArgument(
                "use_external_imu",
                default_value=LaunchConfiguration("serial_enabled"),
                description="Use WIT heading; false explicitly selects odometry heading for simulation/legacy",
            ),
            OpaqueFunction(function=_base_serial_node),
            Node(
                package="robot_sensors",
                executable="ultrasonic_node",
                name="ultrasonic_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("sensor_params_file"),
                    {
                        "serial_enabled": ParameterValue(
                            LaunchConfiguration("serial_enabled"), value_type=bool
                        )
                    },
                ],
            ),
            Node(
                package="robot_sensors",
                executable="wit_imu_node",
                name="wit_imu_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_external_imu")),
                parameters=[LaunchConfiguration("imu_params_file")],
            ),
            Node(
                package="robot_chassis",
                executable="row_navigation_node",
                name="row_navigation_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_external_imu": ParameterValue(
                        LaunchConfiguration("use_external_imu"), value_type=bool)},
                ],
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
