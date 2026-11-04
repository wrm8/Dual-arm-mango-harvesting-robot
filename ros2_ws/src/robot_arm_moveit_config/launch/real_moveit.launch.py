import os
import math
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _read_yaml(path):
    with open(path, "r", encoding="utf-8") as yaml_file:
        return yaml.safe_load(yaml_file) or {}


def _launch_nodes(context):
    description_share = get_package_share_directory("robot_arm_description")
    moveit_share = get_package_share_directory("robot_arm_moveit_config")

    urdf_path = os.path.join(
        description_share, "urdf", "imported", "arm_urdf_v1.urdf"
    )
    config_dir = os.path.join(moveit_share, "config")
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()
    with open(os.path.join(config_dir, "arm.srdf"), "r", encoding="utf-8") as srdf_file:
        robot_description_semantic = srdf_file.read()

    kinematics = _read_yaml(os.path.join(config_dir, "kinematics.yaml"))
    joint_limits = _read_yaml(os.path.join(config_dir, "joint_limits.yaml"))
    ompl = _read_yaml(os.path.join(config_dir, "ompl_planning.yaml"))
    controllers = _read_yaml(os.path.join(config_dir, "moveit_controllers.yaml"))

    # Read the selected hardware calibration, rather than a second copy of tick limits.
    params_file = LaunchConfiguration("params_file").perform(context)
    calibration = _read_yaml(params_file)["robot_arm_driver"]["ros__parameters"]
    model = ET.fromstring(robot_description)
    limits = joint_limits["robot_description_planning"]["joint_limits"]
    for joint in model.findall("joint"):
        name = joint.attrib["name"]
        if name not in limits:
            continue
        index = calibration["joint_names"].index(name)
        zero = float(calibration["zero_ticks"][index])
        scale = float(calibration["directions"][index]) * float(
            calibration["ticks_per_radian"][index]
        )
        if not math.isfinite(scale) or scale == 0.0 or not math.isfinite(zero):
            raise ValueError(f"Invalid calibration for {name}")
        endpoints = [(calibration[key][index] - zero) / scale
                     for key in ("min_ticks", "max_ticks")]
        limit = joint.find("limit")
        lower = max(float(limit.attrib["lower"]), min(endpoints))
        upper = min(float(limit.attrib["upper"]), max(endpoints))
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f"Invalid calibrated limits for {name}")
        limit.set("lower", str(lower))
        limit.set("upper", str(upper))
        limits[name].update(min_position=lower, max_position=upper)
        # These conservative limits shape the MoveIt timing, not servo speed.
        limits[name].update(has_velocity_limits=True, max_velocity=0.3,
                            has_acceleration_limits=True, max_acceleration=0.3)
    robot_description = ET.tostring(model, encoding="unicode")

    move_group_parameters = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": kinematics,
        "robot_description_planning": joint_limits["robot_description_planning"],
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl["ompl"],
    }
    move_group_parameters.update(controllers)

    return [
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
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[move_group_parameters],
            ),
        ]


def generate_launch_description():
    driver_share = get_package_share_directory("robot_arm_driver")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            driver_share, "config", "arm_driver.yaml")),
        DeclareLaunchArgument("allow_motion", default_value="false"),
        OpaqueFunction(function=_launch_nodes),
    ])
