import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _calibrated_ik_limits(params_file):
    """Use the exact JointState/FJT tick mapping, leaving calibration unchanged."""
    with open(params_file, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    try:
        params = document["robot_arm_driver"]["ros__parameters"]
        names = params["joint_names"]
        native_zero = params["zero_ticks"]
        model_zero = params.get("urdf_zero_ticks") or native_zero
        scales = params["ticks_per_radian"]
        directions = params["directions"]
        minimum = params["min_ticks"]
        maximum = params["max_ticks"]
        arrays = (native_zero, model_zero, scales, directions, minimum, maximum)
        if not isinstance(names, list) or not names or len(set(names)) != len(names):
            raise ValueError("joint_names must be a nonempty list with unique names")
        if any(not isinstance(a, list) or len(a) != len(names) for a in arrays):
            raise ValueError("all calibration arrays must match joint_names")
        lower, upper = [], []
        for name in ("joint1", "joint2", "joint3", "joint4", "joint5"):
            index = names.index(name)
            native, zero, scale, direction, lo, hi = (
                float(array[index]) for array in arrays
            )
            if not all(math.isfinite(v) for v in (native, zero, scale, direction, lo, hi)):
                raise ValueError(f"non-finite calibration for {name}")
            if scale <= 0 or abs(direction) != 1 or lo >= hi:
                raise ValueError(f"invalid scale/direction/tick limits for {name}")
            a = (lo - zero) / (direction * scale)
            b = (hi - zero) / (direction * scale)
            lower.append(min(a, b))
            upper.append(max(a, b))
        return lower, upper
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Cannot derive IK limits from {params_file}: {error}") from error


def _launch_ik_node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    lower, upper = _calibrated_ik_limits(params_file)
    return [Node(
        package="robot_arm_node",
        executable="ik_pick_node",
        name="ik_pick_node",
        output="screen",
        parameters=[{
            "base_link": "base_link",
            "tip_link": "tcp_link",
            "ik_solver": LaunchConfiguration("ik_solver").perform(context),
            "ik_lower_limits": lower,
            "ik_upper_limits": upper,
        }],
    )]


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
            DeclareLaunchArgument(
                "ik_solver", default_value="analytic", choices=["analytic", "numerical"],
                description="Direct IK solver; both respect URDF and calibrated limits",
            ),
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
            OpaqueFunction(function=_launch_ik_node),
        ]
    )
