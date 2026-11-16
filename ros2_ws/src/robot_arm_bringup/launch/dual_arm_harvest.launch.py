import copy
import json
import math
import os
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _read_driver(path):
    with open(path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    params = document["robot_arm_driver"]["ros__parameters"]
    names = params["joint_names"]
    if len(names) != 12 or len(set(names)) != 12:
        raise ValueError("dual_arm_driver.yaml must define 12 unique prefixed joints")
    required = ("zero_ticks", "urdf_zero_ticks", "ticks_per_radian", "directions",
                "min_ticks", "max_ticks")
    arrays = {key: params[key] for key in required}
    if any(len(values) != 12 for values in arrays.values()):
        raise ValueError("all dual-arm calibration arrays must contain 12 entries")
    ik_limits = {}
    for side in ("left", "right"):
        lower, upper = [], []
        for joint in range(1, 6):
            index = names.index(f"{side}_joint{joint}")
            scale = float(arrays["directions"][index]) * float(arrays["ticks_per_radian"][index])
            model_zero = float(arrays["urdf_zero_ticks"][index])
            lo = (float(arrays["min_ticks"][index]) - model_zero) / scale
            hi = (float(arrays["max_ticks"][index]) - model_zero) / scale
            if not all(math.isfinite(value) for value in (scale, lo, hi)) or scale == 0 or lo == hi:
                raise ValueError(f"invalid calibrated IK limits for {side}_joint{joint}")
            lower.append(min(lo, hi))
            upper.append(max(lo, hi))
        ik_limits[f"{side}_ik_lower_limits"] = lower
        ik_limits[f"{side}_ik_upper_limits"] = upper
    return params, ik_limits


def _active_driver_overrides(driver, use_motor):
    if not use_motor:
        return {}
    # Keep the full calibration for IK, but poll/control only the ten physical
    # arm joints when both former servo grippers have been replaced by strikers.
    indices = [i for i, name in enumerate(driver["joint_names"])
               if name not in ("left_gripper", "right_gripper")]
    keys = ("joint_names", "servo_ids", "servo_protocols", "zero_ticks", "urdf_zero_ticks",
            "ticks_per_radian", "directions", "min_ticks", "max_ticks")
    if any(len(driver[key]) != 12 for key in keys):
        raise ValueError("all dual-arm driver arrays must contain 12 entries")
    return {key: [driver[key][i] for i in indices] for key in keys}


def _read_transforms(path):
    with open(path, "r", encoding="utf-8") as stream:
        document = json.load(stream)
    if document.get("source_frame") != "camera_color_optical_frame":
        raise ValueError("camera transform source_frame must be camera_color_optical_frame")
    result = {}
    for side in ("left", "right"):
        item = document[side]
        expected_frame = f"{side}_base_link"
        if item.get("target_frame") != expected_frame:
            raise ValueError(f"{side} target_frame must be {expected_frame}")
        matrix = item.get("camera_to_base")
        if not isinstance(matrix, list) or len(matrix) != 4 or any(
                not isinstance(row, list) or len(row) != 4 for row in matrix):
            raise ValueError(f"{side} camera_to_base must be a 4x4 matrix")
        flat = [float(value) for row in matrix for value in row]
        _validate_transform(flat)
        result[f"{side}_camera_to_base"] = flat
        result[f"{side}_transform_calibrated"] = item.get("calibrated") is True
    return result


def _validate_transform(matrix):
    if len(matrix) != 16 or not all(math.isfinite(value) for value in matrix):
        raise ValueError("camera_to_base must contain 16 finite values")
    if any(abs(matrix[12 + i] - value) > 1e-6
           for i, value in enumerate((0, 0, 0, 1))):
        raise ValueError("camera_to_base has an invalid homogeneous last row")
    rotation = [matrix[4 * row:4 * row + 3] for row in range(3)]
    for i in range(3):
        for j in range(3):
            if abs(sum(rotation[k][i] * rotation[k][j] for k in range(3))
                   - float(i == j)) > 1e-4:
                raise ValueError("camera_to_base rotation must be orthonormal")
    a, b, c = rotation
    determinant = (a[0] * (b[1] * c[2] - b[2] * c[1])
                   - a[1] * (b[0] * c[2] - b[2] * c[0])
                   + a[2] * (b[0] * c[1] - b[1] * c[0]))
    if abs(determinant - 1) > 1e-4:
        raise ValueError("camera_to_base rotation must have determinant +1")


def _camera_to_base_mount(matrix):
    """URDF camera-parent/base-child placement is inverse(T_base_camera)."""
    _validate_transform(matrix)
    rotation = [[matrix[4 * col + row] for col in range(3)] for row in range(3)]
    xyz = [-sum(rotation[row][col] * matrix[4 * col + 3] for col in range(3))
           for row in range(3)]
    horizontal = math.hypot(rotation[0][0], rotation[1][0])
    pitch = math.atan2(-rotation[2][0], horizontal)
    if horizontal > 1e-9:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.0
    return xyz, (roll, pitch, yaw)


def _dual_arm_description(description, transforms):
    """Reuse the single-arm geometry with names matching the shared driver."""
    single = ET.fromstring(description)
    dual = ET.Element("robot", name="dual_mango_arm")
    ET.SubElement(dual, "link", name="camera_color_optical_frame")
    for side in ("left", "right"):
        prefix = side + "_"
        for element in single:
            if element.tag not in ("link", "joint", "material"):
                raise ValueError(f"unsupported single-arm URDF element: {element.tag}")
            clone = copy.deepcopy(element)
            clone.set("name", prefix + clone.attrib["name"])
            for material in clone.iter("material"):
                if material is not clone and "name" in material.attrib:
                    material.set("name", prefix + material.attrib["name"])
            if clone.tag == "joint":
                for tag in ("parent", "child"):
                    reference = clone.find(tag)
                    reference.set("link", prefix + reference.attrib["link"])
                mimic = clone.find("mimic")
                if mimic is not None:
                    mimic.set("joint", prefix + mimic.attrib["joint"])
            dual.append(clone)
        xyz, rpy = _camera_to_base_mount(transforms[f"{side}_camera_to_base"])
        mount = ET.SubElement(dual, "joint", name=prefix + "camera_mount", type="fixed")
        ET.SubElement(mount, "parent", link="camera_color_optical_frame")
        ET.SubElement(mount, "child", link=prefix + "base_link")
        ET.SubElement(mount, "origin", xyz=" ".join(format(v, ".17g") for v in xyz),
                      rpy=" ".join(format(v, ".17g") for v in rpy))
    return ET.tostring(dual, encoding="unicode")


def _launch_nodes(context):
    for name in ("arm_speeds", "move_duration_ms"):
        if LaunchConfiguration(name).perform(context).strip():
            raise ValueError(f"{name} has been split: use top_{name}, bottom_{name}, home_{name}")
    driver_path = LaunchConfiguration("driver_params").perform(context)
    transforms_path = LaunchConfiguration("camera_transforms").perform(context)
    harvest_path = LaunchConfiguration("harvest_params").perform(context)
    with open(harvest_path, "r", encoding="utf-8") as stream:
        harvest_config = yaml.safe_load(stream)["dual_arm_harvest_node"]["ros__parameters"]
    modes = {}
    for name in ("use_motor", "skip_strike"):
        value = LaunchConfiguration(name).perform(context).strip().lower()
        if value and value not in ("true", "false"):
            raise ValueError(f"{name} must be true or false")
        modes[name] = value == "true" if value else harvest_config.get(name, False)
        if not isinstance(modes[name], bool):
            raise ValueError(f"YAML {name} must be a boolean")
    driver, ik_limits = _read_driver(driver_path)
    transforms = _read_transforms(transforms_path)
    description_share = get_package_share_directory("robot_arm_description")
    with open(os.path.join(description_share, "urdf", "imported", "arm_urdf_v1.urdf"),
              "r", encoding="utf-8") as stream:
        description = stream.read()
    description = _dual_arm_description(description, transforms)
    runtime = {
        "driver_joint_names": driver["joint_names"],
        "zero_ticks": driver["zero_ticks"],
        "urdf_zero_ticks": driver["urdf_zero_ticks"],
        "ticks_per_radian": driver["ticks_per_radian"],
        "directions": driver["directions"],
        "min_ticks": driver["min_ticks"],
        "max_ticks": driver["max_ticks"],
        **ik_limits,
        **transforms,
        "left_ticks_calibrated": ParameterValue(
            LaunchConfiguration("left_ticks_calibrated"), value_type=bool),
        "right_ticks_calibrated": ParameterValue(
            LaunchConfiguration("right_ticks_calibrated"), value_type=bool),
        "gripper_speed": ParameterValue(LaunchConfiguration("gripper_speed"), value_type=int),
        "target_tool_tilt": ParameterValue(LaunchConfiguration("target_tool_tilt"), value_type=float),
        "target_joint5": ParameterValue(LaunchConfiguration("target_joint5"), value_type=float),
        "gripper_pregrasp_native_rad": ParameterValue(
            LaunchConfiguration("gripper_pregrasp_native_rad"), value_type=float),
        "gripper_close_native_rad": ParameterValue(
            LaunchConfiguration("gripper_close_native_rad"), value_type=float),
        "move_timeout_ms": ParameterValue(LaunchConfiguration("move_timeout_ms"), value_type=int),
        "home_timeout_ms": ParameterValue(LaunchConfiguration("home_timeout_ms"), value_type=int),
        "open_delay_ms": ParameterValue(LaunchConfiguration("open_delay_ms"), value_type=int),
        "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
        "allow_motion": ParameterValue(LaunchConfiguration("allow_motion"), value_type=bool),
        "stop_at_target": ParameterValue(LaunchConfiguration("stop_at_target"), value_type=bool),
    }
    # Blank launch arguments leave the YAML value in effect; explicit arguments override it.
    # Motion defaults live in the C++ coordinator. Blank arguments preserve them.
    for prefix in ("top", "bottom", "home"):
        speed_name = prefix + "_arm_speeds"
        value = LaunchConfiguration(speed_name).perform(context).strip()
        if value:
            runtime[speed_name] = [int(item) for item in value.split(",")]
        duration_name = prefix + "_move_duration_ms"
        if LaunchConfiguration(duration_name).perform(context).strip():
            runtime[duration_name] = ParameterValue(LaunchConfiguration(duration_name), value_type=int)
    for name, value_type in (("use_motor", bool), ("skip_strike", bool),
                             ("use_size_profiles", bool), ("gripper_duration_ms", int),
                             ("gripper_open_native_rad", float), ("motor_return_delay_s", float)):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            runtime[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": ParameterValue(description, value_type=str)}]),
        Node(package="robot_arm_driver", executable="robot_arm_hx_driver_node",
             name="robot_arm_driver", output="screen",
             parameters=[driver_path, {
                 **_active_driver_overrides(driver, modes["use_motor"]),
                 "allow_motion": ParameterValue(
                     LaunchConfiguration("allow_motion"), value_type=bool)}]),
        Node(package="robot_arm_node", executable="dual_arm_harvest_node",
             name="dual_arm_harvest_node", output="screen", parameters=[harvest_path, runtime],
             remappings=[("/vision/harvest_targets", LaunchConfiguration("targets_topic"))]),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(description_share, "rviz", "dual_arm.rviz")]),
    ]
    # Position calibration also works before the striker STM32 is connected.
    stop_at_target = LaunchConfiguration("stop_at_target").perform(context).strip().lower() == "true"
    if modes["use_motor"] and not modes["skip_strike"] and not stop_at_target:
        nodes.append(Node(
            package="robot_end_effector", executable="serial_striker_node.py",
            name="serial_striker_node", output="screen",
            parameters=[LaunchConfiguration("motor_params"), {
                "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                "allow_motion": ParameterValue(LaunchConfiguration("allow_motion"), value_type=bool)}]))
    return nodes


def generate_launch_description():
    driver_share = get_package_share_directory("robot_arm_driver")
    vision_share = get_package_share_directory("robot_vision")
    bringup_share = get_package_share_directory("robot_arm_bringup")
    motor_share = get_package_share_directory("robot_end_effector")
    return LaunchDescription([
        DeclareLaunchArgument(
            "driver_params",
            default_value=os.path.join(driver_share, "config", "dual_arm_driver.yaml")),
        DeclareLaunchArgument(
            "camera_transforms",
            default_value=os.path.join(vision_share, "config", "camera_to_bases.json")),
        DeclareLaunchArgument(
            "harvest_params",
            default_value=os.path.join(bringup_share, "config", "dual_arm_harvest.yaml")),
        DeclareLaunchArgument("use_size_profiles", default_value="",
                             description="Override YAML size selection; true selects by target.is_small"),
        DeclareLaunchArgument("use_motor", default_value="",
                             description="Trigger a separate STM32 over serial instead of servo grippers"),
        DeclareLaunchArgument("skip_strike", default_value="",
                             description="Motor mode: move, wait, return without starting a striker node or sending serial"),
        DeclareLaunchArgument("motor_params", default_value=os.path.join(
            motor_share, "config", "serial_striker.yaml")),
        DeclareLaunchArgument("motor_return_delay_s", default_value="",
                             description="Local seconds after STM32 trigger before returning home (default 1); no MCU feedback"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("allow_motion", default_value="false"),
        DeclareLaunchArgument("stop_at_target", default_value="false",
                             description="Move only the selected arm joints; hold at target without gripping or returning"),
        DeclareLaunchArgument("targets_topic", default_value="/vision/harvest_targets",
                             description="Use /manual/harvest_targets for manual one-shot target tests"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("left_ticks_calibrated", default_value="false"),
        DeclareLaunchArgument("right_ticks_calibrated", default_value="false"),
        DeclareLaunchArgument("arm_speeds", default_value="",
                             description="Removed shared setting; use top/bottom/home_arm_speeds"),
        DeclareLaunchArgument("top_arm_speeds", default_value="",
                             description="Optional joint1..5 speeds for top fruit; blank uses C++ defaults"),
        DeclareLaunchArgument("bottom_arm_speeds", default_value="",
                             description="Optional joint1..5 speeds for bottom fruit; blank uses C++ defaults"),
        DeclareLaunchArgument("home_arm_speeds", default_value="",
                             description="Optional joint1..5 speeds for normal and timeout home"),
        DeclareLaunchArgument("gripper_speed", default_value="20"),
        DeclareLaunchArgument("target_tool_tilt", default_value="0.0"),
        DeclareLaunchArgument("target_joint5", default_value="0.0"),
        DeclareLaunchArgument("gripper_pregrasp_native_rad", default_value="-0.2"),
        DeclareLaunchArgument("gripper_close_native_rad", default_value="-2.0"),
        DeclareLaunchArgument("gripper_open_native_rad", default_value="",
                             description="Override YAML release angle (native radians)"),
        DeclareLaunchArgument("move_duration_ms", default_value="",
                             description="Removed shared setting; use top/bottom/home_move_duration_ms"),
        DeclareLaunchArgument("top_move_duration_ms", default_value="",
                             description="V3 joint5 time for top fruit; blank uses C++ default"),
        DeclareLaunchArgument("bottom_move_duration_ms", default_value="",
                             description="V3 joint5 time for bottom fruit; blank uses C++ default"),
        DeclareLaunchArgument("home_move_duration_ms", default_value="",
                             description="V3 joint5 time for returning home; blank uses C++ default"),
        DeclareLaunchArgument("gripper_duration_ms", default_value="",
                             description="Override YAML V3 gripper travel time (default 2000 ms)"),
        DeclareLaunchArgument("move_timeout_ms", default_value="15000"),
        DeclareLaunchArgument("home_timeout_ms", default_value="15000"),
        DeclareLaunchArgument("open_delay_ms", default_value="500"),
        OpaqueFunction(function=_launch_nodes),
    ])
