"""Vision, dual arms and row navigation; supports testing without the striker MCU."""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _on_system_exit(event, context, startup_home, base, started_at):
    if event.action is startup_home and event.returncode == 0:
        # Keep the original minimum startup delay, but never release the
        # chassis before a successful home result. This process is one-shot.
        delay = float(LaunchConfiguration("chassis_start_delay_s").perform(context))
        return [TimerAction(period=max(0.0, delay - (time.monotonic() - started_at)),
                            actions=[base],
                            condition=IfCondition(LaunchConfiguration("enable_chassis")))]
    return [EmitEvent(event=Shutdown(
        reason="A harvest-system process exited or startup home failed; stopping the system"))]


def generate_launch_description():
    started_at = time.monotonic()
    bringup = get_package_share_directory("robot_arm_bringup")
    chassis = get_package_share_directory("robot_chassis")
    sensors = get_package_share_directory("robot_sensors")
    vision = get_package_share_directory("robot_vision")
    driver = get_package_share_directory("robot_arm_driver")
    motor = get_package_share_directory("robot_end_effector")
    defaults = {
        "driver_params": os.path.join(driver, "config", "dual_arm_driver.yaml"),
        "camera_transforms": os.path.join(vision, "config", "camera_to_bases.json"),
        "harvest_params": os.path.join(bringup, "config", "dual_arm_harvest.yaml"),
        "motor_params": os.path.join(motor, "config", "serial_striker.yaml"),
        "vision_params": os.path.join(vision, "config", "vision.yaml"),
        "chassis_params": os.path.join(chassis, "config", "chassis.yaml"),
        "sensor_params": os.path.join(sensors, "config", "ultrasonic.yaml"),
        "imu_params": os.path.join(sensors, "config", "wit_imu.yaml"),
        "dry_run": "true", "allow_motion": "false",
        "use_motor": "true", "skip_strike": "true", "stop_at_target": "false",
        "use_size_profiles": "true", "motor_return_delay_s": "1.0",
        "left_ticks_calibrated": "false", "right_ticks_calibrated": "false",
        # Blank motion overrides leave the three C++ profiles authoritative.
        "top_arm_speeds": "", "bottom_arm_speeds": "", "home_arm_speeds": "",
        "top_move_duration_ms": "", "bottom_move_duration_ms": "", "home_move_duration_ms": "",
        "arm_speeds": "", "move_duration_ms": "",  # Deprecated: child reports migration error.
        "move_timeout_ms": "15000", "home_timeout_ms": "15000",
        "target_tool_tilt": "0.0", "target_joint5": "0.0", "rviz": "false",
        "enable_chassis": "true", "chassis_start_delay_s": "10.0",
    }
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, value in defaults.items()]
    arm_names = (
        "driver_params", "camera_transforms", "harvest_params", "motor_params",
        "dry_run", "allow_motion", "use_motor", "skip_strike", "stop_at_target",
        "use_size_profiles", "motor_return_delay_s", "left_ticks_calibrated",
        "right_ticks_calibrated", "arm_speeds", "move_duration_ms", "move_timeout_ms",
        "home_timeout_ms", "target_tool_tilt", "target_joint5", "rviz",
        "top_arm_speeds", "bottom_arm_speeds", "home_arm_speeds",
        "top_move_duration_ms", "bottom_move_duration_ms", "home_move_duration_ms",
    )
    arms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "dual_arm_harvest.launch.py")),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in arm_names},
            "targets_topic": "/vision/harvest_targets",
        }.items(),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vision, "launch", "vision.launch.py")),
        launch_arguments={"params_file": LaunchConfiguration("vision_params"),
                          "publish_targets": "true"}.items(),
    )
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(chassis, "launch", "chassis.launch.py")),
        launch_arguments={
            "params_file": LaunchConfiguration("chassis_params"),
            "sensor_params_file": LaunchConfiguration("sensor_params"),
            "imu_params_file": LaunchConfiguration("imu_params"),
            "dry_run": LaunchConfiguration("dry_run"),
            "allow_motion": LaunchConfiguration("allow_motion"),
            "serial_enabled": "true", "use_external_imu": "true",
        }.items(),
    )
    startup_home = Node(package="robot_arm_bringup", executable="startup_home.py",
                        name="startup_arm_home", output="screen")
    # The successful one-shot home is expected to exit; all other exits stop
    # the system. No chassis processes are launched while home is pending.
    stop_on_exit = RegisterEventHandler(OnProcessExit(on_exit=lambda event, context:
        _on_system_exit(event, context, startup_home, base, started_at)))
    return LaunchDescription(arguments + [
        stop_on_exit, arms, camera, startup_home,
    ])
