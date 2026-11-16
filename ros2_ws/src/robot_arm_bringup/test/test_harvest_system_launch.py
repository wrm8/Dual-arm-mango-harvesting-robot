"""Check unified startup wiring and shared motion interlocks without hardware."""
import importlib.util
import sys
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.utilities import evaluate_parameters, normalize_parameters


PACKAGE = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


system = load("harvest_system_launch", PACKAGE / "launch/harvest_system.launch.py")
chassis = load("harvest_chassis_launch", PACKAGE.parent / "robot_chassis/launch/chassis.launch.py")


class HarvestSystemLaunch(unittest.TestCase):
    def generate(self, **overrides):
        with patch.object(system, "get_package_share_directory",
                          side_effect=lambda name: str(PACKAGE.parent / name)):
            description = system.generate_launch_description()
        context = LaunchContext()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
        context.launch_configurations.update(overrides)
        return description, context

    @staticmethod
    def arguments(action, context):
        return {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                for name, value in action.launch_arguments}

    def test_system_wires_vision_arms_and_home_before_delayed_chassis(self):
        description, context = self.generate()
        includes = [action for action in description.entities if isinstance(action, IncludeLaunchDescription)]
        arms = next(action for action in includes if "use_motor" in self.arguments(action, context))
        vision = next(action for action in includes if "publish_targets" in self.arguments(action, context))
        arm_args = self.arguments(arms, context)
        self.assertEqual(arm_args["targets_topic"], "/vision/harvest_targets")
        self.assertEqual(arm_args["use_motor"], "true")
        self.assertEqual(arm_args["skip_strike"], "true")
        self.assertEqual(arm_args["stop_at_target"], "false")
        for prefix in ("top", "bottom", "home"):
            self.assertEqual(arm_args[prefix + "_arm_speeds"], "")
            self.assertEqual(arm_args[prefix + "_move_duration_ms"], "")
        self.assertEqual(arm_args["dry_run"], "true")
        self.assertEqual(arm_args["allow_motion"], "false")
        self.assertEqual(self.arguments(vision, context)["publish_targets"], "true")
        self.assertFalse(any(isinstance(action, TimerAction) for action in description.entities))
        home = next(action for action in description.entities if isinstance(action, system.Node))
        base_placeholder = object()
        with patch.object(system.time, 'monotonic', return_value=3.0):
            timer, = system._on_system_exit(SimpleNamespace(action=home, returncode=0),
                                           context, home, base_placeholder, 0.0)
        self.assertTrue(timer.condition.evaluate(context))
        self.assertEqual(timer.period, 7.0)
        self.assertEqual(context.launch_configurations["chassis_start_delay_s"], "10.0")
        self.assertEqual(timer.actions, [base_placeholder])
        # The base include is supplied to the exit callback, not launched initially.
        # Interlocks are independently checked against the actual child launch below.
        self.assertEqual(len(includes), 2)

    def test_failed_home_and_other_process_exits_never_start_chassis(self):
        _, context = self.generate()
        home, base = object(), object()
        for action, code in ((home, 1), (object(), 0), (object(), 1)):
            actions = system._on_system_exit(SimpleNamespace(action=action, returncode=code),
                                             context, home, base, 0.0)
            self.assertFalse(any(isinstance(action, TimerAction) for action in actions))

    def test_real_process_exit_only_releases_base_after_successful_home(self):
        for code in (0, 1):
            with self.subTest(home_exit_code=code):
                released = []
                home = ExecuteProcess(cmd=[sys.executable, '-c', f'raise SystemExit({code})'])
                def start_base(context):
                    released.append(True)
                    return [EmitEvent(event=Shutdown(reason='test base released'))]
                base = OpaqueFunction(function=start_base)
                description = LaunchDescription([
                    DeclareLaunchArgument('chassis_start_delay_s', default_value='0.0'),
                    DeclareLaunchArgument('enable_chassis', default_value='true'),
                    RegisterEventHandler(OnProcessExit(on_exit=lambda event, context:
                        system._on_system_exit(event, context, home, base, 0.0))),
                    home,
                ])
                service = LaunchService()
                service.include_launch_description(description)
                service.run()
                self.assertEqual(released, [True] if code == 0 else [])

    def test_home_after_minimum_delay_does_not_wait_another_ten_seconds(self):
        _, context = self.generate()
        home = object()
        with patch.object(system.time, 'monotonic', return_value=12.0):
            timer, = system._on_system_exit(SimpleNamespace(action=home, returncode=0),
                                           context, home, object(), 0.0)
        self.assertEqual(timer.period, 0.0)

    def test_base_include_receives_shared_interlocks(self):
        # Capture the deferred child without starting any ROS processes.
        captured = []
        original = system.IncludeLaunchDescription
        def include(*args, **kwargs):
            action = original(*args, **kwargs)
            captured.append(action)
            return action
        with patch.object(system, 'IncludeLaunchDescription', side_effect=include):
            description, context = self.generate()
        base = next(action for action in captured if 'serial_enabled' in self.arguments(action, context))
        base_args = self.arguments(base, context)
        self.assertEqual(base_args["dry_run"], 'true')
        self.assertEqual(base_args["allow_motion"], 'false')
        self.assertEqual(base_args["use_external_imu"], "true")

    def test_arm_only_and_real_striker_options_are_forwarded(self):
        description, context = self.generate(enable_chassis="false", skip_strike="false",
                                             dry_run="false", allow_motion="true")
        home = object()
        timer, = system._on_system_exit(SimpleNamespace(action=home, returncode=0),
                                       context, home, object(), 0.0)
        self.assertFalse(timer.condition.evaluate(context))
        arms = next(action for action in description.entities
                    if isinstance(action, IncludeLaunchDescription)
                    and "use_motor" in self.arguments(action, context))
        self.assertEqual(self.arguments(arms, context)["skip_strike"], "false")

    def test_chassis_launch_applies_system_interlocks_over_yaml(self):
        context = LaunchContext()
        context.launch_configurations.update(params_file="/tmp/chassis.yaml", dry_run="true", allow_motion="false")
        with patch.object(chassis, "Node", side_effect=lambda **kwargs: kwargs):
            nodes = chassis._base_serial_node(context)
        overrides = evaluate_parameters(context, normalize_parameters([nodes[0]["parameters"][1]]))[0]
        self.assertIs(overrides["dry_run"], True)
        self.assertIs(overrides["allow_motion"], False)
        context.launch_configurations.update(dry_run="", allow_motion="")
        with patch.object(chassis, "Node", side_effect=lambda **kwargs: kwargs):
            self.assertEqual(chassis._base_serial_node(context)[0]["parameters"][1], {})


if __name__ == "__main__":
    unittest.main()
