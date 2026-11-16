"""Verify editable grasp YAML remains authoritative unless explicitly overridden."""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = Path(os.environ.get("MANGO_TEST_SOURCE_ROOT", PACKAGE.parent))
spec = importlib.util.spec_from_file_location("grasp_profile_launch", PACKAGE / "launch/dual_arm_harvest.launch.py")
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


class GraspProfileLaunch(unittest.TestCase):
    def runtime(self, **overrides):
        context = LaunchContext()

        def share(package):
            return str(PACKAGE.parent / package if package in ("robot_arm_bringup", "robot_end_effector")
                       else SOURCE / package)

        with patch.object(launch, "get_package_share_directory", side_effect=share):
            for action in launch.generate_launch_description().entities:
                if isinstance(action, DeclareLaunchArgument):
                    context.launch_configurations[action.name] = perform_substitutions(
                        context, action.default_value)
            context.launch_configurations.update(overrides)
            with patch.object(launch, "Node", side_effect=lambda **values: values):
                actions = launch._launch_nodes(context)
        self.actions = actions
        node = next(action for action in actions if action["name"] == "dual_arm_harvest_node")
        path, runtime = node["parameters"]
        return path, evaluate_parameters(context, normalize_parameters([runtime]))[0]

    def test_default_launch_preserves_yaml_profile_settings(self):
        path, runtime = self.runtime()
        self.assertEqual(Path(path), PACKAGE / "config/dual_arm_harvest.yaml")
        for key in ("use_motor", "use_size_profiles", "gripper_duration_ms", "gripper_open_native_rad"):
            self.assertNotIn(key, runtime)
        params = yaml.safe_load(Path(path).read_text())["dual_arm_harvest_node"]["ros__parameters"]
        self.assertTrue(params["use_size_profiles"])
        self.assertIs(params["use_motor"], False)
        self.assertFalse(any(action["package"] == "robot_end_effector" for action in self.actions))
        self.assertEqual(params["motor_return_delay_s"], 1.0)
        self.assertEqual(params["gripper_duration_ms"], 2000)
        for side in ("left", "right"):
            for size, pregrasp, close in (("small", -1., -2.), ("large", 0., -1.)):
                key = side + "_" + size
                self.assertEqual(len(params[key + "_offset_m"]), 3)
                self.assertEqual(params[key + "_gripper_pregrasp_native_rad"], pregrasp)
                self.assertEqual(params[key + "_gripper_close_native_rad"], close)

    def test_motor_mode_driver_excludes_only_the_two_replaced_grippers(self):
        _, coordinator = self.runtime(use_motor="true", skip_strike="true")
        driver_node = next(action for action in self.actions if action['name'] == 'robot_arm_driver')
        path, override = driver_node['parameters']
        original = yaml.safe_load(Path(path).read_text())['robot_arm_driver']['ros__parameters']
        indices = [i for i, name in enumerate(original['joint_names']) if not name.endswith('gripper')]
        for key in ('joint_names', 'servo_ids', 'servo_protocols', 'zero_ticks', 'urdf_zero_ticks',
                    'ticks_per_radian', 'directions', 'min_ticks', 'max_ticks'):
            self.assertEqual(override[key], [original[key][i] for i in indices])
        self.assertEqual(len(coordinator['driver_joint_names']), 12)
        self.runtime(use_motor="false")
        driver_node = next(action for action in self.actions if action['name'] == 'robot_arm_driver')
        self.assertNotIn('joint_names', driver_node['parameters'][1])

    def test_explicit_launch_values_override_yaml(self):
        _, runtime = self.runtime(use_motor="true", motor_return_delay_s="2.0",
                                  use_size_profiles="false", gripper_duration_ms="3000",
                                  gripper_open_native_rad="-0.3")
        self.assertIs(runtime["use_size_profiles"], False)
        self.assertIs(runtime["use_motor"], True)
        self.assertTrue(any(action["package"] == "robot_end_effector" for action in self.actions))
        self.assertEqual(runtime["motor_return_delay_s"], 2.0)
        motor = next(action for action in self.actions if action["package"] == "robot_end_effector")
        self.assertEqual(motor["executable"], "serial_striker_node.py")
        self.assertNotIn("motor_counts_per_output_rev", runtime)
        self.assertEqual(runtime["gripper_duration_ms"], 3000)
        self.assertEqual(runtime["gripper_open_native_rad"], -.3)

    def test_motion_defaults_stay_in_cpp_and_explicit_profiles_are_forwarded(self):
        _, runtime = self.runtime()
        for prefix in ("top", "bottom", "home"):
            self.assertNotIn(prefix + "_arm_speeds", runtime)
            self.assertNotIn(prefix + "_move_duration_ms", runtime)
        self.assertNotIn("arm_speeds", runtime)
        self.assertNotIn("move_duration_ms", runtime)
        _, runtime = self.runtime(top_arm_speeds="61,310,510,520,21",
                                  bottom_arm_speeds="42,220,330,440,22",
                                  home_arm_speeds="31,110,210,310,23",
                                  top_move_duration_ms="7000", bottom_move_duration_ms="3000",
                                  home_move_duration_ms="1000")
        self.assertEqual(list(runtime['top_arm_speeds']), [61, 310, 510, 520, 21])
        self.assertEqual(list(runtime['bottom_arm_speeds']), [42, 220, 330, 440, 22])
        self.assertEqual(list(runtime['home_arm_speeds']), [31, 110, 210, 310, 23])
        self.assertEqual([runtime[p + '_move_duration_ms'] for p in ('top', 'bottom', 'home')],
                         [7000, 3000, 1000])

    def test_old_shared_speed_arguments_report_migration(self):
        for name, value in (('arm_speeds', '50,300,500,500,20'), ('move_duration_ms', '5000')):
            with self.assertRaisesRegex(ValueError, 'has been split'):
                self.runtime(**{name: value})

    def test_yaml_motor_mode_starts_serial_node_and_can_be_disabled_explicitly(self):
        document = yaml.safe_load((PACKAGE / "config/dual_arm_harvest.yaml").read_text())
        document["dual_arm_harvest_node"]["ros__parameters"]["use_motor"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor_params.yaml"
            path.write_text(yaml.safe_dump(document))
            self.runtime(harvest_params=str(path))
            self.assertTrue(any(action["package"] == "robot_end_effector" for action in self.actions))
            self.runtime(harvest_params=str(path), use_motor="false")
            self.assertFalse(any(action["package"] == "robot_end_effector" for action in self.actions))

    def test_target_only_motor_mode_does_not_need_striker_hardware(self):
        _, runtime = self.runtime(use_motor="true", stop_at_target="true",
                                  motor_params="/nonexistent/striker.yaml")
        self.assertTrue(runtime["use_motor"])
        self.assertTrue(runtime["stop_at_target"])
        self.assertFalse(any(action["package"] == "robot_end_effector" for action in self.actions))

    def test_skipped_strike_runs_full_arm_cycle_without_serial_node(self):
        _, runtime = self.runtime(use_motor="true", skip_strike="true", stop_at_target="false",
                                  motor_params="/nonexistent/striker.yaml")
        self.assertTrue(runtime["use_motor"])
        self.assertTrue(runtime["skip_strike"])
        self.assertFalse(runtime["stop_at_target"])
        self.assertFalse(any(action["package"] == "robot_end_effector" for action in self.actions))

    def test_yaml_skip_strike_can_be_overridden_to_enable_hardware(self):
        document = yaml.safe_load((PACKAGE / "config/dual_arm_harvest.yaml").read_text())
        params = document["dual_arm_harvest_node"]["ros__parameters"]
        params.update(use_motor=True, skip_strike=True)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "params.yaml"
            path.write_text(yaml.safe_dump(document))
            self.runtime(harvest_params=str(path))
            self.assertFalse(any(action["package"] == "robot_end_effector" for action in self.actions))
            _, runtime = self.runtime(harvest_params=str(path), skip_strike="false")
            self.assertFalse(runtime["skip_strike"])
            self.assertTrue(any(action["package"] == "robot_end_effector" for action in self.actions))


if __name__ == "__main__":
    unittest.main()
