"""Run after sourcing ROS: python3 -m unittest discover -s test -v."""
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

import yaml

_spec = importlib.util.spec_from_file_location(
    "real_launch", Path(__file__).resolve().parents[1] / "launch" / "real.launch.py"
)
_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_launch)


class CalibratedIkLimits(unittest.TestCase):
    def params(self):
        return {
            "joint_names": [f"joint{i}" for i in range(1, 6)] + ["gripper"],
            "zero_ticks": [1908, 2012, 1640, 1491, 518, 760],
            "urdf_zero_ticks": [],
            "ticks_per_radian": [651.8986] * 4 + [238.7324] * 2,
            "directions": [1, 1, -1, 1, 1, 1],
            "min_ticks": [1470, 1430, 1640, 840, 135, 195],
            "max_ticks": [2380, 3500, 3650, 3640, 890, 760],
        }

    def derive(self, params):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "driver.yaml"
            path.write_text(yaml.safe_dump({"robot_arm_driver": {"ros__parameters": params}}))
            return _launch._calibrated_ik_limits(str(path))

    def test_native_mapping_and_reversed_joint(self):
        lower, upper = self.derive(self.params())
        self.assertEqual(len(lower), 5)
        self.assertAlmostEqual(lower[2], -2010 / 651.8986)
        self.assertEqual(upper[2], 0)
        self.assertAlmostEqual(lower[0], -438 / 651.8986)
        self.assertAlmostEqual(upper[0], 472 / 651.8986)

    def test_model_zero_takes_precedence_and_joint_order_is_named(self):
        params = self.params()
        params["urdf_zero_ticks"] = [1887, 2384, 3602, 1889, 518, 760]
        lower, upper = self.derive(params)
        self.assertAlmostEqual(lower[2], -48 / 651.8986)
        self.assertAlmostEqual(upper[2], 1962 / 651.8986)
        for values in params.values():
            values.reverse()
        self.assertEqual((lower, upper), self.derive(params))

    def test_bad_calibration_cannot_silently_disable_limits(self):
        for key, value in (("directions", [0] * 6), ("zero_ticks", [1]),
                ("ticks_per_radian", [math.nan] * 6), ("urdf_zero_ticks", [1]),
                ("joint_names", ["joint1"] * 6), ("min_ticks", [4000] * 6)):
            with self.subTest(key=key):
                params = self.params()
                params[key] = value
                with self.assertRaises(ValueError):
                    self.derive(params)


if __name__ == "__main__":
    unittest.main()
