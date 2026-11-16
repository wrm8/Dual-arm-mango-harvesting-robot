from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from dual_arm_contract import group_targets, transform_point, validate_transform


class DualArmContractTests(unittest.TestCase):
    def test_separate_base_transforms_apply_translation(self):
        rotation = [0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1]
        left = rotation.copy()
        right = rotation.copy()
        left[3], right[3] = 0.1, -0.2
        self.assertEqual(transform_point(left, [0.2, 0, 0.5]), [0.6, -0.2, 0])
        self.assertEqual(transform_point(right, [0.2, 0, 0.5]), [0.3, -0.2, 0])

    def test_rejects_bad_transform(self):
        with self.assertRaises(ValueError):
            validate_transform([0.0] * 16)

    def test_groups_left_and_right_and_keeps_repeat_order(self):
        targets = [{'slot': 'right_bottom'}, {'slot': 'left_bottom'},
                   {'slot': 'left_top'}]
        result = group_targets(targets)
        self.assertEqual([t['slot'] for t in result['left']],
                         ['left_bottom', 'left_top'])
        self.assertEqual([t['slot'] for t in result['right']], ['right_bottom'])


if __name__ == '__main__':
    unittest.main()
