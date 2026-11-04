import json
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT / 'yolo_mango')]
from decision import Fruit, decide
from harvest_observation import fruits_in_depth, selected_targets, tree_observation


class HarvestObservationTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT / 'yolo_mango/configs/vision.json').read_text())
        self.cfg.update(selection_mode='per_side', target_policy='green_or_small',
                        tie_relative_tolerance=0.05)
        self.fruits = [
            Fruit(1, 'green_mango', 120, 100, 100, .95, .5, surface_xyz_m=[0, 0, .5]),
            Fruit(2, 'green_mango', 130, 250, 70, .95, .5, surface_xyz_m=[.1, 0, .5]),
            Fruit(3, 'yellow_mango', 450, 100, 100, .95, .5, surface_xyz_m=[-.1, 0, .5]),
            Fruit(4, 'yellow_mango', 460, 250, 70, .95, .5, surface_xyz_m=[.2, 0, .5])]
        for f in self.fruits:
            f.width_px, f.width_m, f.height_m = 40, .04, .08

    def test_depth_gate_counts_only_in_range_not_other_tree_fruit(self):
        for depth in [.39, .61, float('nan'), None]:
            self.fruits[0].depth_m = depth
            self.assertEqual(len(fruits_in_depth(self.fruits, .4, .6)), 3)
        self.fruits[0].depth_m = .4
        self.fruits[1].depth_m = .6
        self.assertEqual(len(fruits_in_depth(self.fruits, .4, .6)), 4)

    def test_both_small_plus_green_deduplicates_and_transforms(self):
        result = decide(self.fruits, 640, self.cfg)
        matrix = np.array([[0,0,1,.1],[-1,0,0,0],[0,-1,0,.2],[0,0,0,1.]])
        targets = selected_targets(result, self.fruits, matrix, .75)
        self.assertEqual(len(targets), 3)
        small_green = next(t for t in targets if t['slot'] == 'left_bottom')
        self.assertTrue(small_green['is_small'] and small_green['is_green'])
        np.testing.assert_allclose(small_green['point'], [.6,-.1,.2])
        self.assertEqual(small_green['height_m'], .08)
        self.fruits[1].surface_xyz_m = None
        self.assertIsNone(selected_targets(result, self.fruits, matrix, .75))

    def test_equal_yellow_four_has_valid_empty_selection(self):
        for f in self.fruits:
            f.color = 'yellow_mango'; f.height_px = 100
        result = decide(self.fruits,640,self.cfg)
        self.assertEqual(selected_targets(result,self.fruits,np.eye(4),.75), [])

    def test_tree_image_error_needs_no_arm_calibration(self):
        trunk = dict(cx=420, cy=200, depth_m=.5, confidence=.95)
        tree = tree_observation([trunk], self.fruits, 640, .5, .4, .6)
        self.assertAlmostEqual(tree['error_ratio'], 100 / 640)
        self.assertEqual(tree['error_px'], 100)
        trunk['cx'] = 220
        self.assertAlmostEqual(tree_observation([trunk], [], 640, .5, .4, .6)['error_ratio'],
                               -100 / 640)
        trunk['cx'] = 320
        self.assertEqual(tree_observation([trunk], [], 640, .5, .4, .6)['error_ratio'], 0)

    def test_tree_uses_absolute_depth_gate_including_both_boundaries(self):
        trunk = dict(cx=320, cy=200, depth_m=.5, confidence=.95)
        for depth in [.4, .5, .6]:
            trunk['depth_m'] = depth
            self.assertIsNotNone(tree_observation([trunk], [], 640, .5, .4, .6))
        for depth in [.399, .601, 2, 0, None, float('nan'), float('inf')]:
            trunk['depth_m'] = depth
            self.assertIsNone(tree_observation([trunk], self.fruits, 640, .5, .4, .6))
        self.assertIsNone(tree_observation([], self.fruits, 640, .5, .4, .6))

    def test_out_of_range_trunk_cannot_hide_a_valid_trunk(self):
        trunks = [dict(cx=320, cy=200, depth_m=.7, confidence=.99),
                  dict(cx=420, cy=200, depth_m=.5, confidence=.9)]
        self.assertEqual(tree_observation(trunks, [], 640, .5, .4, .6)['error_px'], 100)

    def test_rotated_display_width_and_target_center_define_error(self):
        trunk = dict(cx=340, cy=200, depth_m=.5, confidence=.95)
        # A 640x480 frame rotated cw is displayed with width=480, not 640.
        self.assertAlmostEqual(
            tree_observation([trunk], [], 480, .5, .4, .6)['error_ratio'], 100 / 480)
        trunk['cx'] = 480 * .55
        self.assertEqual(tree_observation([trunk], [], 480, .55, .4, .6)['error_ratio'], 0)

    def test_nonfinite_trunk_geometry_cannot_be_reported_valid(self):
        trunk = dict(cx=320, cy=200, depth_m=.5, confidence=.95)
        for key in ('cx', 'confidence'):
            bad = dict(trunk, **{key: float('nan')})
            self.assertIsNone(tree_observation([bad], [], 640, .5, .4, .6))


if __name__ == '__main__':
    unittest.main()
