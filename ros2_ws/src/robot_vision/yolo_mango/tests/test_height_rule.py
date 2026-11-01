import json
from pathlib import Path
import tempfile
import unittest
from decision import Fruit, decide
from stability import StabilityGate
from vision import save_observation

ROOT = Path(__file__).resolve().parents[1]


class HeightRuleTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT / 'configs/vision.json').read_text())
        self.fruits = [
            Fruit(0, 'green_mango', 150, 100, 100),
            Fruit(1, 'yellow_mango', 240, 300, 70),
            Fruit(2, 'green_mango', 410, 100, 120),
            Fruit(3, 'yellow_mango', 490, 300, 120),
        ]

    def decide(self):
        return decide(self.fruits, 640, self.cfg)

    def test_default_selects_only_shorter_left_fruit(self):
        result = self.decide()
        self.assertEqual(result['small_ids'], [1])
        self.assertEqual(result['small_id'], 1)
        self.assertEqual(result['small_by_side'], {'left': 1})
        self.assertEqual([t['id'] for t in result['targets']], [1])
        self.assertEqual(result['size_labels']['2'], 'equal')

    def test_right_shorter(self):
        self.fruits[1].height_px = 100
        self.fruits[3].height_px = 80
        self.assertEqual(self.decide()['small_id'], 3)

    def test_sides_are_independent_not_global_minimum(self):
        self.fruits[0].height_px = self.fruits[1].height_px = 50
        self.fruits[3].height_px = 90
        self.assertEqual(self.decide()['small_id'], 3)

    def test_equal_both_sides_means_confirmed_zero(self):
        self.fruits[1].height_px = 100
        result = self.decide()
        self.assertEqual(result['status'], 'confirmed')
        self.assertEqual(result['reason'], 'no_small_fruit')
        self.assertIsNone(result['small_id'])
        self.assertEqual(result['small_ids'], [])
        self.assertEqual(result['targets'], [])

    def test_both_different_never_returns_two(self):
        self.fruits[3].height_px = 60
        result = self.decide()
        self.assertEqual(result['reason'], 'multiple_small_candidates')
        self.assertEqual(result['status'], 'uncertain')
        self.assertEqual(result['small_ids'], [])
        self.assertEqual(result['small_by_side'], {})
        self.assertEqual(result['targets'], [])
        self.assertEqual(set(result['ratios']), {'left', 'right'})

    def test_small_difference_still_selects_shorter_by_default(self):
        self.fruits[1].height_px = 99.9
        self.assertEqual(self.decide()['small_id'], 1)

    def test_configurable_equal_tolerance(self):
        self.cfg['tie_relative_tolerance'] = 0.03
        self.fruits[3].height_px = 118
        self.assertEqual(self.decide()['small_id'], 1)
        self.fruits[1].height_px = 99
        self.assertEqual(self.decide()['small_ids'], [])

    def test_colors_do_not_change_size(self):
        self.fruits[1].color = 'green_mango'
        result = self.decide()
        self.assertEqual(result['small_id'], 1)
        self.assertEqual(result['targets'][0]['reasons'], ['small'])

    def test_height_not_width_area_or_vertical_position(self):
        # Decision input deliberately has no area or width; top/bottom can swap.
        self.fruits[0].cy, self.fruits[1].cy = 300, 100
        self.assertEqual(self.decide()['small_id'], 1)

    def test_depth_scaled_height_can_remove_apparent_difference(self):
        self.cfg['height_mode'] = 'depth_scaled'
        for fruit in self.fruits:
            fruit.depth_m = 1.0
        self.fruits[0].depth_m = 0.7
        result = self.decide()
        self.assertEqual(result['status'], 'confirmed')
        self.assertEqual(result['small_ids'], [])

    def test_order_does_not_change_winner(self):
        self.fruits.reverse()
        self.assertEqual(self.decide()['small_id'], 1)

    def test_duplicate_ids_rejected(self):
        self.fruits[0].id = 1
        self.assertEqual(self.decide()['reason'], 'duplicate_ids')

    def test_invalid_width_and_tolerance(self):
        with self.assertRaises(ValueError):
            decide(self.fruits, 0, self.cfg)
        self.cfg['tie_relative_tolerance'] = float('nan')
        with self.assertRaises(ValueError):
            self.decide()

    def test_ambiguous_frame_resets_stability(self):
        gate = StabilityGate(self.cfg)
        for _ in range(self.cfg['stable_frames']):
            stable = gate.update(self.decide(), self.fruits)
        self.assertTrue(stable)
        self.fruits[3].height_px = 80
        self.assertFalse(gate.update(self.decide(), self.fruits))
        self.assertEqual(gate.count, 0)

    def test_zero_targets_can_be_stable(self):
        self.fruits[1].height_px = 100
        gate = StabilityGate(self.cfg)
        for _ in range(self.cfg['stable_frames']):
            stable = gate.update(self.decide(), self.fruits)
        self.assertTrue(stable)

    def test_json_round_trip_has_single_small_and_color(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'nested/latest.json'
            save_observation(path, self.decide(), self.fruits, self.cfg, 1, False, False)
            data = json.loads(path.read_text())
            self.assertEqual(data['schema_version'], 2)
            self.assertEqual(data['decision']['small_id'], 1)
            self.assertEqual(data['fruits'][0]['color'], 'green_mango')
            self.assertFalse(data['target_surface_positions_available'])
            self.assertFalse(path.with_suffix('.tmp').exists())


if __name__ == '__main__':
    unittest.main()