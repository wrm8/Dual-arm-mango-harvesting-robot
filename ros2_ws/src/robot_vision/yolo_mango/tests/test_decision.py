import json
from pathlib import Path
import tempfile
import unittest

from decision import Fruit, decide
from stability import StabilityGate
from check_dataset import validate

ROOT = Path(__file__).resolve().parents[1]


class DecisionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT / 'configs/vision.json').read_text())
        self.cfg['selection_mode'] = 'unique_small'
        self.cfg['target_policy'] = 'green_or_small'
        self.fruits = [
            Fruit(0, 'yellow_mango', 180, 100, 100),
            Fruit(1, 'yellow_mango', 220, 300, 70),
            Fruit(2, 'green_mango', 450, 100, 100),
            Fruit(3, 'yellow_mango', 420, 300, 97),
        ]

    def run_decision(self):
        return decide(self.fruits, 640, self.cfg)

    def test_yellow_small_and_green_big(self):
        result = self.run_decision()
        self.assertEqual(result['status'], 'confirmed')
        self.assertEqual(result['small_id'], 1)
        self.assertEqual({t['id'] for t in result['targets']}, {1, 2})

    def test_small_on_right(self):
        self.fruits[1].height_px = 98
        self.fruits[3].height_px = 70
        self.assertEqual(self.run_decision()['small_id'], 3)

    def test_green_small_not_duplicated(self):
        self.fruits[1].color = 'green_mango'
        self.fruits[2].color = 'yellow_mango'
        result = self.run_decision()
        self.assertEqual(len(result['targets']), 1)
        self.assertEqual(result['targets'][0]['reasons'], ['green', 'small'])

    def test_missing_fruit(self):
        self.fruits.pop()
        self.assertEqual(self.run_decision()['status'], 'uncertain')
        self.assertEqual(self.run_decision()['targets'], [])

    def test_extra_detection(self):
        self.fruits.append(Fruit(4, 'green_mango', 500, 200, 80))
        self.assertEqual(self.run_decision()['status'], 'uncertain')

    def test_both_pairs_different(self):
        self.fruits[3].height_px = 60
        self.assertEqual(self.run_decision()['status'], 'uncertain')

    def test_both_pairs_similar(self):
        self.fruits[1].height_px = 97
        self.assertEqual(self.run_decision()['status'], 'uncertain')

    def test_gray_band(self):
        self.fruits[1].height_px = 87
        self.assertEqual(self.run_decision()['reason'], 'height_ratio_in_uncertain_band')

    def test_center_line(self):
        self.fruits[0].cx = 320
        self.assertEqual(self.run_decision()['reason'], 'fruit_near_center_line')

    def test_wrong_side_count(self):
        self.fruits[0].cx = 500
        self.assertEqual(self.run_decision()['reason'], 'expected_two_per_side')

    def test_thresholds_invalid(self):
        self.cfg['small_ratio'] = 0.99
        with self.assertRaises(ValueError):
            self.run_decision()

    def test_nonfinite_height(self):
        self.fruits[1].height_px = float('nan')
        self.assertEqual(self.run_decision()['status'], 'uncertain')

    def test_low_confidence(self):
        self.fruits[0].confidence = 0.2
        self.assertEqual(self.run_decision()['status'], 'uncertain')

    def test_depth_corrects_false_small(self):
        # Left apparent ratio is 0.70, but physical scaled heights are equal.
        self.fruits[0].depth_m = 0.7
        self.fruits[1].depth_m = 1.0
        self.fruits[2].depth_m = 1.0
        self.fruits[3].depth_m = 1.0
        self.cfg['height_mode'] = 'depth_scaled'
        self.assertEqual(self.run_decision()['reason'], 'expected_exactly_one_small_candidate')

    def test_missing_depth_has_no_fallback(self):
        self.cfg['height_mode'] = 'depth_scaled'
        self.assertEqual(self.run_decision()['reason'], 'missing_or_invalid_depth')

    def test_stable_and_reset(self):
        self.cfg['stable_frames'] = 3
        gate = StabilityGate(self.cfg)
        result = self.run_decision()
        self.assertFalse(gate.update(result, self.fruits))
        self.assertFalse(gate.update(result, self.fruits))
        self.assertTrue(gate.update(result, self.fruits))
        self.fruits[0].cx += 20
        self.assertFalse(gate.update(self.run_decision(), self.fruits))
        self.assertEqual(gate.count, 1)
        self.fruits.pop()
        self.assertFalse(gate.update(self.run_decision(), self.fruits))
        self.assertEqual(gate.count, 0)

    def test_detector_id_reordering_preserves_stability(self):
        self.cfg['stable_frames'] = 2
        gate = StabilityGate(self.cfg)
        self.assertFalse(gate.update(self.run_decision(), self.fruits))
        for f in self.fruits:
            f.id += 10
        self.assertTrue(gate.update(self.run_decision(), self.fruits))

    def test_color_change_resets_stability(self):
        self.cfg['stable_frames'] = 2
        gate = StabilityGate(self.cfg)
        gate.update(self.run_decision(), self.fruits)
        self.fruits[0].color = 'green_mango'
        self.assertFalse(gate.update(self.run_decision(), self.fruits))

    def test_depth_jump_resets_stability(self):
        self.cfg['stable_frames'] = 2
        for f in self.fruits:
            f.depth_m = 1.0
        gate = StabilityGate(self.cfg)
        gate.update(self.run_decision(), self.fruits)
        self.fruits[1].depth_m = 1.1
        self.assertFalse(gate.update(self.run_decision(), self.fruits))


class DatasetTests(unittest.TestCase):
    def test_empty_dataset_reports_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            _, errors = validate(directory)
            self.assertEqual(len(errors), 2)

    def test_polygon_dataset_and_box_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for split in ('train', 'val'):
                (root / 'images' / split).mkdir(parents=True)
                (root / 'labels' / split).mkdir(parents=True)
                (root / 'images' / split / 'one.jpg').touch()
                (root / 'labels' / split / 'one.txt').write_text(
                    '0 0.1 0.1 0.3 0.1 0.2 0.3\n'
                    '1 0.4 0.4 0.6 0.4 0.5 0.6\n'
                    '2 0.6 0.6 0.8 0.6 0.7 0.8\n')
            _, errors = validate(root)
            self.assertFalse(errors)
            (root / 'labels/train/one.txt').write_text('0 0.5 0.5 0.2 0.2\n')
            _, errors = validate(root)
            self.assertTrue(any('polygon' in e for e in errors))


if __name__ == '__main__':
    unittest.main()
