import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from target_contract import TargetTracker, load_calibration


class TargetContractTests(unittest.TestCase):
    def setUp(self):
        self.tracker = TargetTracker()
        self.matrix = np.array([[0, 0, 1, .1], [-1, 0, 0, 0],
                                [0, -1, 0, .2], [0, 0, 0, 1.]])
        self.fruit = SimpleNamespace(id=2, confidence=.95, surface_xyz_m=[.1, .2, .6])
        self.result = {'status': 'confirmed', 'targets': [{'id': 2}],
                       'slots': {'left_top': 2}}

    def select(self, **changes):
        args = dict(result=self.result, fruits=[self.fruit], stable=True, matrix=self.matrix)
        args.update(changes)
        return self.tracker.select(**args)

    def test_converts_optical_axes_and_translation_in_meters(self):
        target = self.select()
        np.testing.assert_allclose(target['point'], [.7, -.1, 0])
        self.assertEqual(target['confidence'], .95)

    def test_no_valid_target_without_stability_calibration_depth_or_confidence(self):
        self.assertIsNone(self.select(stable=False))
        self.assertIsNone(self.select(matrix=None))
        self.fruit.surface_xyz_m = None
        self.assertIsNone(self.select())
        self.fruit.surface_xyz_m = [0, 0, float('nan')]
        self.assertIsNone(self.select())
        self.fruit.surface_xyz_m = [0, 0, .5]
        self.fruit.confidence = .6
        self.assertIsNone(self.select())

    def test_detection_reordering_does_not_change_track_id(self):
        first = self.select()['track_id']
        self.fruit.id = 9
        self.result['targets'][0]['id'] = 9
        self.result['slots']['left_top'] = 9
        self.assertEqual(self.select()['track_id'], first)
        self.fruit.surface_xyz_m[2] += .3
        self.assertNotEqual(self.select()['track_id'], first)

    def test_rejects_ambiguous_multiple_targets(self):
        self.result['targets'].append({'id': 7})
        self.assertIsNone(self.select())

    def test_rejects_unmeasured_wrong_frame_and_nonrigid_calibration(self):
        data = {'calibrated': True, 'target_frame': 'base_link',
                'source_frame': 'camera_color_optical_frame', 'camera_to_robot': self.matrix.tolist()}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'calibration.json'
            path.write_text(json.dumps(data))
            np.testing.assert_allclose(load_calibration(path), self.matrix)
            for key, value in [('calibrated', False), ('target_frame', 'camera'),
                               ('camera_to_robot', np.zeros((4, 4)).tolist())]:
                invalid = dict(data, **{key: value})
                path.write_text(json.dumps(invalid))
                with self.assertRaises(ValueError):
                    load_calibration(path)


if __name__ == '__main__':
    unittest.main()
