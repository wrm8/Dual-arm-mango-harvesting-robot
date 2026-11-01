import contextlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

try:
    import cv2
    import numpy as np
    import yaml
except ImportError:
    cv2 = None

import train
import evaluate
from check_dataset import validate
from dataset_config import resolve_data


@unittest.skipIf(cv2 is None, 'OpenCV/numpy/PyYAML required for workflow tests')
class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.data = self.root / 'dataset'
        for split in ('train', 'val'):
            images = self.data / 'images' / split
            labels = self.data / 'labels' / split
            images.mkdir(parents=True)
            labels.mkdir(parents=True)
            cv2.imwrite(str(images / 'one.jpg'), np.zeros((32, 32, 3), dtype=np.uint8))
            (labels / 'one.txt').write_text(
                '0 0.1 0.1 0.3 0.1 0.2 0.3\n'
                '1 0.4 0.4 0.6 0.4 0.5 0.6\n'
                '2 0.6 0.6 0.8 0.6 0.7 0.8\n')
        self.config = self.root / 'data.yaml'
        self.config.write_text(yaml.safe_dump({
            'path': str(self.data), 'train': 'images/train', 'val': 'images/val',
            'names': {0: 'green_mango', 1: 'tree_trunk', 2: 'yellow_mango'}}))
        self.weights = self.root / 'best.pt'
        self.weights.touch()  # Stub only, never load as real weights.

    def test_resolved_dataset_uses_absolute_path(self):
        with contextlib.redirect_stdout(io.StringIO()):
            path = resolve_data(self.config, self.root / 'out/resolved.yaml')
        data = yaml.safe_load(path.read_text())
        self.assertEqual(Path(data['path']), self.data)

    def test_corrupt_image_rejected(self):
        (self.data / 'images/train/one.jpg').write_bytes(b'not an image')
        _, errors = validate(self.data, check_images=True)
        self.assertTrue(any('cannot decode image' in error for error in errors))

    def test_training_passes_color_preserving_settings(self):
        model = Mock(task='segment')
        with patch.object(train, 'ROOT', self.root), patch.object(sys, 'argv', [
                'train.py', '--data', str(self.config), '--device', 'cpu']), patch.dict(
                    sys.modules, {'torch': SimpleNamespace(),
                                  'ultralytics': SimpleNamespace(YOLO=lambda _: model)}):
            with contextlib.redirect_stdout(io.StringIO()):
                train.main()
        kwargs = model.train.call_args.kwargs
        self.assertEqual(kwargs['hsv_h'], 0)
        self.assertEqual(kwargs['mosaic'], 0)
        self.assertEqual(kwargs['device'], 'cpu')
        self.assertTrue(Path(kwargs['data']).is_file())

    def test_check_only_does_not_import_model(self):
        with patch.object(train, 'ROOT', self.root), patch.object(sys, 'argv', [
                'train.py', '--data', str(self.config), '--check-only']), patch.dict(
                    sys.modules, {'ultralytics': None, 'torch': None}):
            with contextlib.redirect_stdout(io.StringIO()):
                train.main()

    def test_evaluation_writes_serializable_metrics(self):
        saved = self.root / 'evaluation'
        saved.mkdir()
        metrics = SimpleNamespace(save_dir=saved, results_dict={'metrics/mAP50(M)': 0.75},
                                  summary=lambda: [{'Class': 'yellow_mango',
                                                    'Images': np.int64(71),
                                                    'mAP50': np.float64(0.8)}])
        model = Mock(task='segment', names={0: 'green_mango', 1: 'tree_trunk',
                                           2: 'yellow_mango'})
        model.val.return_value = metrics
        with patch.object(evaluate, 'ROOT', self.root), patch.object(sys, 'argv', [
                'evaluate.py', '--data', str(self.config), '--weights', str(self.weights)]), patch.dict(
                    sys.modules, {'ultralytics': SimpleNamespace(YOLO=lambda _: model)}):
            with contextlib.redirect_stdout(io.StringIO()):
                evaluate.main()
        result = json.loads((saved / 'evaluation.json').read_text())
        self.assertEqual(result['metrics']['metrics/mAP50(M)'], 0.75)
        self.assertEqual(result['per_class'][0]['Class'], 'yellow_mango')
        self.assertEqual(result['per_class'][0]['Images'], 71)
        self.assertEqual(model.val.call_args.kwargs['split'], 'val')
