"""Synthetic boxes/masks test integration, not recognition accuracy."""
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from decision import decide
from stability import StabilityGate
from vision import detect
from rgb_source import run_rgb
from frame_transform import rotate_image

ROOT = Path(__file__).resolve().parents[1]


class Tensor:
    def __init__(self, data):
        self.data = np.asarray(data)
    def cpu(self):
        return self
    def numpy(self):
        return self.data
    def tolist(self):
        return self.data.tolist()
    def item(self):
        return self.data.item()
    def __len__(self):
        return len(self.data)
    def __getitem__(self, item):
        return Tensor(self.data[item])


class FakeModel:
    names = {0: 'green_mango', 1: 'tree_trunk', 2: 'yellow_mango'}
    def predict(self, bgr, **kwargs):
        self.kwargs = kwargs
        # Small fruit is wider and has more area: the winner MUST use height.
        coords = [(60, 40, 80, 140), (160, 220, 280, 290),
                  (390, 40, 440, 160), (480, 220, 530, 340),
                  (300, 30, 340, 400)]
        masks, boxes = [], []
        for index, (x1, y1, x2, y2) in enumerate(coords):
            mask = np.zeros(bgr.shape[:2], dtype=np.float32)
            mask[y1:y2, x1:x2] = 1
            masks.append(mask)
            classes = [0, 2, 0, 2, 1]
            boxes.append(SimpleNamespace(xyxy=Tensor([[x1, y1, x2, y2]]),
                         cls=Tensor(classes[index]), conf=Tensor(0.95)))
        return [SimpleNamespace(masks=SimpleNamespace(data=Tensor(masks)),
                                boxes=boxes, plot=lambda: bgr.copy())]


@unittest.skipIf(np is None, 'OpenCV/numpy required for image integration tests')
class VisionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT / 'configs/vision.json').read_text())
        self.model = FakeModel()
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)

    def test_box_height_and_color_are_used_despite_opposite_area_order(self):
        fruits, _ = detect(self.model, self.image, self.cfg, 640, 'cpu')
        self.assertEqual([f.height_px for f in fruits], [100, 70, 120, 120])
        self.assertEqual(fruits[1].color, 'yellow_mango')
        self.assertNotIn('tree_trunk', [fruit.color for fruit in fruits])
        self.assertEqual(decide(fruits, 640, self.cfg)['small_id'], 1)
        self.assertTrue(self.model.kwargs['agnostic_nms'])

    def test_rotated_frames_are_contiguous_for_ultralytics_plotting(self):
        image = np.zeros((12, 8, 3), dtype=np.uint8)
        for rotation in ('cw', 'ccw'):
            rotated = rotate_image(image, rotation)
            self.assertTrue(rotated.flags['C_CONTIGUOUS'])

    def test_no_masks_returns_no_fruits(self):
        with patch.object(self.model, 'predict', return_value=[
                SimpleNamespace(masks=None, plot=lambda: self.image.copy())]):
            fruits, _ = detect(self.model, self.image, self.cfg, 640, 'cpu')
        self.assertEqual(fruits, [])

    def test_wrong_mask_shape_fails(self):
        with self.assertRaises(RuntimeError):
            self._wrong_mask()

    def _wrong_mask(self):
        prediction = self.model.predict(self.image)[0]
        prediction.masks.data = Tensor(np.ones((4, 10, 10)))
        with patch.object(self.model, 'predict', return_value=[prediction]):
            detect(self.model, self.image, self.cfg, 640, 'cpu')

    def args(self, directory, source):
        return SimpleNamespace(source=str(source), output=Path(directory),
                               imgsz=640, device='cpu', show=False, max_frames=0,
                               width=640, height=480, fps=10, save_video=False)

    def test_image_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'input.png'
            cv2.imwrite(str(source), self.image)
            run_rgb(self.model, self.args(directory, source), self.cfg, StabilityGate(self.cfg))
            data = json.loads((Path(directory) / 'latest.json').read_text())
            self.assertEqual(data['decision']['small_id'], 1)
            self.assertTrue((Path(directory) / 'annotated.jpg').exists())
            self.assertFalse(data['temporal_stable'])

    def test_video_history_and_end_invalidation(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'input.avi'
            writer = cv2.VideoWriter(str(source), cv2.VideoWriter_fourcc(*'MJPG'), 10, (640, 480))
            self.assertTrue(writer.isOpened())
            for _ in range(3):
                writer.write(self.image)
            writer.release()
            args = self.args(directory, source)
            run_rgb(self.model, args, self.cfg, StabilityGate(self.cfg))
            history = (Path(directory) / 'observations.jsonl').read_text().splitlines()
            self.assertEqual(len(history), 3)
            self.assertEqual(json.loads(history[0])['decision']['small_id'], 1)
            latest = json.loads((Path(directory) / 'latest.json').read_text())
            self.assertEqual(latest['decision']['reason'], 'end_of_video')
            self.assertEqual(latest['decision']['targets'], [])

    def test_unreadable_source_invalidates_and_releases(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.args(directory, Path(directory) / 'missing.avi')
            with self.assertRaises(RuntimeError):
                run_rgb(self.model, args, self.cfg, StabilityGate(self.cfg))
            data = json.loads((Path(directory) / 'latest.json').read_text())
            self.assertEqual(data['decision']['reason'], 'source_error')


if __name__ == '__main__':
    unittest.main()
