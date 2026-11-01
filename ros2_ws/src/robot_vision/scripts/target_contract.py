"""Translate stable model decisions into the existing base_link target contract."""
import json
import math
from pathlib import Path


def load_calibration(path, target_frame='base_link'):
    import numpy as np

    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('calibrated') is not True:
        raise ValueError('Calibration must be measured and marked calibrated=true')
    if data.get('target_frame') != target_frame:
        raise ValueError('Calibration target_frame must match target_frame parameter')
    if data.get('source_frame') != 'camera_color_optical_frame':
        raise ValueError('Calibration source_frame must be camera_color_optical_frame')
    matrix = np.asarray(data.get('camera_to_robot'), dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('camera_to_robot must contain 16 finite values')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError('Invalid homogeneous transform last row')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or not np.isclose(
            np.linalg.det(rotation), 1.0, atol=1e-4):
        raise ValueError('Calibration rotation must be a proper rigid rotation')
    return matrix


class TargetTracker:
    """Associate one selected target by slot and 3D distance, independent of box IDs."""
    def __init__(self, max_distance=0.08):
        self.max_distance = max_distance
        self.last_point = self.last_slot = None
        self.track_id = 0

    def select(self, result, fruits, stable, matrix, min_confidence=0.75):
        if not stable or matrix is None or result.get('status') != 'confirmed':
            return None
        targets = result.get('targets', [])
        if len(targets) != 1:
            return None
        fruit = next((f for f in fruits if f.id == targets[0]['id']), None)
        if (fruit is None or not math.isfinite(fruit.confidence) or
                not min_confidence <= fruit.confidence <= 1 or
                fruit.surface_xyz_m is None or len(fruit.surface_xyz_m) != 3 or
                not all(math.isfinite(v) for v in fruit.surface_xyz_m)):
            return None
        import numpy as np

        point = (matrix @ np.array([*fruit.surface_xyz_m, 1.0]))[:3].tolist()
        if not all(math.isfinite(v) for v in point):
            return None
        slot = next((s for s, i in result.get('slots', {}).items() if i == fruit.id), None)
        if (self.last_point is None or slot != self.last_slot or
                math.dist(point, self.last_point) > self.max_distance):
            self.track_id = self.track_id % 4294967295 + 1
        self.last_point, self.last_slot = point, slot
        return {'point': point, 'confidence': fruit.confidence, 'track_id': self.track_id}
