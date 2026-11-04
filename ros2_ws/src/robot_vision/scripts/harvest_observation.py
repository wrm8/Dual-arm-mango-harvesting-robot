"""Depth gating, image-space tree centering and calibrated grasp snapshots."""
import math


def fruits_in_depth(fruits, minimum, maximum):
    return [f for f in fruits if f.depth_m is not None and
            math.isfinite(f.depth_m) and minimum <= f.depth_m <= maximum]


def tree_observation(trunks, fruits, width, center_ratio, minimum, maximum):
    """Center a depth-qualified trunk in the displayed (already rotated) image.

    Error is a fraction of image width, positive to the image right. No arm
    calibration is needed. Fruits only help associate a trunk: losing sight
    of the mango must not invalidate a visible in-range trunk during alignment.
    """
    cx = width * center_ratio
    mean_x = sum(f.cx for f in fruits) / len(fruits) if fruits else cx
    candidates = [t for t in trunks if t.get('depth_m') is not None and
                  math.isfinite(t['depth_m']) and minimum <= t['depth_m'] <= maximum and
                  math.isfinite(t['cx']) and 0 <= t['cx'] < width and
                  math.isfinite(t['confidence']) and 0 <= t['confidence'] <= 1]
    if not candidates:
        return None
    tree = min(candidates, key=lambda t: (abs(t['cx']-mean_x), -t['confidence']))
    return {'error_ratio': (tree['cx']-cx)/width, 'error_px': tree['cx']-cx,
            'confidence': tree['confidence'], 'depth_m': tree['depth_m']}


def selected_targets(result, fruits, matrix, min_confidence):
    """All selected slots, each once. Reject incomplete geometry as a whole."""
    import numpy as np

    if matrix is None or result.get('status') != 'confirmed':
        return None
    by_id = {f.id: f for f in fruits}
    slots = {fruit_id: slot for slot, fruit_id in result.get('slots', {}).items()}
    targets = []
    seen = set()
    for selected in result.get('targets', []):
        fid = selected['id']
        if fid in seen:
            continue
        seen.add(fid)
        f = by_id[fid]
        values = [f.confidence, f.depth_m, f.width_px, f.height_px, f.width_m, f.height_m]
        if (any(v is None or not math.isfinite(v) for v in values) or
                not min_confidence <= f.confidence <= 1 or
                f.surface_xyz_m is None or len(f.surface_xyz_m) != 3 or
                not all(math.isfinite(v) for v in f.surface_xyz_m)):
            return None
        point = (matrix @ np.array([*f.surface_xyz_m, 1.0]))[:3].tolist()
        if not all(math.isfinite(v) for v in point):
            return None
        slot = slots[fid]
        targets.append({'slot': slot, 'color': f.color,
                        'is_green': f.color == 'green_mango',
                        'is_small': 'small' in selected['reasons'],
                        'point': point, 'confidence': f.confidence,
                        'depth_m': f.depth_m, 'width_px': f.width_px,
                        'height_px': f.height_px, 'width_m': f.width_m,
                        'height_m': f.height_m})
    return sorted(targets, key=lambda target: target['slot'])
