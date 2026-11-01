"""Four-fruit geometry and color decisions, independent of ML and cameras."""
from dataclasses import dataclass
from math import isfinite, isclose


@dataclass
class Fruit:
    id: int
    color: str
    cx: float
    cy: float
    height_px: float
    confidence: float = 1.0
    depth_m: float | None = None
    range_m: float | None = None
    surface_xyz_m: list[float] | None = None


def validate_config(cfg):
    mode = cfg.get('selection_mode', 'per_side')
    if cfg.get('target_policy', 'small_only') not in ('small_only', 'green_or_small'):
        raise ValueError('target_policy must be small_only or green_or_small')
    if mode not in ('per_side', 'unique_small'):
        raise ValueError('selection_mode must be per_side or unique_small')
    if not 0 < cfg['small_ratio'] < cfg['similar_ratio'] <= 1:
        raise ValueError('Require 0 < small_ratio < similar_ratio <= 1')
    if cfg['height_mode'] not in ('pixel', 'depth_scaled'):
        raise ValueError('height_mode must be pixel or depth_scaled')
    if not 0 < cfg['center_x_ratio'] < 1 or not 0 <= cfg['center_deadband_ratio'] < 0.5:
        raise ValueError('Invalid center or deadband')
    if not 0 <= cfg['confidence'] <= 1:
        raise ValueError('confidence must be in [0, 1]')
    if not 0 <= cfg.get('tie_relative_tolerance', 0.0) < 1:
        raise ValueError('tie_relative_tolerance must be in [0, 1)')
    if not 0 < cfg['depth_min_m'] < cfg['depth_max_m'] or not isfinite(cfg['depth_max_m']):
        raise ValueError('Invalid depth range')


def decide(fruits, width, cfg):
    """Compare two fruits per side. Equal pairs are skipped; sides are independent."""
    validate_config(cfg)
    if not isfinite(width) or width <= 0:
        raise ValueError('width must be finite and positive')
    mode = cfg.get('selection_mode', 'per_side')
    result = {'status': 'uncertain', 'reason': '', 'selection_mode': mode,
              'ratios': {}, 'pairs': {}, 'small_id': None, 'small_ids': [],
              'small_by_side': {}, 'size_labels': {}, 'targets': [], 'slots': {}}

    def reject(reason):
        result['reason'] = reason
        return result

    if len(fruits) != 4:
        return reject('expected_four_fruits')
    if len({f.id for f in fruits}) != 4:
        return reject('duplicate_ids')
    groups = {'left': [], 'right': []}
    center = width * cfg['center_x_ratio']
    band = width * cfg['center_deadband_ratio']
    for f in fruits:
        if (f.color not in ('yellow_mango', 'green_mango') or
                not all(isfinite(v) for v in (f.cx, f.cy, f.height_px, f.confidence)) or
                not 0 <= f.cx < width or f.cy < 0 or
                f.height_px <= 0 or not cfg['confidence'] <= f.confidence <= 1):
            return reject('invalid_or_low_confidence_detection')
        if abs(f.cx - center) <= band:
            return reject('fruit_near_center_line')
        groups['left' if f.cx < center else 'right'].append(f)
    if any(len(group) != 2 for group in groups.values()):
        return reject('expected_two_per_side')

    candidates = {}
    ambiguous = None
    for side, group in groups.items():
        # Horizontal tie break makes slot order deterministic for level fruit.
        for vertical, f in zip(('top', 'bottom'), sorted(group, key=lambda f: (f.cy, f.cx))):
            result['slots'][f'{side}_{vertical}'] = f.id
        heights = []
        for f in group:
            height = f.height_px
            if cfg['height_mode'] == 'depth_scaled':
                if (f.depth_m is None or not isfinite(f.depth_m) or
                        not cfg['depth_min_m'] <= f.depth_m <= cfg['depth_max_m']):
                    return reject('missing_or_invalid_depth')
                height *= f.depth_m
            heights.append((height, f.id))
        heights.sort()
        ratio = heights[0][0] / heights[1][0]
        result['ratios'][side] = ratio
        result['pairs'][side] = {
            'ids': [item[1] for item in heights],
            'measured_heights': [item[0] for item in heights],
            'ratio': ratio,
        }
        if mode == 'per_side':
            if isclose(ratio, 1.0, rel_tol=0, abs_tol=1e-12) or (
                    1.0 - ratio <= cfg.get('tie_relative_tolerance', 0.0)):
                result['pairs'][side]['comparison'] = 'equal'
            else:
                candidates[side] = heights[0][1]
                result['pairs'][side]['comparison'] = 'different'
        elif ratio <= cfg['small_ratio']:
            candidates[side] = heights[0][1]
        elif ratio < cfg['similar_ratio']:
            ambiguous = 'height_ratio_in_uncertain_band'
    if ambiguous:
        return reject(ambiguous)
    if mode == 'per_side' and len(candidates) > 1:
        return reject('multiple_small_candidates')
    if mode == 'unique_small' and len(candidates) != 1:
        return reject('expected_exactly_one_small_candidate')
    result['small_by_side'] = candidates
    result['small_ids'] = list(candidates.values())
    if len(candidates) == 1:
        result['small_id'] = result['small_ids'][0]
    for f in fruits:
        small = f.id in result['small_ids']
        side = 'left' if f.cx < center else 'right'
        result['size_labels'][str(f.id)] = (
            'small' if small else ('large' if side in candidates else 'equal'))

        reasons = []
        if f.color == 'green_mango' and cfg.get('target_policy', 'small_only') == 'green_or_small':
            reasons.append('green')
        if small:
            reasons.append('small')
        if reasons:
            result['targets'].append({'id': f.id, 'reasons': reasons,
                                      'surface_xyz_m': f.surface_xyz_m})
    result.update(status='confirmed', reason=(
        ('one_small_fruit' if candidates else 'no_small_fruit') if mode == 'per_side'
        else 'one_different_pair_one_similar_pair'))
    return result
