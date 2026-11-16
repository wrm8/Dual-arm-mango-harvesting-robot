"""Pure helpers for routing camera-frame harvest targets to two arm bases."""
import math


def validate_transform(values):
    if len(values) != 16 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError('camera transform must contain 16 finite values')
    m = [float(v) for v in values]
    if any(abs(a - b) > 1e-6 for a, b in zip(m[12:16], (0.0, 0.0, 0.0, 1.0))):
        raise ValueError('invalid homogeneous transform last row')
    r = [[m[row * 4 + col] for col in range(3)] for row in range(3)]
    for i in range(3):
        for j in range(3):
            dot = sum(r[k][i] * r[k][j] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > 1e-4:
                raise ValueError('camera transform rotation must be orthonormal')
    determinant = (
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
    if abs(determinant - 1.0) > 1e-4:
        raise ValueError('camera transform rotation must be proper')
    return m


def transform_point(matrix, point):
    if len(point) != 3 or not all(math.isfinite(float(v)) for v in point):
        raise ValueError('target point must contain 3 finite coordinates')
    m = validate_transform(matrix)
    x, y, z = (float(v) for v in point)
    return [m[0] * x + m[1] * y + m[2] * z + m[3],
            m[4] * x + m[5] * y + m[6] * z + m[7],
            m[8] * x + m[9] * y + m[10] * z + m[11]]


def side_for_slot(slot):
    if slot.startswith('left_'):
        return 'left'
    if slot.startswith('right_'):
        return 'right'
    raise ValueError(f'unknown harvest slot: {slot}')


def group_targets(targets):
    """Keep slot order within each arm; caller processes equal indexes in parallel."""
    grouped = {'left': [], 'right': []}
    for target in sorted(targets, key=lambda item: item['slot']):
        grouped[side_for_slot(target['slot'])].append(target)
    return grouped
