"""Validate YOLO polygon labels without loading ML dependencies."""
import argparse
import math
from pathlib import Path

IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


def validate(root, check_images=False):
    root = Path(root)
    errors = []
    stats = {}
    for split in ('train', 'val'):
        images_dir = root / 'images' / split
        labels_dir = root / 'labels' / split
        images = sorted(p for p in images_dir.glob('*') if p.suffix.lower() in IMAGE_EXTENSIONS)
        if not images:
            errors.append(f'{images_dir}: no images')
        stems = [p.stem for p in images]
        if len(stems) != len(set(stems)):
            errors.append(f'{split}: duplicate image stems')
        counts = {0: 0, 1: 0, 2: 0}
        stem_set = set(stems)
        for image in images:
            if check_images:
                import cv2
                import numpy as np
                try:
                    decoded = cv2.imdecode(np.fromfile(str(image), dtype=np.uint8), cv2.IMREAD_COLOR)
                    if decoded is None:
                        errors.append(f'{image}: cannot decode image')
                except (OSError, cv2.error) as exc:
                    errors.append(f'{image}: cannot decode image: {exc}')
            label = labels_dir / (image.stem + '.txt')
            if not label.exists():
                errors.append(f'{label}: missing label (use empty file for negative image)')
                continue
            for line_num, line in enumerate(label.read_text(encoding='utf-8-sig').splitlines(), 1):
                if not line.strip():
                    continue
                try:
                    tokens = line.split()
                    cls = int(tokens[0])
                    coords = [float(v) for v in tokens[1:]]
                    if cls not in counts:
                        raise ValueError('class must be 0, 1, or 2')
                    if len(coords) < 6 or len(coords) % 2:
                        raise ValueError('need at least 3 polygon vertices; detection boxes are not valid')
                    if any(not math.isfinite(v) or not 0 <= v <= 1 for v in coords):
                        raise ValueError('coordinates must be finite and normalized to [0,1]')
                    pts = list(zip(coords[::2], coords[1::2]))
                    area = abs(sum(x * pts[(i+1) % len(pts)][1] -
                                   y * pts[(i+1) % len(pts)][0]
                                   for i, (x, y) in enumerate(pts)))
                    if area <= 1e-10:
                        raise ValueError('zero-area polygon')
                    counts[cls] += 1
                except (ValueError, IndexError) as exc:
                    errors.append(f'{label}:{line_num}: {exc}')
        for label in labels_dir.glob('*.txt'):
            if label.stem not in stem_set:
                errors.append(f'{label}: no matching image')
        stats[split] = {
            'images': len(images), 'green_mango': counts[0],
            'tree_trunk': counts[1], 'yellow_mango': counts[2]
        }
        if images and not all(counts.values()):
            errors.append(f'{split}: all three classes must be represented')
    return stats, errors


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default=str(Path(__file__).parent / 'datasets/mango'))
    parser.add_argument('--check-images', action='store_true', help='Also decode images using OpenCV')
    args = parser.parse_args()
    stats, errors = validate(args.root, check_images=args.check_images)
    print(stats)
    for error in errors:
        print(error)
    raise SystemExit(1 if errors else 0)
