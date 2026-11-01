"""Evaluate trained mango/tree-trunk segmentation weights on held-out images."""
import argparse
import json
from pathlib import Path
from dataset_config import ROOT, NAMES, resolve_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--data', type=Path, default=ROOT / 'configs/data.yaml')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=4)
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    if not args.weights.is_file():
        parser.error(f'Trained weights not found: {args.weights}')
    try:
        data = resolve_data(args.data, ROOT / 'outputs/data.eval.yaml')
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    from ultralytics import YOLO
    model = YOLO(str(args.weights))
    if model.task != 'segment' or model.names != NAMES:
        parser.error('Expected segmentation weights with the three configured classes')
    metrics = model.val(data=str(data), split='val', imgsz=args.imgsz,
                        batch=args.batch, device=args.device, workers=args.workers,
                        project=str(ROOT / 'runs/segment'), name='evaluation', plots=True)
    per_class = [
        {key: value.item() if hasattr(value, 'item') else value
         for key, value in row.items()}
        for row in metrics.summary()
    ]
    report = {
        'weights': str(args.weights.resolve()),
        'metrics': {key: float(value) for key, value in metrics.results_dict.items()},
        'per_class': per_class,
        'note': 'Color and segmentation metrics only; size rules need scene-level evaluation.'
    }
    path = Path(metrics.save_dir) / 'evaluation.json'
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False),
                    encoding='utf-8')
    print(f'Evaluation saved: {path}')


if __name__ == '__main__':
    main()
