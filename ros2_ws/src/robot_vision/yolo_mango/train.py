"""Train only color instance segmentation; size is a runtime geometric rule."""
import argparse
from pathlib import Path
from dataset_config import resolve_data

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, default=ROOT / 'configs/data.yaml')
    parser.add_argument('--model', default='yolo11n-seg.pt')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=4)
    parser.add_argument('--device', default='auto')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    output = ROOT / 'outputs'
    try:
        resolved = resolve_data(args.data, output / 'data.resolved.yaml')
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    if args.check_only:
        return
    import torch
    if args.device == 'auto':
        args.device = '0' if torch.cuda.is_available() else 'cpu'
    from ultralytics import YOLO
    model = YOLO(args.model)
    if model.task != 'segment':
        raise SystemExit('Use a segmentation model, e.g. yolo11n-seg.pt')
    model.train(data=str(resolved), epochs=args.epochs, imgsz=args.imgsz,
                batch=args.batch, device=args.device, workers=args.workers,
                project=str(ROOT / 'runs/segment'), name='mango',
                hsv_h=0.0, hsv_s=0.15, hsv_v=0.25, degrees=0.0,
                flipud=0.0, fliplr=0.5, mosaic=0.0, mixup=0.0,
                copy_paste=0.0, seed=42)


if __name__ == '__main__':
    main()
