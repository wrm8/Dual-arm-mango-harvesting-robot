"""Check WSL training dependencies and forward/backward; no dataset required."""
import argparse
import json
import platform
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--require-cuda', action='store_true')
    args = parser.parse_args()
    import cv2
    import numpy as np
    import torch
    import torchvision
    import ultralytics
    from ultralytics import YOLO

    cuda = torch.cuda.is_available()
    if args.require_cuda and not cuda:
        raise SystemExit('CUDA is unavailable. Check WSL GPU driver support.')
    device = 'cuda:0' if cuda else 'cpu'
    torch.set_num_threads(2)
    report = {
        'python': platform.python_version(), 'platform': platform.platform(),
        'torch': torch.__version__, 'torchvision': torchvision.__version__,
        'ultralytics': ultralytics.__version__, 'opencv': cv2.__version__,
        'numpy': np.__version__, 'cuda_runtime': torch.version.cuda,
        'cuda_available': cuda, 'device': device,
        'gpu': torch.cuda.get_device_name(0) if cuda else None,
    }
    # Exercise CUDA convolution, backward and the torchvision NMS extension.
    layer = torch.nn.Conv2d(3, 8, 3).to(device)
    layer(torch.randn(2, 3, 64, 64, device=device)).square().mean().backward()
    boxes = torch.tensor([[0., 0., 20., 20.], [1., 1., 19., 19.]], device=device)
    keep = torchvision.ops.nms(boxes, torch.tensor([0.9, 0.8], device=device), 0.5)
    assert keep.tolist() == [0], 'Unexpected torchvision NMS output'
    # Build actual segmentation architecture locally; no weight/data download.
    model = YOLO('yolo11n-seg.yaml').model.to(device).train()
    outputs = model(torch.randn(2, 3, 64, 64, device=device))

    def tensors(value):
        if isinstance(value, torch.Tensor):
            yield value
        elif isinstance(value, dict):
            for item in value.values():
                yield from tensors(item)
        elif isinstance(value, (tuple, list)):
            for item in value:
                yield from tensors(item)

    values = list(tensors(outputs))
    if not values:
        raise RuntimeError('Segmentation model produced no tensors')
    loss = sum(value.float().square().mean() for value in values)
    if not torch.isfinite(loss):
        raise RuntimeError('Nonfinite model output')
    loss.backward()
    if not any(p.grad is not None for p in model.parameters()):
        raise RuntimeError('No model gradients')
    if cuda:
        torch.cuda.synchronize()
    report['checks'] = ['imports', 'convolution_backward', 'torchvision_nms',
                        'yolo11_segmentation_forward_backward']
    print(json.dumps(report, indent=2))
    destination = Path(__file__).resolve().parent / 'outputs'
    destination.mkdir(exist_ok=True)
    (destination / 'environment.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
