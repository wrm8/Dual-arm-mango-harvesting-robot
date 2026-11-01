"""Exercise height rules with synthetic detections, without weights or hardware."""
import json
from pathlib import Path
from decision import Fruit, decide
from vision import save_observation

ROOT = Path(__file__).resolve().parent


def main():
    cfg = json.loads((ROOT / 'configs/vision.json').read_text())
    fruits = [
        Fruit(0, 'green_mango', 160, 150, 100),
        Fruit(1, 'yellow_mango', 230, 310, 70),
        Fruit(2, 'yellow_mango', 430, 150, 110),
        Fruit(3, 'green_mango', 490, 310, 110),
    ]
    result = decide(fruits, 640, cfg)
    path = ROOT / 'outputs/demo.json'
    save_observation(path, result, fruits, cfg, 1, False, False)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f'Synthetic detections only; this does not test color recognition. Saved: {path}')


if __name__ == '__main__':
    main()