"""Shared dataset checks and path resolution for training and evaluation."""
from pathlib import Path
from check_dataset import validate

ROOT = Path(__file__).resolve().parent
NAMES = {0: 'green_mango', 1: 'tree_trunk', 2: 'yellow_mango'}


def resolve_data(config, output):
    import yaml
    data = yaml.safe_load(Path(config).read_text(encoding='utf-8-sig'))
    if not isinstance(data, dict) or data.get('names') != NAMES:
        raise ValueError('Expected classes 0=green_mango, 1=tree_trunk, 2=yellow_mango')
    if data.get('train') != 'images/train' or data.get('val') != 'images/val':
        raise ValueError('Expected images/train and images/val')
    root = Path(data['path'])
    root = root if root.is_absolute() else ROOT / root
    stats, errors = validate(root, check_images=True)
    print(stats)
    if errors:
        raise ValueError('\n'.join(errors))
    data['path'] = str(root.resolve())
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(data, allow_unicode=True), encoding='utf-8')
    return output
