#!/usr/bin/env python3
"""Add physical, textured Fuel models near the walls of the existing map world."""
import argparse
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OBJECTS = [
    ('demo_chair_west', 'Chair', [-2.45, .35, 0, 0, 0, 0]),
    ('demo_chair_east', 'Chair', [2.4, .35, 0, 0, 0, 3.14159]),
    ('demo_person', 'Standing_person', [1.95, 2.5, 0, 0, 0, 3.14159]),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'reports/demo_objects')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name, asset, pose in OBJECTS:
        folder = ROOT / '.runtime/models' / asset
        tree = ET.parse(folder / 'model.sdf')
        model = tree.getroot().find('model')
        model.set('name', name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, pose))
        for uri in model.iter('uri'):
            if uri.text.startswith('model://'):
                uri.text = str(folder / uri.text.split('/', 3)[-1])
        path = args.output.resolve() / f'{name}.sdf'
        tree.write(path, encoding='unicode')
        request = f'sdf_filename: "{path}" allow_renaming: false'
        result = subprocess.run(['ign', 'service', '-s', '/world/simple_square/create',
            '--reqtype', 'ignition.msgs.EntityFactory', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '10000', '--req', request], capture_output=True, text=True, timeout=15)
        if result.returncode or 'data: true' not in result.stdout:
            raise RuntimeError(f'Failed to create {name}: {result.stdout} {result.stderr}')
        print('Added', name, flush=True)
    (args.output / 'objects.json').write_text(json.dumps(OBJECTS, indent=2) + '\n')


if __name__ == '__main__':
    main()
