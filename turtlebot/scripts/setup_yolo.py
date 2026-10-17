#!/usr/bin/env python3
"""Install a private CPU YOLO runtime, model weights and licensed Gazebo assets."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import urllib.request
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
ASSETS = [
    ('Chair', 'Chair', 'CC BY 4.0', 'https://creativecommons.org/licenses/by/4.0/'),
    ('Standing_person', 'Standing%20person', 'CC0 1.0', 'https://creativecommons.org/publicdomain/zero/1.0/'),
]


def main():
    runtime = ROOT / '.runtime'
    venv = runtime / 'yolo-venv'
    subprocess.run([sys.executable, '-m', 'venv', '--system-site-packages', str(venv)], check=True)
    pip = [str(venv / 'bin/python'), '-m', 'pip', 'install']
    subprocess.run([*pip, '--index-url', 'https://download.pytorch.org/whl/cpu',
                    'torch==2.5.1+cpu', 'torchvision==0.20.1+cpu'], check=True)
    subprocess.run([*pip, 'numpy==1.26.4', 'opencv-python==4.10.0.84', 'ultralytics==8.3.253'], check=True)
    records = []
    for folder, name, license_name, license_url in ASSETS:
        dest = runtime / 'models' / folder
        dest.mkdir(parents=True, exist_ok=True)
        archive = dest / 'download.zip'
        url = f'https://fuel.gazebosim.org/1.0/OpenRobotics/models/{name}/3/{name}.zip'
        if not archive.exists():
            temporary = archive.with_suffix('.partial')
            urllib.request.urlretrieve(url, temporary)
            temporary.replace(archive)
        with ZipFile(archive) as zipped:
            if any(not (dest / item.filename).resolve().is_relative_to(dest.resolve())
                   for item in zipped.infolist()):
                raise RuntimeError('Unsafe asset archive path')
            zipped.extractall(dest)
        records.append({'model': folder, 'owner': 'OpenRobotics', 'source': url,
                        'license': license_name, 'license_url': license_url,
                        'sha256': hashlib.sha256(archive.read_bytes()).hexdigest()})
    weights = runtime / 'weights/yolo11n.pt'
    weights.parent.mkdir(parents=True, exist_ok=True)
    url = 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt'
    if not weights.exists():
        temporary = weights.with_suffix('.partial')
        urllib.request.urlretrieve(url, temporary)
        temporary.replace(weights)
    report = {'assets': records, 'weights': {'source': url,
              'sha256': hashlib.sha256(weights.read_bytes()).hexdigest()}}
    (runtime / 'yolo_setup.json').write_text(json.dumps(report, indent=2) + '\n')
    print('YOLO runtime and Gazebo assets ready.')


if __name__ == '__main__':
    main()
