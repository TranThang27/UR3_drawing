#!/usr/bin/env python3
"""Save and validate an occupancy map, retaining previous map files."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile

from PIL import Image
import yaml


def validate_map(yaml_path):
    metadata = yaml.safe_load(yaml_path.read_text())
    resolution = float(metadata['resolution'])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Map resolution không hợp lệ.')
    origin = metadata['origin']
    if len(origin) != 3 or not all(math.isfinite(float(value)) for value in origin):
        raise ValueError('Map origin không hợp lệ.')
    image_path = yaml_path.parent / metadata['image']
    with Image.open(image_path) as image:
        image.load()
        if image.format != 'PPM':
            raise ValueError('Map phải là ảnh PGM.')
        if not image_path.read_bytes().startswith((b'P5', b'P2')):
            raise ValueError('Ảnh map không có PGM header.')
        width, height = image.size
        histogram = image.convert('L').histogram()
    occupied = sum(histogram[:65])
    # map_saver trinary PGM uses 0 occupied, 254 free and 205 unknown.
    free = sum(histogram[250:])
    total = width * height
    if min(width, height) < 2 or not occupied or not free:
        raise ValueError('Map trống hoặc không có đủ ô trống/vật cản; chưa lưu bản đồ cuối.')
    return {
        'width_px': width, 'height_px': height, 'resolution_m': resolution,
        'free_area_m2': round(free * resolution ** 2, 2),
        'occupied_area_m2': round(occupied * resolution ** 2, 2),
        'unknown_pixels': total - free - occupied,
        'known_fraction_of_image': round((free + occupied) / total, 4),
        'note': 'Known fraction là tỷ lệ trong ảnh map, không phải tỷ lệ toàn môi trường đã khảo sát. '
                'Cần kiểm tra coverage, tường nhân đôi và sai lệch bằng RViz2.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path)
    args = parser.parse_args()
    prefix = args.prefix.expanduser().resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    # Save to a temporary folder: failed/blank saves never replace a good map.
    try:
        with tempfile.TemporaryDirectory(prefix='.map-save-', dir=prefix.parent) as folder:
            temporary = Path(folder) / prefix.name
            subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', str(temporary), '--fmt', 'pgm', '--mode', 'trinary',
                '--free', '0.196', '--occ', '0.65',
                '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=10.0',
            ], check=True, timeout=30)
            saved_yaml = temporary.with_name(temporary.name + '.yaml')
            saved_pgm = temporary.with_name(temporary.name + '.pgm')
            report = validate_map(saved_yaml)
            destinations = [prefix.with_name(prefix.name + suffix)
                            for suffix in ('.pgm', '.yaml', '_report.json')]
            previous = [path for path in destinations if path.exists()]
            if previous:
                backup = prefix.parent / 'backups' / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
                backup.mkdir(parents=True)
                for path in previous:
                    shutil.copy2(path, backup / path.name)
                print(f'Đã giữ bản đồ cũ tại {backup}')
            metadata = yaml.safe_load(saved_yaml.read_text())
            metadata['image'] = destinations[0].name
            saved_yaml.write_text(yaml.safe_dump(metadata, sort_keys=False))
            saved_pgm.replace(destinations[0])
            saved_yaml.replace(destinations[1])
            destinations[2].write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
            print(f'Đã lưu {destinations[1]} và {destinations[0]}')
            print(json.dumps(report, indent=2, ensure_ascii=False))
        return 0
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as error:
        print(f'Lưu map thất bại: {error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
