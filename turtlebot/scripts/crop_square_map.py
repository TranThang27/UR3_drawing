#!/usr/bin/env python3
"""Crop observed cells around the four measured room walls; retain the raw SLAM map.

Only for simple_square with mapping started at its origin. No occupancy cell is
painted or synthesized. Map coordinates are preserved by adjusting YAML origin.
"""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image
import yaml
from save_map import validate_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map', type=Path)
    args = parser.parse_args()
    path = args.map.resolve()
    meta = yaml.safe_load(path.read_text())
    if abs(meta['origin'][2]) > 1e-6:
        parser.error('Expected unrotated map origin for simple_square.')
    image_path = path.parent / meta['image']
    pixels = np.array(Image.open(image_path).convert('L'))
    height, width = pixels.shape
    res = float(meta['resolution'])
    ox, oy, _ = meta['origin']
    xs = ox + (np.arange(width) + 0.5)*res
    ys = oy + (height - np.arange(height) - 0.5)*res
    occupied = pixels < 65
    interior_x = np.abs(xs) < 2.7
    interior_y = np.abs(ys) < 2.7
    measured = {}

    def wall(axis, side):
        coordinates = xs if axis == 'x' else ys
        candidates = np.where(np.abs(coordinates-side*3.0) < 0.22)[0]
        if not len(candidates):
            raise ValueError('Map does not include the expected room wall.')
        scores = (occupied[interior_y][:, candidates].mean(axis=0) if axis == 'x'
                  else occupied[candidates][:, interior_x].mean(axis=1))
        best = int(candidates[np.argmax(scores)])
        continuity = float(scores.max())
        if continuity < 0.85:
            raise ValueError(f'Wall {axis}/{side} not sufficiently continuous: {continuity:.1%}')
        measured[f'{axis}_{side}'] = {'coordinate_m': float(coordinates[best]),
                                      'occupied_fraction': continuity}
        return best

    left, right = wall('x', -1), wall('x', 1)
    bottom, top = wall('y', -1), wall('y', 1)
    # Keep two cells beyond each measured face, including its observed thickness.
    col0, col1 = max(0, left-2), min(width, right+3)
    row0, row1 = max(0, top-2), min(height, bottom+3)
    cropped = pixels[row0:row1, col0:col1]
    raw_yaml = path.with_name(path.stem + '_raw.yaml')
    raw_pgm = path.with_name(path.stem + '_raw.pgm')
    shutil.copy2(image_path, raw_pgm)
    raw_meta = dict(meta, image=raw_pgm.name)
    raw_yaml.write_text(yaml.safe_dump(raw_meta, sort_keys=False))
    meta['origin'] = [ox + col0*res, oy + (height-row1)*res, 0.0]
    Image.fromarray(cropped).save(image_path, format='PPM')
    path.write_text(yaml.safe_dump(meta, sort_keys=False))
    report = validate_map(path)
    report['postprocess'] = {'operation': 'crop_only', 'raw_map': raw_yaml.name,
                            'measured_walls': measured,
                            'note': 'Retained original occupancy cells around measured room walls; '
                                    'removed artifacts outside the closed room.'}
    # Inspect the observed room interior; the opaque interiors of boxes may be unknown.
    room = pixels[top+1:bottom, left+1:right]
    report['known_fraction_room_interior'] = float(np.mean(room != 205))
    path.with_name(path.stem + '_report.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
