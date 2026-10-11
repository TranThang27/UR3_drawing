#!/usr/bin/env python3
"""Plot the actual saved occupancy image with a metric grid; keep map pixels unchanged."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.ticker import MultipleLocator
from matplotlib.transforms import Affine2D
import numpy as np
from PIL import Image
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    meta = yaml.safe_load(args.map.read_text())
    pixels = np.array(Image.open(args.map.parent / meta['image']).convert('L'))
    resolution = float(meta['resolution'])
    x, y, yaw = meta['origin']
    height, width = pixels.shape
    fig, ax = plt.subplots(figsize=(8, 8), constrained_layout=True)
    # PGM rows run top to bottom; the map's origin is its bottom-left corner.
    transform = Affine2D().rotate(yaw).translate(x, y)
    ax.imshow(pixels, cmap='gray', vmin=0, vmax=255, interpolation='nearest',
              origin='upper', extent=[0, width*resolution, 0, height*resolution],
              transform=transform + ax.transData)
    corners = transform.transform([[0, 0], [width*resolution, 0],
                                   [0, height*resolution], [width*resolution, height*resolution]])
    ax.set_xlim(corners[:, 0].min(), corners[:, 0].max())
    ax.set_ylim(corners[:, 1].min(), corners[:, 1].max())
    ax.set_aspect('equal')
    for axis in (ax.xaxis, ax.yaxis):
        axis.set_major_locator(MultipleLocator(1.0))
        axis.set_minor_locator(MultipleLocator(0.5))
    ax.grid(which='major', color='#3377aa', linewidth=0.6, alpha=0.5)
    ax.grid(which='minor', color='#3377aa', linewidth=0.3, alpha=0.25)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'TurtleBot 4 — SLAM Toolbox\n{resolution:g} m/pixel; grid 1 m')
    ax.legend(handles=[Patch(facecolor='black', label='Occupied'),
                       Patch(facecolor='white', edgecolor='black', label='Free'),
                       Patch(facecolor='#cdcdcd', label='Unknown')],
              loc='upper center', bbox_to_anchor=(0.5, -0.07), ncol=3)
    output = args.output or args.map.with_name(args.map.stem + '_grid.png')
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)
    print(output.resolve())


if __name__ == '__main__':
    main()
