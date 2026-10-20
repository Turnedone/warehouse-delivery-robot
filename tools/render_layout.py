#!/usr/bin/env python3
"""Renders the generated map, keep-out and speed masks, stations and obstacle paths to a PNG.

    python3 tools/render_layout.py docs/images/layout.png

Offscreen (matplotlib Agg backend), no window.
"""

import os
import sys

import matplotlib
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, 'src', 'wdr_bringup')


def load_pgm(name):
    with open(os.path.join(PKG, 'maps', name), 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        return np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)


def main(out):
    with open(os.path.join(PKG, 'maps', 'warehouse.yaml')) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(PKG, 'config', 'warehouse_layout.yaml')) as f:
        layout = yaml.safe_load(f)
    occ = load_pgm('warehouse.pgm')
    keepout = load_pgm('keepout_mask.pgm')
    speed = load_pgm('speed_mask.pgm')
    h, w = occ.shape
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    extent = [ox, ox + w * res, oy, oy + h * res]

    img = np.ones((h, w, 3))
    img[occ == 205] = (0.85, 0.85, 0.85)
    img[speed < 255] = (1.0, 0.93, 0.6)
    img[keepout == 0] = (1.0, 0.7, 0.7)
    img[occ == 0] = (0.15, 0.15, 0.15)

    fig, ax = plt.subplots(figsize=(12, 8.5), dpi=110)
    ax.imshow(img, extent=extent, origin='upper')
    for zone in layout['keepout_zones']:
        x0, y0, x1, y1 = zone['rect']
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec='red', lw=1.5))
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, 'keep-out', color='darkred', ha='center',
                va='center', fontsize=8)
    for zone in layout['speed_zones']:
        x0, y0, x1, y1 = zone['rect']
        ax.text((x0 + x1) / 2, y1 - 0.5, f"{zone['percent']} % speed", color='#8a6d00',
                ha='center', fontsize=8)
    colors = {'pickup': 'tab:blue', 'dropoff': 'tab:purple', 'exception': 'tab:orange'}
    for st in layout['stations']:
        x, y, _ = st['pose']
        ax.plot(x, y, 's', color=colors[st['type']], ms=8)
        ax.annotate(st['name'], (x, y), xytext=(4, -12), textcoords='offset points', fontsize=8)
    for dock in layout['docks']:
        x, y, _ = dock['pose']
        ax.plot(x, y, '*', color='tab:green', ms=14)
        ax.annotate(dock['name'], (x, y), xytext=(6, 6), textcoords='offset points', fontsize=8)
    for ob in layout['moving_obstacles']:
        xs, ys = zip(*ob['path'])
        ax.plot(xs, ys, '--', color='tab:red', lw=1)
        ax.annotate(ob['name'], (xs[0], ys[0]), xytext=(4, 4), textcoords='offset points',
                    fontsize=8, color='tab:red')
    ax.set_xlim(-0.5, 24.5)
    ax.set_ylim(-0.5, 16.5)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Generated warehouse: map (black), keep-out mask (red), slow zones (yellow), '
                 'stations, dock, moving obstacle paths', fontsize=9)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out)
    print('wrote', out)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, 'docs', 'images', 'layout.png'))
