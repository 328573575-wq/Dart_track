#!/usr/bin/env python3
"""Load a dataset produced by ``sim_world`` (``meta.json`` + ``truth.jsonl``).

The dataset is the bridge between "I have a simulation" and "I have a
measurement algorithm": it lets you iterate on PnP and triangulation offline,
with ground truth available for every frame.

Example
-------
    from tools.dataset import load_dataset
    meta, frames, truth = load_dataset('runs/20250101_120000')
    print(meta['K'], frames[0]['t'], frames[0]['dart_px'])
"""

from __future__ import annotations

import argparse
import json
import os

import numpy as np


def load_dataset(path):
    """Return ``(meta, frames, truth)``.

    ``meta`` is the static description (intrinsics, camera pose, light
    coordinates).  ``frames`` is a list of dicts with the image path and the
    ground truth for that frame; ``truth`` is the matching list of raw records.
    """
    meta_path = os.path.join(path, 'meta.json')
    truth_path = os.path.join(path, 'truth.jsonl')
    with open(meta_path, 'r', encoding='utf-8') as handle:
        meta = json.load(handle)
    truth = []
    with open(truth_path, 'r', encoding='utf-8') as handle:
        for line in handle:
            line = line.strip()
            if line:
                truth.append(json.loads(line))

    frames = []
    for record in truth:
        frames.append({
            'path': os.path.join(path, f'frame_{record["i"]:04d}.png'),
            't': record['t'],
            'dart_px': record['dart_px'],
            'dart': np.asarray(record['dart'], dtype=float),
            'velocity': np.asarray(record['velocity'], dtype=float),
            'light_px': np.asarray(record['light_px'], dtype=float).reshape(4, 2),
            'truth': record,
        })
    return meta, frames, truth


def main():
    parser = argparse.ArgumentParser(description='inspect a saved dart dataset')
    parser.add_argument('path', help='dataset folder containing meta.json')
    args = parser.parse_args()

    meta, frames, _ = load_dataset(args.path)
    camera = meta['K']
    print(f'dataset      : {os.path.abspath(args.path)}')
    print(f'frames       : {len(frames)}')
    print(f'image        : {meta["width"]} x {meta["height"]} @ {meta["fps"]:g} fps')
    print(f'flight time  : {meta["flight_time"]:.3f} s')
    print(f'K            : fx={camera[0][0]:.1f} fy={camera[1][1]:.1f} '
          f'cx={camera[0][2]:.1f} cy={camera[1][2]:.1f}')
    print(f'camera C     : {np.round(meta["C"], 3)}')
    print(f'lights (world):\n{np.round(np.asarray(meta["lights"]), 3)}')
    for i in (0, len(frames) // 2, len(frames) - 1):
        frame = frames[i]
        print(f'  frame {frame["truth"]["i"]:3d} t={frame["t"]:.3f}s '
              f'dart={np.round(frame["dart"], 2)} px={np.round(frame["dart_px"], 1)}')
    missing = [f['path'] for f in frames if not os.path.exists(f['path'])]
    print(f'missing png  : {len(missing)}')


if __name__ == '__main__':
    main()
