#!/usr/bin/env python3
"""Offline preview / self-check of the synthetic dart scene.

This does NOT need ROS and does not need OpenCV: it renders frames with the
same pure-numpy code the ROS simulator uses, writes PNG files, and prints a
table of projected pixel coordinates so you can see whether the camera
geometry makes sense before touching ROS.

Examples
--------
    python src/dart_sim/tools/preview.py                     # all frames into runs/preview
    python src/dart_sim/tools/preview.py --frames 0,10,20    # a few frames
    python src/dart_sim/tools/preview.py --montage 6         # also write a 3x2 contact sheet
    python src/dart_sim/tools/preview.py --draw-truth        # green rings = ground truth
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.normpath(os.path.dirname(HERE))
sys.path.insert(0, PKG_ROOT)

from dart_sim.render import render_frame, write_png  # noqa: E402
from dart_sim.scene import Scene, jsonable  # noqa: E402

DEFAULT_SCENE = os.path.join(PKG_ROOT, 'config', 'scene.yaml')


def parse_frames(spec, total):
    if spec in (None, '', 'all'):
        return list(range(total))
    indices = []
    for part in str(spec).split(','):
        part = part.strip()
        if not part:
            continue
        if '-' in part:
            first, last = part.split('-', 1)
            indices.extend(range(int(first), int(last) + 1))
        else:
            indices.append(int(part))
    return [i for i in indices if 0 <= i < total]


def make_montage(frames, cols=3, gap=6):
    height, width = frames[0].shape[:2]
    rows = (len(frames) + cols - 1) // cols
    canvas = np.full((rows * height + (rows - 1) * gap,
                      cols * width + (cols - 1) * gap, 3), 255, dtype=np.uint8)
    for index, frame in enumerate(frames):
        row, col = divmod(index, cols)
        y0 = row * (height + gap)
        x0 = col * (width + gap)
        canvas[y0:y0 + height, x0:x0 + width] = frame
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scene', default=DEFAULT_SCENE, help='scene YAML file')
    parser.add_argument('--out', default=os.path.join('runs', 'preview'),
                        help='output directory for PNG files')
    parser.add_argument('--frames', default='all', help="e.g. '0,10,20' or '0-20'")
    parser.add_argument('--stride', type=int, default=1, help='keep every Nth frame')
    parser.add_argument('--montage', type=int, default=0, help='contact sheet frame count')
    parser.add_argument('--draw-truth', action='store_true', help='draw green truth rings')
    args = parser.parse_args()

    scene = Scene.from_yaml(args.scene)
    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    camera = scene.camera
    print(f'scene        : {args.scene}')
    print(f'image        : {camera.width} x {camera.height} @ {scene.fps:g} fps')
    print(f'flight time  : {scene.t_end:.3f} s   frames: {scene.n_frames}')
    print(f'camera C     : {np.round(camera.C, 3)}')
    print(f'camera R_wc  :\n{np.round(camera.R_wc, 4)}')

    indices = parse_frames(args.frames, scene.n_frames)[::max(1, args.stride)]
    if not indices:
        print('no frames selected')
        return 1

    rng = np.random.default_rng(scene.seed)
    header = f'{"frame":>5} {"t(s)":>7} {"dart(u,v)":>20} {"depth":>7} {"lights":>7}'
    print('\n' + header)
    print('-' * len(header))

    selected = []
    for index in indices:
        t = index / scene.fps
        image = render_frame(scene, t, rng, draw_truth=args.draw_truth)
        write_png(os.path.join(out_dir, f'frame_{index:04d}.png'), image)
        selected.append(image)

        truth = scene.groundtruth(t)
        u, v = truth['dart_px']
        in_view = 0
        for (lu, lv), valid in zip(truth['light_px'], truth['light_valid']):
            if valid and 0 <= lu < camera.width and 0 <= lv < camera.height:
                in_view += 1
        print(f'{index:5d} {t:7.3f} ({u:7.1f},{v:7.1f}) {truth["dart_depth"]:7.2f} '
              f'{in_view}/4')

    with open(os.path.join(out_dir, 'scene_meta.json'), 'w', encoding='utf-8') as handle:
        json.dump(jsonable({
            'scene': os.path.abspath(args.scene),
            'width': camera.width,
            'height': camera.height,
            'K': camera.K,
            'D': camera.D,
            'R_wc': camera.R_wc,
            'C': camera.C,
            'lights': scene.lights,
            'flight_time': scene.t_end,
            'n_frames': scene.n_frames,
        }), handle, indent=2)

    if args.montage > 0:
        count = min(args.montage, len(selected))
        picks = np.linspace(0, len(selected) - 1, count).astype(int)
        montage = make_montage([selected[i] for i in picks])
        write_png(os.path.join(out_dir, 'montage.png'), montage)
        print(f'montage      : {count} frames -> {os.path.join(out_dir, "montage.png")}')

    print(f'wrote        : {len(selected)} frames -> {out_dir}')

    # quick visibility sanity check over the whole trajectory
    all_dart = []
    for t in np.linspace(0.0, scene.t_end, 200):
        uv, depth, valid = camera.project(scene.dart.position(t))
        if valid[0]:
            all_dart.append(uv[0])
    all_dart = np.asarray(all_dart)
    if all_dart.size:
        inside = ((all_dart[:, 0] >= 0) & (all_dart[:, 0] < camera.width)
                  & (all_dart[:, 1] >= 0) & (all_dart[:, 1] < camera.height))
        print(f'dart path    : u [{all_dart[:,0].min():.1f}, {all_dart[:,0].max():.1f}] '
              f'v [{all_dart[:,1].min():.1f}, {all_dart[:,1].max():.1f}] '
              f'in-view {inside.mean()*100:.0f}% of samples')
    return 0


if __name__ == '__main__':
    sys.exit(main())


