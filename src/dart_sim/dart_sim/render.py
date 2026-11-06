"""Tiny pure-numpy renderer for the synthetic dart scene.

No OpenCV and no image libraries are required: glowing dots are drawn as
analytic soft disks, the dart strip as a soft line segment, and PNG files are
written with the standard library (zlib + struct).  That keeps the simulator
runnable inside a bare ROS 2 / WSL install and on plain Windows Python.
"""

from __future__ import annotations

import struct
import zlib

import numpy as np


# ---------------------------------------------------------------------------
# PNG output
# ---------------------------------------------------------------------------
def write_png(path, rgb):
    """Write an ``(H, W, 3)`` uint8 RGB array as a PNG file."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    height, width = rgb.shape[:2]

    raw = bytearray()
    for row in range(height):
        raw.append(0)  # filter type: none
        raw.extend(rgb[row].tobytes())

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))

    ihdr = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    with open(path, 'wb') as handle:
        handle.write(b'\x89PNG\r\n\x1a\n')
        handle.write(chunk(b'IHDR', ihdr))
        handle.write(chunk(b'IDAT', zlib.compress(bytes(raw), 6)))
        handle.write(chunk(b'IEND', b''))


# ---------------------------------------------------------------------------
# splatting primitives (additive light)
# ---------------------------------------------------------------------------
def _window(shape, cx, cy, pad):
    height, width = shape[:2]
    x0 = max(0, int(np.floor(cx - pad)))
    x1 = min(width - 1, int(np.ceil(cx + pad)))
    y0 = max(0, int(np.floor(cy - pad)))
    y1 = min(height - 1, int(np.ceil(cy + pad)))
    if x1 < x0 or y1 < y0:
        return None
    return x0, x1, y0, y1


def splat_disk(img, cx, cy, radius, gain, color=(1.0, 1.0, 1.0)):
    """Add a soft glowing disk centred at ``(cx, cy)`` with pixel radius."""
    radius = max(float(radius), 1.0)
    win = _window(img.shape, cx, cy, 3.0 * radius + 2.0)
    if win is None:
        return
    x0, x1, y0, y1 = win
    xs = np.arange(x0, x1 + 1, dtype=np.float32)
    ys = np.arange(y0, y1 + 1, dtype=np.float32)
    dx = xs[None, :] - cx
    dy = ys[:, None] - cy
    d2 = dx * dx + dy * dy
    rr = radius * radius

    core = np.clip(1.0 - d2 / rr, 0.0, 1.0) ** 0.5
    halo = 0.30 * np.exp(-d2 / (2.0 * rr))
    intensity = gain * (core + halo)
    img[y0:y1 + 1, x0:x1 + 1] += intensity[..., None] * np.asarray(color, dtype=np.float32)


def splat_line(img, a, b, half_width, gain, color=(1.0, 1.0, 1.0)):
    """Add a soft line segment from pixel ``a`` to pixel ``b``."""
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    half_width = max(float(half_width), 0.8)
    pad = 3.0 * half_width + 2.0
    win = _window(img.shape, 0.5 * (ax + bx), 0.5 * (ay + by),
                  pad + 0.5 * max(abs(bx - ax), abs(by - ay)))
    if win is None:
        return
    x0, x1, y0, y1 = win
    xs = np.arange(x0, x1 + 1, dtype=np.float32)[None, :]
    ys = np.arange(y0, y1 + 1, dtype=np.float32)[:, None]

    vx, vy = bx - ax, by - ay
    length2 = vx * vx + vy * vy
    if length2 < 1e-9:
        dist = np.hypot(xs - ax, ys - ay)
    else:
        tt = np.clip(((xs - ax) * vx + (ys - ay) * vy) / length2, 0.0, 1.0)
        dist = np.hypot(xs - (ax + tt * vx), ys - (ay + tt * vy))

    core = np.clip(1.0 - (dist / half_width) ** 2, 0.0, 1.0) ** 0.5
    halo = 0.25 * np.exp(-(dist / (2.0 * half_width)) ** 2)
    intensity = gain * (core + halo)
    img[y0:y1 + 1, x0:x1 + 1] += intensity[..., None] * np.asarray(color, dtype=np.float32)


def draw_ring(img, cx, cy, radius, color=(0, 255, 0), thickness=1):
    """Replace pixels of a thin ring (used for the ground-truth overlay)."""
    height, width = img.shape[:2]
    x0 = max(0, int(np.floor(cx - radius - 2)))
    x1 = min(width - 1, int(np.ceil(cx + radius + 2)))
    y0 = max(0, int(np.floor(cy - radius - 2)))
    y1 = min(height - 1, int(np.ceil(cy + radius + 2)))
    if x1 < x0 or y1 < y0:
        return
    xs = np.arange(x0, x1 + 1, dtype=np.float32)[None, :]
    ys = np.arange(y0, y1 + 1, dtype=np.float32)[:, None]
    dist = np.hypot(xs - cx, ys - cy)
    mask = np.abs(dist - radius) <= thickness
    img[y0:y1 + 1, x0:x1 + 1][mask] = color


# ---------------------------------------------------------------------------
# frame rendering
# ---------------------------------------------------------------------------
def render_frame(scene, t, rng, draw_truth=False):
    """Render one synthetic BGR-agnostic RGB frame at time ``t`` (seconds)."""
    cam = scene.camera
    height, width = cam.height, cam.width
    img = np.full((height, width, 3), scene.background, dtype=np.float32)
    img += rng.normal(0.0, scene.noise_sigma, (height, width, 1)).astype(np.float32)

    # a few dim blobs so the background is not perfectly empty
    for _ in range(max(0, scene.clutter)):
        u = rng.uniform(0.0, width)
        v = rng.uniform(0.0, height)
        splat_disk(img, u, v, rng.uniform(1.5, 4.0), rng.uniform(40.0, 90.0),
                   color=(0.55, 0.55, 0.6))

    # ground guide lights
    uv_lights, depth_lights, valid_lights = cam.project(scene.lights)
    for (u, v), depth, valid in zip(uv_lights, depth_lights, valid_lights):
        if not valid:
            continue
        radius = scene.light_radius * cam.fx / max(depth, 1e-3)
        splat_disk(img, u, v, radius, scene.light_gain, color=scene.light_color)

    # dart strip, motion-blurred over the exposure window
    samples = max(1, scene.motion_blur_samples)
    exposure_dt = scene.exposure_frac / scene.fps if scene.fps > 0 else 0.0
    if t >= 0.0:
        for k in range(samples):
            offset = ((k + 0.5) / samples - 0.5) * exposure_dt
            tail, nose = scene.dart.strip_endpoints(t + offset)
            uv_tail, depth_tail, valid_tail = cam.project(tail)
            uv_nose, depth_nose, valid_nose = cam.project(nose)
            if not (valid_tail[0] and valid_nose[0]):
                continue
            depth = 0.5 * (depth_tail[0] + depth_nose[0])
            half_width = 0.5 * scene.dart.strip_width * cam.fx / max(depth, 1e-3)
            splat_line(img, uv_tail[0], uv_nose[0], half_width,
                       scene.dart_gain / samples, color=scene.dart_color)

    if draw_truth:
        for (u, v), depth, valid in zip(uv_lights, depth_lights, valid_lights):
            if not valid:
                continue
            radius = scene.light_radius * cam.fx / max(depth, 1e-3)
            draw_ring(img, u, v, radius + 3.0, color=(0, 255, 0), thickness=1)
        uv_dart, depth_dart, valid_dart = cam.project(scene.dart.position(t))
        if valid_dart[0]:
            draw_ring(img, uv_dart[0, 0], uv_dart[0, 1], 6.0, color=(0, 255, 0), thickness=1)

    np.clip(img, 0.0, 255.0, out=img)
    return img.astype(np.uint8)
