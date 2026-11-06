"""Scene model and projection math for the dart measurement simulation.

This module is ROS-free on purpose: importing it only needs numpy (plus
PyYAML for ``Scene.from_yaml``).  The simulator draws its synthetic image with
exactly the functions defined here, so the ground truth it publishes is
consistent with the projection model that the measurement code will later
invert with PnP / ray-plane intersection.

Frames
------
launch (world)
    Origin at the launcher.  +y is the horizontal launch direction, +z is up,
    +x completes a right-handed frame.  The dart is constrained to x = 0.
camera (OpenCV convention)
    +x right, +y down, +z forward along the optical axis.

A camera pose is stored as ``R_wc`` (columns are the camera x/y/z axes
expressed in the world frame) and ``C`` (camera centre in the world frame).
World -> camera: ``p_cam = R_wc.T @ (p_world - C)``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import os

import numpy as np


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------
def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Return ``(R_wc, C)`` for a camera at *eye* looking at *target*.

    ``R_wc`` columns are the camera +x (right), +y (down) and +z (forward)
    axes expressed in the world frame.
    """
    eye = np.asarray(eye, dtype=float)
    forward = np.asarray(target, dtype=float) - eye
    forward = forward / np.linalg.norm(forward)
    up = np.asarray(up, dtype=float)
    right = np.cross(forward, up)
    right = right / np.linalg.norm(right)
    down = np.cross(forward, right)
    return np.column_stack([right, down, forward]), eye


def distort_normalized(x, y, coeffs):
    """Apply the OpenCV plumb-bob (Brown-Conrady) distortion model."""
    k1, k2, p1, p2, k3 = np.asarray(coeffs, dtype=float).ravel()
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    return xd, yd


def rotation_to_quaternion(R):
    """Return ``(x, y, z, w)`` for a rotation matrix (ROS order)."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def jsonable(value):
    """Convert numpy containers into plain JSON-serialisable Python objects."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    return value


# ---------------------------------------------------------------------------
# camera
# ---------------------------------------------------------------------------
@dataclass
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: np.ndarray
    eye: np.ndarray
    target: np.ndarray
    up: np.ndarray
    R_wc: np.ndarray = field(init=False)
    C: np.ndarray = field(init=False)

    def __post_init__(self):
        self.distortion = np.asarray(self.distortion, dtype=float).ravel()
        self.eye = np.asarray(self.eye, dtype=float)
        self.target = np.asarray(self.target, dtype=float)
        self.up = np.asarray(self.up, dtype=float)
        self.R_wc, self.C = look_at(self.eye, self.target, self.up)

    @property
    def K(self):
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ])

    @property
    def D(self):
        return self.distortion.reshape(-1)

    def world_to_camera(self, points):
        points = np.atleast_2d(np.asarray(points, dtype=float))
        return (points - self.C) @ self.R_wc

    def camera_to_world(self, points):
        points = np.atleast_2d(np.asarray(points, dtype=float))
        return points @ self.R_wc.T + self.C

    def project(self, points):
        """Project world points.

        Returns ``(uv, depth, valid)`` where ``uv`` is ``(N, 2)`` pixels,
        ``depth`` is the camera-frame z and ``valid`` marks points in front of
        the camera.
        """
        cam = self.world_to_camera(points)
        depth = cam[:, 2]
        valid = depth > 1e-6
        safe = np.where(valid, depth, 1.0)
        x = cam[:, 0] / safe
        y = cam[:, 1] / safe
        xd, yd = distort_normalized(x, y, self.distortion)
        uv = np.stack([self.fx * xd + self.cx, self.fy * yd + self.cy], axis=1)
        return uv, depth, valid

    def project_one(self, point):
        uv, depth, valid = self.project(point)
        return uv[0], float(depth[0]), bool(valid[0])

    def pixel_to_ray(self, uv):
        """Back-project a pixel to a unit ray in the world frame."""
        uv = np.atleast_2d(np.asarray(uv, dtype=float))
        xd = (uv[:, 0] - self.cx) / self.fx
        yd = (uv[:, 1] - self.cy) / self.fy
        # Undistort by fixed-point iteration (good enough for small distortion).
        x, y = xd.copy(), yd.copy()
        for _ in range(10):
            xp, yp = distort_normalized(x, y, self.distortion)
            x += xd - xp
            y += yd - yp
        rays_cam = np.stack([x, y, np.ones_like(x)], axis=1)
        rays_cam /= np.linalg.norm(rays_cam, axis=1, keepdims=True)
        return rays_cam @ self.R_wc.T


# ---------------------------------------------------------------------------
# dart
# ---------------------------------------------------------------------------
@dataclass
class DartTrajectory:
    origin: np.ndarray
    speed: float
    elevation_deg: float
    gravity: float = 9.81
    strip_length: float = 0.15
    strip_width: float = 0.02

    def __post_init__(self):
        self.origin = np.asarray(self.origin, dtype=float)
        theta = np.deg2rad(self.elevation_deg)
        self.v0 = np.array([
            0.0,
            self.speed * np.cos(theta),
            self.speed * np.sin(theta),
        ])

    def position(self, t):
        t = np.asarray(t, dtype=float)
        out = self.origin + self.v0 * t[..., None]
        out[..., 2] -= 0.5 * self.gravity * t * t
        return out

    def velocity(self, t):
        t = np.asarray(t, dtype=float)
        out = np.broadcast_to(self.v0, t.shape + (3,)).copy()
        out[..., 2] -= self.gravity * t
        return out

    def flight_time(self, z_ground=0.0):
        a = -0.5 * self.gravity
        b = self.v0[2]
        c = self.origin[2] - z_ground
        disc = b * b - 4.0 * a * c
        return float((-b - np.sqrt(max(disc, 0.0))) / (2.0 * a))

    def strip_endpoints(self, t):
        """Return ``(tail, nose)`` world points of the glowing strip."""
        p = self.position(t)
        v = self.velocity(t)
        n = np.linalg.norm(v, axis=-1, keepdims=True)
        fallback = np.array([0.0, 1.0, 0.0])
        direction = np.where(n > 1e-9, v / np.maximum(n, 1e-9), fallback)
        half = 0.5 * self.strip_length
        return p - half * direction, p + half * direction


# ---------------------------------------------------------------------------
# full scene
# ---------------------------------------------------------------------------
@dataclass
class Scene:
    camera: CameraModel
    lights: np.ndarray
    light_radius: float
    dart: DartTrajectory
    fps: float
    background: float
    noise_sigma: float
    light_gain: float
    dart_gain: float
    exposure_frac: float
    motion_blur_samples: int
    clutter: int
    seed: int
    light_color: tuple
    dart_color: tuple

    @property
    def width(self):
        return self.camera.width

    @property
    def height(self):
        return self.camera.height

    @property
    def t_end(self):
        return self.dart.flight_time()

    @property
    def n_frames(self):
        return int(np.floor(self.t_end * self.fps)) + 1

    def frame_times(self):
        return np.arange(self.n_frames, dtype=float) / self.fps

    def groundtruth(self, t):
        """Everything the measurement code is not allowed to see."""
        cam = self.camera
        uv_lights, depth_lights, valid_lights = cam.project(self.lights)
        p = self.dart.position(t)
        v = self.dart.velocity(t)
        tail, nose = self.dart.strip_endpoints(t)
        uv_dart, depth_dart, valid_dart = cam.project(p)
        uv_tail, _, _ = cam.project(tail)
        uv_nose, _, _ = cam.project(nose)
        return {
            't': float(t),
            'dart': p,
            'velocity': v,
            'dart_px': uv_dart[0],
            'dart_depth': depth_dart[0],
            'dart_valid': valid_dart[0],
            'strip_tail_px': uv_tail[0],
            'strip_nose_px': uv_nose[0],
            'light_px': uv_lights,
            'light_depth': depth_lights,
            'light_valid': valid_lights,
        }

    @classmethod
    def from_yaml(cls, path):
        import yaml

        with open(path, 'r', encoding='utf-8') as handle:
            cfg = yaml.safe_load(handle)

        cam_cfg = cfg['camera']
        frame_cfg = cfg['frame']
        camera = CameraModel(
            width=int(frame_cfg['width']),
            height=int(frame_cfg['height']),
            fx=float(cam_cfg['fx']),
            fy=float(cam_cfg['fy']),
            cx=float(cam_cfg['cx']),
            cy=float(cam_cfg['cy']),
            distortion=np.asarray(cam_cfg.get('distortion', [0.0] * 5), dtype=float),
            eye=np.asarray(cam_cfg['eye'], dtype=float),
            target=np.asarray(cam_cfg['target'], dtype=float),
            up=np.asarray(cam_cfg.get('up', [0.0, 0.0, 1.0]), dtype=float),
        )

        dart_cfg = cfg['dart']
        dart = DartTrajectory(
            origin=np.asarray(dart_cfg['origin'], dtype=float),
            speed=float(dart_cfg['speed']),
            elevation_deg=float(dart_cfg['elevation_deg']),
            gravity=float(dart_cfg.get('gravity', 9.81)),
            strip_length=float(dart_cfg.get('strip_length', 0.15)),
            strip_width=float(dart_cfg.get('strip_width', 0.02)),
        )

        render_cfg = cfg.get('render', {})
        return cls(
            camera=camera,
            lights=np.asarray(cfg['lights']['points'], dtype=float),
            light_radius=float(cfg['lights'].get('radius', 0.06)),
            dart=dart,
            fps=float(frame_cfg.get('fps', 60.0)),
            background=float(render_cfg.get('background', 18.0)),
            noise_sigma=float(render_cfg.get('noise_sigma', 2.5)),
            light_gain=float(render_cfg.get('light_gain', 255.0)),
            dart_gain=float(render_cfg.get('dart_gain', 255.0)),
            exposure_frac=float(render_cfg.get('exposure_frac', 0.5)),
            motion_blur_samples=int(render_cfg.get('motion_blur_samples', 5)),
            clutter=int(render_cfg.get('clutter', 0)),
            seed=int(render_cfg.get('seed', 7)),
            light_color=tuple(float(v) for v in render_cfg.get(
                'light_color', [0.15, 1.0, 0.32])),
            dart_color=tuple(float(v) for v in render_cfg.get(
                'dart_color', [1.0, 0.95, 0.85])),
        )


def default_scene_path():
    """Path of the packaged default scene file."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, '..', 'config', 'scene.yaml')
