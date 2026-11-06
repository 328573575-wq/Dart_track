#!/usr/bin/env python3
"""sim_world: publish a synthetic camera stream plus ground truth.

What it does
------------
* renders the scene described by ``config/scene.yaml`` frame by frame,
* publishes the image on ``/camera/image_raw`` (bgr8) and camera intrinsics on
  ``/camera/camera_info``, exactly like a real camera driver would,
* publishes the hidden ground truth on ``/sim/truth/*`` so you can score your
  measurement code,
* optionally saves every frame as a PNG together with ``meta.json`` (static
  parameters) and ``truth.jsonl`` (one record per frame).

The saved folder is a complete, reproducible dataset: you can develop and
debug the PnP / triangulation code offline, then swap the image source for a
real camera without changing the algorithm.
"""

from __future__ import annotations

import array
import json
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray, String

from dart_sim.render import render_frame, write_png
from dart_sim.scene import Scene, jsonable, rotation_to_quaternion


class SimWorld(Node):
    def __init__(self):
        super().__init__('sim_world')

        default_scene = os.path.join(
            get_package_share_directory('dart_sim'), 'config', 'scene.yaml')
        self.declare_parameter('scene_file', default_scene)
        self.declare_parameter('save', False)
        self.declare_parameter('save_dir', '')
        self.declare_parameter('time_scale', 1.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('draw_truth', False)
        self.declare_parameter('debug_rate', False)
        self.declare_parameter('exit_when_done', False)

        scene_file = str(self.get_parameter('scene_file').value)
        self.scene = Scene.from_yaml(scene_file)
        self.time_scale = max(1e-3, float(self.get_parameter('time_scale').value))
        self.loop = bool(self.get_parameter('loop').value)
        self.draw_truth = bool(self.get_parameter('draw_truth').value)

        self.rng = np.random.default_rng(self.scene.seed)
        self.frame_index = 0
        self.debug_rate = bool(self.get_parameter('debug_rate').value)
        self.exit_when_done = bool(self.get_parameter('exit_when_done').value)
        self.requested_exit = False
        self._exit_timer = None
        self._last_perf = None

        self.pub_image = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.pub_dart = self.create_publisher(PointStamped, '/sim/truth/dart', 10)
        self.pub_velocity = self.create_publisher(Vector3Stamped, '/sim/truth/dart_velocity', 10)
        self.pub_dart_pixel = self.create_publisher(PointStamped, '/sim/truth/dart_pixel', 10)
        self.pub_camera_pose = self.create_publisher(PoseStamped, '/sim/truth/camera_pose', 10)
        self.pub_light_pixels = self.create_publisher(Float64MultiArray, '/sim/truth/light_pixels', 10)
        self.pub_light_points = self.create_publisher(Float64MultiArray, '/sim/truth/light_points', 10)
        self.pub_status = self.create_publisher(String, '/sim/status', 10)

        self.save = bool(self.get_parameter('save').value)
        self.out_dir = None
        self._truth_file = None
        if self.save:
            base = str(self.get_parameter('save_dir').value or '').strip()
            if not base:
                base = os.path.join(os.getcwd(), 'runs')
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.out_dir = os.path.join(base, stamp)
            os.makedirs(self.out_dir, exist_ok=True)
            self._write_meta(scene_file)
            self._truth_file = open(os.path.join(self.out_dir, 'truth.jsonl'),
                                    'w', encoding='utf-8')
            self.get_logger().info(f'saving dataset to {self.out_dir}')

        n = self.scene.n_frames
        self.get_logger().info(
            f'scene {self.scene.width}x{self.scene.height} @ {self.scene.fps:g} fps, '
            f'{n} frames, flight time {self.scene.t_end:.3f} s')
        self.get_logger().info(
            'topics: /camera/image_raw /camera/camera_info /sim/truth/* /sim/status')

        period = 1.0 / (self.scene.fps * self.time_scale)
        self.timer = self.create_timer(period, self._tick)

    # ------------------------------------------------------------------ tick
    def _tick(self):
        if self.debug_rate:
            now = time.perf_counter()
            if self._last_perf is not None:
                self.get_logger().info(f'tick dt {1000.0 * (now - self._last_perf):.1f} ms')
            self._last_perf = now
        index = self.frame_index
        t = index / self.scene.fps
        image = render_frame(self.scene, t, self.rng, draw_truth=self.draw_truth)
        stamp = self.get_clock().now().to_msg()

        self._publish_image(image, stamp)
        self._publish_camera_info(stamp)

        truth = self._groundtruth(t)
        self._publish_truth(truth, stamp)

        if self.save:
            self._save_frame(image, truth, index, t)

        self.pub_status.publish(String(
            data=f'frame {index + 1}/{self.scene.n_frames}  t={t:.3f}s'))

        self.frame_index += 1
        if self.frame_index >= self.scene.n_frames:
            if self.loop:
                self.frame_index = 0
            else:
                self.timer.cancel()
                self.get_logger().info('simulation finished')
                if self._truth_file is not None:
                    self._truth_file.flush()
                if self.exit_when_done:
                    # let the last published messages drain, then stop the loop
                    self._exit_timer = self.create_timer(0.2, self._request_exit)

    # --------------------------------------------------------------- publish
    def _publish_image(self, rgb, stamp):
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = 'camera_optical'
        message.height = int(rgb.shape[0])
        message.width = int(rgb.shape[1])
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = int(rgb.shape[1]) * 3
        # NOTE: assigning raw bytes hits the generated setter, which validates
        # every element in Python (~44 ms for 640x480x3).  Assigning an
        # ``array('B')`` is a memcpy fast path instead (~0.03 ms).
        message.data = array.array('B', rgb[:, :, ::-1].tobytes())
        self.pub_image.publish(message)

    def _publish_camera_info(self, stamp):
        camera = self.scene.camera
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = 'camera_optical'
        message.width = int(camera.width)
        message.height = int(camera.height)
        message.distortion_model = 'plumb_bob'
        message.d = [float(v) for v in camera.D]
        message.k = [float(v) for v in camera.K.reshape(-1)]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [float(camera.fx), 0.0, float(camera.cx), 0.0,
                     0.0, float(camera.fy), float(camera.cy), 0.0,
                     0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(message)

    def _groundtruth(self, t):
        camera = self.scene.camera
        uv_lights, depth_lights, valid_lights = camera.project(self.scene.lights)
        position = self.scene.dart.position(t)
        velocity = self.scene.dart.velocity(t)
        tail, nose = self.scene.dart.strip_endpoints(t)
        uv_dart, depth_dart, valid_dart = camera.project(position)
        uv_tail, _, _ = camera.project(tail)
        uv_nose, _, _ = camera.project(nose)
        return {
            't': float(t),
            'dart': position,
            'velocity': velocity,
            'dart_px': uv_dart[0],
            'dart_depth': float(depth_dart[0]),
            'dart_valid': bool(valid_dart[0]),
            'strip_tail_px': uv_tail[0],
            'strip_nose_px': uv_nose[0],
            'light_px': uv_lights,
            'light_depth': depth_lights,
            'light_valid': valid_lights,
        }

    def _publish_truth(self, truth, stamp):
        dart = PointStamped()
        dart.header.stamp = stamp
        dart.header.frame_id = 'launch'
        dart.point.x, dart.point.y, dart.point.z = [float(v) for v in truth['dart']]
        self.pub_dart.publish(dart)

        velocity = Vector3Stamped()
        velocity.header.stamp = stamp
        velocity.header.frame_id = 'launch'
        velocity.vector.x, velocity.vector.y, velocity.vector.z = [
            float(v) for v in truth['velocity']]
        self.pub_velocity.publish(velocity)

        pixel = PointStamped()
        pixel.header.stamp = stamp
        pixel.header.frame_id = 'camera_optical'
        pixel.point.x = float(truth['dart_px'][0])
        pixel.point.y = float(truth['dart_px'][1])
        pixel.point.z = float(truth['dart_depth'])
        self.pub_dart_pixel.publish(pixel)

        camera = self.scene.camera
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'launch'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = [
            float(v) for v in camera.C]
        qx, qy, qz, qw = rotation_to_quaternion(camera.R_wc)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        self.pub_camera_pose.publish(pose)

        light_pixels = Float64MultiArray()
        light_pixels.data = [float(v) for v in truth['light_px'].reshape(-1)]
        self.pub_light_pixels.publish(light_pixels)

        light_points = Float64MultiArray()
        light_points.data = [float(v) for v in self.scene.lights.reshape(-1)]
        self.pub_light_points.publish(light_points)

    # ------------------------------------------------------------------ save
    def _write_meta(self, scene_file):
        camera = self.scene.camera
        meta = {
            'scene_file': os.path.abspath(scene_file),
            'width': camera.width,
            'height': camera.height,
            'fps': self.scene.fps,
            'n_frames': self.scene.n_frames,
            'flight_time': self.scene.t_end,
            'K': camera.K,
            'D': camera.D,
            'R_wc': camera.R_wc,
            'C': camera.C,
            'lights': self.scene.lights,
            'light_radius': self.scene.light_radius,
            'dart': {
                'origin': self.scene.dart.origin,
                'speed': self.scene.dart.speed,
                'elevation_deg': self.scene.dart.elevation_deg,
                'gravity': self.scene.dart.gravity,
                'strip_length': self.scene.dart.strip_length,
                'strip_width': self.scene.dart.strip_width,
            },
            'render': {
                'background': self.scene.background,
                'noise_sigma': self.scene.noise_sigma,
                'exposure_frac': self.scene.exposure_frac,
                'motion_blur_samples': self.scene.motion_blur_samples,
            },
        }
        with open(os.path.join(self.out_dir, 'meta.json'), 'w', encoding='utf-8') as handle:
            json.dump(jsonable(meta), handle, indent=2)

    def _save_frame(self, image, truth, index, t):
        write_png(os.path.join(self.out_dir, f'frame_{index:04d}.png'), image)
        record = {
            'i': index,
            't': float(t),
            'dart': truth['dart'],
            'velocity': truth['velocity'],
            'dart_px': truth['dart_px'],
            'dart_depth': truth['dart_depth'],
            'dart_valid': truth['dart_valid'],
            'strip_tail_px': truth['strip_tail_px'],
            'strip_nose_px': truth['strip_nose_px'],
            'light_px': truth['light_px'],
            'light_depth': truth['light_depth'],
            'light_valid': truth['light_valid'],
        }
        self._truth_file.write(json.dumps(jsonable(record)) + '\n')
        self._truth_file.flush()

    def _request_exit(self):
        """Ask ``main`` to leave the spin loop.

        ``rclpy.shutdown()`` must not be called from inside a callback (it can
        deadlock with the executor), so only set a flag here.
        """
        if self.requested_exit:
            return
        self.requested_exit = True
        if getattr(self, '_exit_timer', None) is not None:
            self._exit_timer.cancel()
        self.get_logger().info('shutting down (exit_when_done)')

    def destroy_node(self):
        if self._truth_file is not None:
            try:
                self._truth_file.close()
            except OSError:
                pass
            self._truth_file = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimWorld()
    try:
        while rclpy.ok() and not node.requested_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

