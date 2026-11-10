#!/usr/bin/env python3
"""sim_eval: score your measurement against the simulator's ground truth.

What it listens to
------------------
``/sim/truth/dart_front``, ``/sim/truth/dart_back``
    PointStamped, frame ``throw``: the two ends of the glowing strip.
``/sim/truth/camera_pose``
    PoseStamped, frame ``throw``: camera centre + orientation ``R_wc``.
``/dart_point``
    ``dart_point_interfaces/msg/DartPoint``: your position_node output.
``/tf``
    ``throw -> camera``: your solvepose_node output.

Pairing is done by ``header.stamp``, so each printed number compares the
measurement of one frame with the truth of *the same* frame: no "latest vs
latest" smearing.

Output
------
Once per ``report_period`` seconds (default 1 s):

    [  12.3 s] n=120 | dart  front 1.2 mm  back 2.0 mm | cam  pos 0.8 mm  rot 0.05 deg
                       max  3.1 mm      4.2 mm             1.9 mm      0.11 deg

Run it next to your launch::

    ros2 run dart_sim sim_eval

It needs ``dart_point_interfaces`` (your package); if that is missing it
prints a hint and exits, so dart_sim itself stays independent of the
measurement packages.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import Buffer, TransformListener

try:
    from dart_point_interfaces.msg import DartPoint
    HAVE_DART_POINT = True
except ImportError:  # pragma: no cover - depends on the workspace
    DartPoint = None
    HAVE_DART_POINT = False


def quat_to_matrix(x, y, z, w):
    """Quaternion (ROS xyzw) -> 3x3 rotation matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _fold_angle(deg):
    """把角度折到 [-90, 90]。

    飞镖在发射过程中与 +y 轴的夹角不可能超过 ±90°，所以超出这个范围只可能是
    "镖头镖尾判反"造成的 180° 偏差，折一下就能得到真实仰角。
    """
    deg = (deg + 180.0) % 360.0 - 180.0
    if deg > 90.0:
        deg -= 180.0
    elif deg < -90.0:
        deg += 180.0
    return deg


def rotation_angle_deg(R):
    """Angle of a rotation matrix, in degrees (robust near 0 and 180)."""
    cos = (np.trace(R) - 1.0) * 0.5
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


class SimEval(Node):
    def __init__(self, report_period=None):
        super().__init__('sim_eval')

        self.declare_parameter('report_period', 1.0)
        self.period = max(0.1, float(self.get_parameter('report_period').value))
        if report_period is not None:
            self.period = max(0.1, float(report_period))

        self.truth_front = {}
        self.truth_back = {}
        self.truth_cam = {}
        self.measurement = {}
        self.max_err = {'front': 0.0, 'back': 0.0, 'pos': 0.0, 'rot': 0.0, 'ang': 0.0}
        self.count = 0
        self.swapped = 0
        self.last_line = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PointStamped, '/sim/truth/dart_front', self._on_truth_front, 10)
        self.create_subscription(
            PointStamped, '/sim/truth/dart_back', self._on_truth_back, 10)
        self.create_subscription(
            PoseStamped, '/sim/truth/camera_pose', self._on_truth_cam, 10)
        self.create_subscription(DartPoint, '/dart_point', self._on_measurement, 10)

        self.create_timer(self.period, self._report)
        self.get_logger().info(
            'waiting for /dart_point + truth; errors are per-frame (matched by stamp)')

    # ------------------------------------------------------------- callbacks
    @staticmethod
    def _stamp_ns(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _on_truth_front(self, message):
        self.truth_front[self._stamp_ns(message)] = self._to_array(message.point)

    def _on_truth_back(self, message):
        self.truth_back[self._stamp_ns(message)] = self._to_array(message.point)

    def _on_truth_cam(self, message):
        pose = message.pose
        self.truth_cam[self._stamp_ns(message)] = (
            self._to_array(pose.position),
            quat_to_matrix(pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w),
        )

    def _on_measurement(self, message):
        stamp_ns = self._stamp_ns(message)
        self.measurement[stamp_ns] = message
        self._prune()

        front = self.truth_front.get(stamp_ns)
        back = self.truth_back.get(stamp_ns)
        if front is None or back is None:
            return  # truth for this frame not here yet

        measured_front = self._to_array(message.dart_real_front)
        measured_back = self._to_array(message.dart_real_back)
        err_front = float(np.linalg.norm(measured_front - front))
        err_back = float(np.linalg.norm(measured_back - back))
        # 如果"测得的头"更接近真值的尾、"测得的尾"更接近真值的头，就是头尾判反了。
        cross = (float(np.linalg.norm(measured_front - back))
                 + float(np.linalg.norm(measured_back - front)))
        if cross < err_front + err_back:
            self.swapped += 1

        # 角度误差：真值角度用同一帧的灯条两端算（和 position_node 里同一个公式），
        # 两边都折到 [-90, 90] 再比，这样头尾判反不会造成 180° 的假误差。
        truth_ang = math.degrees(math.atan2(front[2] - back[2], front[1] - back[1]))
        truth_ang = _fold_angle(truth_ang)
        err_ang = _fold_angle(message.dart_angle_degree - truth_ang)

        err_pos = None
        err_rot = None
        truth_cam = self.truth_cam.get(stamp_ns)
        if truth_cam is not None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'throw', 'camera', Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=0.05))
            except Exception:  # noqa: BLE001 - transform may not be there yet
                tf = None
            if tf is not None:
                pos = self._to_array(tf.transform.translation)
                q = tf.transform.rotation
                R = quat_to_matrix(q.x, q.y, q.z, q.w)
                err_pos = float(np.linalg.norm(pos - truth_cam[0]))
                err_rot = rotation_angle_deg(R.T @ truth_cam[1])

        self.count += 1
        self.max_err['front'] = max(self.max_err['front'], err_front)
        self.max_err['back'] = max(self.max_err['back'], err_back)
        self.max_err['ang'] = max(self.max_err['ang'], abs(err_ang))
        if err_pos is not None:
            self.max_err['pos'] = max(self.max_err['pos'], err_pos)
            self.max_err['rot'] = max(self.max_err['rot'], err_rot)

        self.last_line = (
            f'dart  front {1000.0 * err_front:7.2f} mm  back {1000.0 * err_back:7.2f} mm'
            f'  ang {err_ang:6.2f} deg'
            + ('' if err_pos is None else
               f' | cam  pos {1000.0 * err_pos:6.2f} mm  rot {err_rot:6.3f} deg'))

    def _report(self):
        if self.last_line is None:
            return
        max_line = (
            f'max   {1000.0 * self.max_err["front"]:7.2f} mm       '
            f'{1000.0 * self.max_err["back"]:7.2f} mm'
            f'       {self.max_err["ang"]:6.2f} deg'
            f' |      {1000.0 * self.max_err["pos"]:6.2f} mm       '
            f'{self.max_err["rot"]:6.3f} deg')
        self.get_logger().info(
            f'[score] n={self.count:5d} | {self.last_line}'
            + (f'  [head/tail swapped: {self.swapped}]' if self.swapped else ''))
        self.get_logger().info(f'[score]           {max_line}')

    # --------------------------------------------------------------- helpers
    @staticmethod
    def _to_array(vector):
        return np.array([vector.x, vector.y, vector.z], dtype=float)

    def _prune(self):
        """Keep only the most recent few hundred frames."""
        if len(self.measurement) <= 400:
            return
        for store in (self.measurement, self.truth_front, self.truth_back, self.truth_cam):
            for key in sorted(store)[:-200]:
                store.pop(key, None)


def main(args=None):
    if not HAVE_DART_POINT:
        print('sim_eval: dart_point_interfaces 不在环境里；'
              '先 source 你的工作空间（这个节点是用来给 dart_track 打分数的）。')
        return
    rclpy.init(args=args)
    node = SimEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
