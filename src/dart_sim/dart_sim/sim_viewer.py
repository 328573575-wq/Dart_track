#!/usr/bin/env python3
"""sim_viewer: show the synthetic camera stream with ground-truth overlays.

Requires OpenCV (``python3-opencv``).  If OpenCV is missing the node says so
once and keeps running silently, so the rest of the system is unaffected.
Set ``show_truth:=false`` to look at the raw image exactly as a real camera
would deliver it.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

try:
    import cv2
except ImportError:  # pragma: no cover - depends on the machine
    cv2 = None


class SimViewer(Node):
    def __init__(self):
        super().__init__('sim_viewer')
        self.declare_parameter('window', 'dart_sim')
        self.declare_parameter('show_truth', True)
        self.window = str(self.get_parameter('window').value)
        self.show_truth = bool(self.get_parameter('show_truth').value)

        self.light_pixels = None
        self.dart_pixel = None
        self.dart_depth = float('nan')
        self._gui_error = False
        self.requested_exit = False

        if cv2 is None:
            self.get_logger().error(
                'OpenCV (cv2) is not installed; install python3-opencv to use the viewer')
        else:
            self.get_logger().info('viewer ready; press q in the image window to quit')

        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)
        self.create_subscription(Float64MultiArray, '/sim/truth/light_pixels',
                                 self._on_lights, 10)
        self.create_subscription(PointStamped, '/sim/truth/dart_pixel',
                                 self._on_dart, 10)

    def _on_lights(self, message):
        data = np.asarray(message.data, dtype=float)
        if data.size >= 8:
            self.light_pixels = data[:8].reshape(4, 2)

    def _on_dart(self, message):
        self.dart_pixel = (message.point.x, message.point.y)
        self.dart_depth = message.point.z

    def _on_image(self, message):
        if cv2 is None:
            return
        try:
            buffer = np.frombuffer(message.data, dtype=np.uint8)
            if message.encoding == 'bgr8':
                frame = buffer.reshape(message.height, message.width, 3)
            elif message.encoding == 'rgb8':
                frame = buffer.reshape(message.height, message.width, 3)[:, :, ::-1]
            else:
                self.get_logger().warn(
                    f'unsupported encoding {message.encoding!r}', throttle_duration_sec=5.0)
                return
            frame = np.ascontiguousarray(frame)

            if self.show_truth and self.light_pixels is not None:
                for u, v in self.light_pixels:
                    cv2.circle(frame, (int(round(u)), int(round(v))), 8,
                               (0, 255, 0), 1, cv2.LINE_AA)
            if self.show_truth and self.dart_pixel is not None:
                u, v = self.dart_pixel
                cv2.drawMarker(frame, (int(round(u)), int(round(v))),
                               (0, 0, 255), cv2.MARKER_CROSS, 14, 1)
                cv2.putText(frame, f'dart {u:6.1f},{v:6.1f}  z={self.dart_depth:5.2f} m',
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1,
                            cv2.LINE_AA)

            cv2.imshow(self.window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                self.get_logger().info('quit requested')
                self.requested_exit = True
        except cv2.error as exc:  # typically: no display available
            if not self._gui_error:
                self._gui_error = True
                self.get_logger().error(
                    f'cannot open a GUI window ({exc}); run with gui:=false or use an X server')

    def destroy_node(self):
        if cv2 is not None and not self._gui_error:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimViewer()
    try:
        while rclpy.ok() and not node.requested_exit:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
