"""Camera perception helpers for the WRO mission node.

Owns:
  - /camera_info subscription (fills a PinholeCameraModel)
  - /limelight/detections subscription (Detection2DArray)
  - Pillar count for the "open vs obstacle" endgame decision
  - Pixel  ->  base_link projection using intrinsics + TF
  - Line / pillar queries at higher-level semantics

Requires a shared TF buffer from the owning node (the mission also uses it,
no reason to have two listeners).

Usage:
    self.cam = CameraPerception(self, self.tf_buffer)
    direction = self.cam.detect_direction()          # 'ccw' | 'cw' | None
    pillar = self.cam.closest_pillar_in_map(pose)     # (cls, xm, ym) | None
"""
import math
from typing import Optional, Tuple

from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transformer
from image_geometry import PinholeCameraModel


LINE_ORANGE = 'Orange_Line'
LINE_BLUE = 'Blue_Line'
LINE_CLASSES = (LINE_ORANGE, LINE_BLUE)

PILLAR_RED = 'Red_Pillar'
PILLAR_GREEN = 'Green_Pillar'
PILLAR_CLASSES = (PILLAR_RED, PILLAR_GREEN)


class CameraPerception:
    def __init__(
        self,
        node,
        tf_buffer: tf2_ros.Buffer,
        info_topic: str = '/camera_info',
        detections_topic: str = '/limelight/detections',
        pillar_confirm_count: int = 5,
        camera_optical_frame: str = 'camera_optical_frame',
    ):
        self.node = node
        self.tf_buffer = tf_buffer
        self.pillar_confirm_count = pillar_confirm_count
        self.camera_optical_frame = camera_optical_frame

        self.cam_model = PinholeCameraModel()
        self.have_info = False
        self.latest_detections = []

        self.pillar_detections_total = 0
        self.pillar_seen_ever = False
        self.line_detections_total = 0

        node.create_subscription(CameraInfo, info_topic, self._on_info, 10)
        node.create_subscription(
            Detection2DArray, detections_topic, self._on_detections, 10)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_info(self, msg: CameraInfo) -> None:
        self.cam_model.fromCameraInfo(msg)
        self.have_info = True

    def _on_detections(self, msg: Detection2DArray) -> None:
        self.latest_detections = msg.detections

        for det in msg.detections:
            if not det.results:
                continue
            cls = det.results[0].hypothesis.class_id
            if cls in LINE_CLASSES:
                self.line_detections_total += 1
            if not self.pillar_seen_ever and cls in PILLAR_CLASSES:
                self.pillar_detections_total += 1

        if (not self.pillar_seen_ever
                and self.pillar_detections_total >= self.pillar_confirm_count):
            self.pillar_seen_ever = True
            self.node.get_logger().info(
                f'pillars seen ({self.pillar_detections_total}) — '
                f'will park at end')

    # ------------------------------------------------------------------
    # High-level queries
    # ------------------------------------------------------------------
    def detect_direction(self) -> Optional[str]:
        """From the first line color seen: 'ccw' (orange) / 'cw' (blue).
        Returns None if no line is visible."""
        for det in self.latest_detections:
            if not det.results:
                continue
            cls = det.results[0].hypothesis.class_id
            if cls == LINE_ORANGE:
                return 'ccw'
            if cls == LINE_BLUE:
                return 'cw'
        return None

    def closest_pillar_in_map(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> Optional[Tuple[str, float, float]]:
        """Return (class, x_map, y_map) for the closest visible pillar,
        or None. robot_pose is (x, y, yaw) in the map frame — used only
        for the distance-to-robot comparison."""
        rx, ry, _ = robot_pose
        best = None
        best_dist = float('inf')

        for det in self.latest_detections:
            if not det.results:
                continue
            cls = det.results[0].hypothesis.class_id
            if cls not in PILLAR_CLASSES:
                continue
            body = self.pixel_to_body(
                det.bbox.center.position.x,
                det.bbox.center.position.y,
            )
            if body is None:
                continue
            world = self._body_to_map(*body)
            if world is None:
                continue
            d = math.hypot(world[0] - rx, world[1] - ry)
            if d < best_dist:
                best_dist, best = d, (cls, world[0], world[1])
        return best

    # ------------------------------------------------------------------
    # Frame conversions
    # ------------------------------------------------------------------
    def pixel_to_body(self, px: float, py: float) -> Optional[Tuple[float, float]]:
        """Project image pixel onto ground plane (z=0) in base_link frame."""
        if not self.have_info:
            return None
        ray = self.cam_model.projectPixelTo3dRay((px, py))

        far = PointStamped()
        far.header.frame_id = self.camera_optical_frame
        far.header.stamp = self.node.get_clock().now().to_msg()
        far.point.x = ray[0] * 10.0
        far.point.y = ray[1] * 10.0
        far.point.z = ray[2] * 10.0

        origin = PointStamped()
        origin.header = far.header

        try:
            far_body = self.tf_buffer.transform(
                far, 'base_link', Duration(seconds=0.1))
            origin_body = self.tf_buffer.transform(
                origin, 'base_link', Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return None

        ox, oy, oz = (origin_body.point.x, origin_body.point.y,
                      origin_body.point.z)
        fx, fy, fz = (far_body.point.x, far_body.point.y, far_body.point.z)
        dz = fz - oz
        if dz >= -1e-6:
            return None
        s = -oz / dz
        return ox + s * (fx - ox), oy + s * (fy - oy)

    def _body_to_map(
        self,
        x_body: float,
        y_body: float,
    ) -> Optional[Tuple[float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return None
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        return tx + c * x_body - s * y_body, ty + s * x_body + c * y_body
