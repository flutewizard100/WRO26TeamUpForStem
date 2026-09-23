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
from typing import List, Optional, Tuple

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

WALL = 'Wall'

# Landmark accumulator: sightings within this radius (map frame) are
# merged into the same landmark. WRO pillars are 5x5 cm, so 0.15 m gives
# generous slack for projection / localization noise.
LANDMARK_MERGE_RADIUS_M = 0.15
LANDMARK_MIN_SIGHTINGS = 3

# Walls: same accumulator, but merges use a smaller radius (walls are
# thin). Confirmation requires several sightings so a single-frame false
# positive (shadow, dark texture, projection glitch) can't leak into
# /camera_obstacles.
WALL_MERGE_RADIUS_M = 0.05
WALL_MIN_SIGHTINGS = 5


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

        # Persistent pillar landmarks in the map frame. Each entry:
        # {'cls': str, 'x': float, 'y': float, 'n': int (sighting count)}
        self.pillar_landmarks: List[dict] = []
        # Same schema for wall landmarks; class_id is always 'Wall'.
        self.wall_landmarks: List[dict] = []

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
            if cls in PILLAR_CLASSES:
                if not self.pillar_seen_ever:
                    self.pillar_detections_total += 1
                self._integrate_pillar_landmark(det, cls)
            if cls == WALL:
                self._integrate_wall_landmark(det)

        if (not self.pillar_seen_ever
                and self.pillar_detections_total >= self.pillar_confirm_count):
            self.pillar_seen_ever = True
            self.node.get_logger().info(
                f'pillars seen ({self.pillar_detections_total}) — '
                f'will park at end')

    # ------------------------------------------------------------------
    # High-level queries
    # ------------------------------------------------------------------
    def detect_direction(
        self,
    ) -> Optional[Tuple[Tuple[str, float, float],
                         Optional[Tuple[str, float, float]]]]:
        """Return the two nearest visible line detections, projected to
        the map frame, as
            ((cls, x_map, y_map),                # closest
             (cls, x_map, y_map) | None)         # next-closest, or None
        Returns None if no line was detected in the current frame or if
        the TF projection to map failed for every detection."""
        closest = None
        closest_dist = float('inf')
        next_closest = None
        next_closest_dist = float('inf')

        for det in self.latest_detections:
            if not det.results:
                continue
            cls = det.results[0].hypothesis.class_id
            if cls not in LINE_CLASSES:
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
            d = math.hypot(body[0], body[1])       # dist from robot to line
            if d < closest_dist:
                next_closest_dist, next_closest = closest_dist, closest
                closest_dist, closest = d, (cls, world[0], world[1])
            elif d < next_closest_dist:
                next_closest_dist, next_closest = d, (cls, world[0], world[1])

        if closest is None:
            return None
        return closest, next_closest


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
    # Pillar landmarks (accumulated over time in the map frame)
    # ------------------------------------------------------------------
    def _integrate_pillar_landmark(self, det, cls: str) -> None:
        """Project one pillar detection into map frame and merge it into
        the persistent landmark list. No-op if TF isn't ready."""
        body = self.pixel_to_body(
            det.bbox.center.position.x,
            det.bbox.center.position.y,
        )
        if body is None:
            return
        world = self._body_to_map(*body)
        if world is None:
            return
        x, y = world

        best_idx: Optional[int] = None
        best_d = LANDMARK_MERGE_RADIUS_M
        for i, lm in enumerate(self.pillar_landmarks):
            if lm['cls'] != cls:
                continue
            d = math.hypot(lm['x'] - x, lm['y'] - y)
            if d < best_d:
                best_d, best_idx = d, i

        if best_idx is None:
            self.pillar_landmarks.append(
                {'cls': cls, 'x': x, 'y': y, 'n': 1})
        else:
            lm = self.pillar_landmarks[best_idx]
            n = lm['n']
            lm['x'] = (lm['x'] * n + x) / (n + 1)
            lm['y'] = (lm['y'] * n + y) / (n + 1)
            lm['n'] = n + 1

    def confirmed_pillar_landmarks(
        self,
        min_sightings: int = LANDMARK_MIN_SIGHTINGS,
    ) -> List[Tuple[str, float, float]]:
        """Landmarks seen at least `min_sightings` times, as
        (class, x_map, y_map)."""
        return [(lm['cls'], lm['x'], lm['y'])
                for lm in self.pillar_landmarks
                if lm['n'] >= min_sightings]

    def closest_pillar_landmark(
        self,
        robot_pose: Tuple[float, float, float],
        min_sightings: int = LANDMARK_MIN_SIGHTINGS,
    ) -> Optional[Tuple[str, float, float]]:
        """Closest confirmed landmark to the robot. Prefers the landmark
        list over `closest_pillar_in_map` because it survives frames
        where the pillar drops out of view."""
        rx, ry, _ = robot_pose
        best = None
        best_d = float('inf')
        for cls, x, y in self.confirmed_pillar_landmarks(min_sightings):
            d = math.hypot(x - rx, y - ry)
            if d < best_d:
                best_d, best = d, (cls, x, y)
        return best

    # ------------------------------------------------------------------
    # Wall landmarks (accumulated over time in the map frame)
    # ------------------------------------------------------------------
    def _integrate_wall_landmark(self, det) -> None:
        """Project the bbox's bottom-center (wall base = where the wall
        meets the floor) into the map frame and merge into the wall
        landmark list. No-op if TF isn't ready."""
        bx = det.bbox.center.position.x
        by = det.bbox.center.position.y + det.bbox.size_y / 2.0
        body = self.pixel_to_body(bx, by)
        if body is None:
            return
        world = self._body_to_map(*body)
        if world is None:
            return
        x, y = world

        best_idx: Optional[int] = None
        best_d = WALL_MERGE_RADIUS_M
        for i, lm in enumerate(self.wall_landmarks):
            d = math.hypot(lm['x'] - x, lm['y'] - y)
            if d < best_d:
                best_d, best_idx = d, i

        if best_idx is None:
            self.wall_landmarks.append(
                {'cls': WALL, 'x': x, 'y': y, 'n': 1})
        else:
            lm = self.wall_landmarks[best_idx]
            n = lm['n']
            lm['x'] = (lm['x'] * n + x) / (n + 1)
            lm['y'] = (lm['y'] * n + y) / (n + 1)
            lm['n'] = n + 1

    def confirmed_wall_landmarks(
        self,
        min_sightings: int = WALL_MIN_SIGHTINGS,
    ) -> List[Tuple[float, float]]:
        """Wall landmarks seen at least `min_sightings` times, as
        (x_map, y_map)."""
        return [(lm['x'], lm['y'])
                for lm in self.wall_landmarks
                if lm['n'] >= min_sightings]


    
    # ------------------------------------------------------------------
    # Frame conversions
    # ------------------------------------------------------------------
    def pixel_to_body(self, px: float, py: float) -> Optional[Tuple[float, float]]:
        """Project image pixel onto the ground plane (z=0 in base_footprint
        frame — that's the actual ground; base_link sits chassis_z above it).
        Returns (x, y) in base_footprint frame, which has the same xy as
        base_link, so downstream body-frame code is unaffected."""
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
                far, 'base_footprint', Duration(seconds=0.1))
            origin_body = self.tf_buffer.transform(
                origin, 'base_footprint', Duration(seconds=0.1))
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
