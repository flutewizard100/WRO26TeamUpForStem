"""Lidar-only perception helpers for the WRO mission node.

Encapsulates the /scan subscription and any geometry that depends on
raw lidar data. Owning class instantiates this and calls its methods —
no separate node.

Angles passed to `median_in_arc` / `front_distance` are in the ROBOT
(base_link) frame:
    0        = forward   (+X in base_link)
    +pi/2    = left      (+Y)
    -pi/2    = right     (-Y)

The mount rotation of the physical lidar lives ONLY in the URDF joint
between `base_link` and `base_laser`. This class reads that transform
from TF and converts angles internally, so remounting the lidar only
requires editing the URDF.

Usage:
    self.lidar = LidarPerception(self, self.tf_buffer)
    front = self.lidar.front_distance()
    dyn = self.lidar.forward_edge_waypoint((rx, ry, yaw))
"""
import math
from typing import Optional, Tuple

from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

import tf2_ros


DEFAULT_HALF_WIDTH_RAD = math.radians(8)


class LidarPerception:
    def __init__(
        self,
        node,
        tf_buffer: tf2_ros.Buffer,
        scan_topic: str = '/scan',
        forward_margin_m: float = 0.30,
        max_forward_m: float = 3.0,
        robot_frame: str = 'base_link',
        laser_frame: str = 'base_laser',
    ):
        self.node = node
        self.tf_buffer = tf_buffer
        self.forward_margin_m = forward_margin_m
        self.max_forward_m = max_forward_m
        self.robot_frame = robot_frame
        self.laser_frame = laser_frame
        self.latest_scan: Optional[LaserScan] = None
        self._mount_yaw: Optional[float] = None

        node.create_subscription(LaserScan, scan_topic, self._on_scan, 10)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    # ------------------------------------------------------------------
    # Mount yaw (lidar -> robot), cached after first successful lookup
    # ------------------------------------------------------------------
    def _get_mount_yaw(self) -> Optional[float]:
        if self._mount_yaw is not None:
            return self._mount_yaw
        try:
            t = self.tf_buffer.lookup_transform(
                self.robot_frame, self.laser_frame, Time(),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._mount_yaw = yaw
        return yaw

    # ------------------------------------------------------------------
    # Queries — angles are in the ROBOT frame (0 = forward)
    # ------------------------------------------------------------------
    def median_in_arc(
        self,
        center_rad: float,
        half_width_rad: float = DEFAULT_HALF_WIDTH_RAD,
    ) -> Optional[float]:
        """Median finite range in a small arc, or None if no scan / no
        TF yet."""
        scan = self.latest_scan
        if scan is None:
            return None
        yaw = self._get_mount_yaw()
        if yaw is None:
            return None
        laser_center = center_rad - yaw
        vals = []
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            d = math.atan2(
                math.sin(angle - laser_center),
                math.cos(angle - laser_center),
            )
            if (
                abs(d) <= half_width_rad
                and math.isfinite(r)
                and scan.range_min < r < scan.range_max
            ):
                vals.append(r)
        if not vals:
            return scan.range_max
        vals.sort()
        return vals[len(vals) // 2]

    def front_distance(self, center_rad: float = 0.0) -> Optional[float]:
        """Convenience: median front-arc distance in metres."""
        return self.median_in_arc(center_rad)

    def min_in_arc(
        self,
        center_rad: float,
        half_width_rad: float = DEFAULT_HALF_WIDTH_RAD,
        min_range_m: float = 0.15,
    ) -> Optional[float]:
        """Shortest valid range in an arc, ignoring returns closer than
        `min_range_m` (self-hits / near-field noise). Returns None if no
        scan / no TF yet, or no valid return in the arc."""
        scan = self.latest_scan
        if scan is None:
            return None
        yaw = self._get_mount_yaw()
        if yaw is None:
            return None
        laser_center = center_rad - yaw
        best: Optional[float] = None
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            d = math.atan2(
                math.sin(angle - laser_center),
                math.cos(angle - laser_center),
            )
            if abs(d) > half_width_rad:
                continue
            if not math.isfinite(r):
                continue
            if r <= min_range_m or r >= scan.range_max:
                continue
            if best is None or r < best:
                best = r
        return best

    def left_min_distance(
        self,
        half_width_rad: float = DEFAULT_HALF_WIDTH_RAD,
        min_range_m: float = 0.15,
    ) -> Optional[float]:
        """Shortest valid range on the robot's left (+pi/2) side."""
        return self.min_in_arc(math.pi / 2, half_width_rad, min_range_m)

    def right_min_distance(
        self,
        half_width_rad: float = DEFAULT_HALF_WIDTH_RAD,
        min_range_m: float = 0.15,
    ) -> Optional[float]:
        """Shortest valid range on the robot's right (-pi/2) side."""
        return self.min_in_arc(-math.pi / 2, half_width_rad, min_range_m)

    def forward_edge_waypoint(
        self,
        pose: Tuple[float, float, float],
        center_rad: float = 0.0,
        max_distance: Optional[float] = None,
    ) -> Optional[Tuple[float, float]]:
        """Compute (x_map, y_map) straight ahead of `pose`, at the edge
        of visible range minus safety margin. Returns None if no scan
        is available or nothing worthwhile to drive toward.

        `max_distance` (metres) caps the step; when None, the class's
        `max_forward_m` is used. Pass a smaller value for short
        exploration hops.
        """
        front = self.median_in_arc(center_rad)
        if front is None:
            return None
        cap = self.max_forward_m if max_distance is None else max_distance
        dist = min(front, cap) - self.forward_margin_m
        if dist <= 0.1:
            return None
        rx, ry, yaw = pose
        return (rx + dist * math.cos(yaw), ry + dist * math.sin(yaw))
