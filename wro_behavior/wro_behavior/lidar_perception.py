"""Lidar-only perception helpers for the WRO mission node.

Encapsulates the /scan subscription and any geometry that depends on
raw lidar data. Owning class instantiates this and calls its methods —
no separate node.

Usage:
    self.lidar = LidarPerception(self)   # `self` is a rclpy Node
    front = self.lidar.front_distance()
    dyn = self.lidar.forward_edge_waypoint((rx, ry, yaw))
"""
import math
from typing import Optional, Tuple

from sensor_msgs.msg import LaserScan


DEFAULT_HALF_WIDTH_RAD = math.radians(8)
LASER_MOUNT_YAW_RAD = -1.5708 


class LidarPerception:
    """Subscribes to /scan and provides distance queries.

    All bearing arguments are in the sensor's own frame (`base_laser`).
    If the lidar is rotated relative to `base_link` (URDF joint has a
    non-zero yaw), pass the compensated angle when calling median_in_arc.
    """

    def __init__(
        self,
        node,
        scan_topic: str = '/scan',
        forward_margin_m: float = 0.30,
        max_forward_m: float = 3.0,
    ):
        self.node = node
        self.forward_margin_m = forward_margin_m
        self.max_forward_m = max_forward_m
        self.latest_scan: Optional[LaserScan] = None

        node.create_subscription(LaserScan, scan_topic, self._on_scan, 10)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------
    def median_in_arc(
        self,
        center_rad: float,
        half_width_rad: float = DEFAULT_HALF_WIDTH_RAD,
    ) -> Optional[float]:
        """Median finite range in a small arc, or None if no scan yet."""
        scan = self.latest_scan
        if scan is None:
            return None
        laser_center = center_rad - LASER_MOUNT_YAW_RAD #Change *
        vals = []
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            d = math.atan2(
                math.sin(angle - center_rad),
                math.cos(angle - center_rad),
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

    def forward_edge_waypoint(
        self,
        pose: Tuple[float, float, float],
        center_rad: float = 0.0,
    ) -> Optional[Tuple[float, float]]:
        """Compute (x_map, y_map) straight ahead of `pose`, at the edge
        of visible range minus safety margin. Returns None if no scan
        is available or nothing worthwhile to drive toward.
        """
        front = self.median_in_arc(center_rad)
        if front is None:
            return None
        dist = min(front, self.max_forward_m) - self.forward_margin_m
        if dist <= 0.1:
            return None
        rx, ry, yaw = pose
        return (rx + dist * math.cos(yaw), ry + dist * math.sin(yaw))
