"""Lidar-only perception helpers for the WRO mission node.

Encapsulates the /scan subscription and any geometry that depends on
raw lidar data. Owning class instantiates this and calls its methods —
no separate node.

Angles passed to `_median_in_arc` / `front_distance` are in the ROBOT
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


# Half-angle of the arc used when nobody passes one explicitly (~16° wedge).
DEFAULT_HALF_WIDTH_RAD = math.radians(8)


class LidarPerception:
    def __init__(
        self,
        node,
        tf_buffer: tf2_ros.Buffer,
        scan_topic: str = '/scan',
        forward_margin_m: float = 0.30,   # clearance kept in front when picking a forward waypoint
        max_forward_m: float = 3.0,       # cap on how far ahead we ever aim
        robot_frame: str = 'base_link',
        laser_frame: str = 'base_laser',
    ):
        self.tf_buffer = tf_buffer
        self.forward_margin_m = forward_margin_m
        self.max_forward_m = max_forward_m
        self.robot_frame = robot_frame
        self.laser_frame = laser_frame
        # Latest LaserScan msg — updated by _on_scan, read by every query.
        self.latest_scan: Optional[LaserScan] = None
        # Cached lidar→robot yaw offset, filled on first successful TF lookup.
        self._mount_yaw: Optional[float] = None

        node.create_subscription(LaserScan, scan_topic, self._on_scan, 10)

    def front_distance(self, center_rad: float = 0.0) -> Optional[float]:
        """Convenience: median front-arc distance in metres."""
        return self._median_in_arc(center_rad)
    
    def left_min_distance(
        self,
        half_width_rad: float = DEFAULT_HALF_WIDTH_RAD,
        min_range_m: float = 0.15,
    ) -> Optional[float]:
        """Shortest valid range on the robot's left (+pi/2) side."""
        return self._min_in_arc(math.pi / 2, half_width_rad, min_range_m)

    def right_min_distance(
        self,
        half_width_rad: float = DEFAULT_HALF_WIDTH_RAD,
        min_range_m: float = 0.15,
    ) -> Optional[float]:
        """Shortest valid range on the robot's right (-pi/2) side."""
        return self._min_in_arc(-math.pi / 2, half_width_rad, min_range_m)

    def longest_forward_ray_angle(self) -> Optional[float]:
        """Angle (in the ROBOT frame, radians) of the longest valid
        lidar return inside the forward half-plane [-pi/2, +pi/2].
        Returns None if no scan / no TF yet, or no valid return in
        the arc.

        Useful for pointing toward the widest opening ahead — e.g.
        picking which way is "down the corridor".
        """
        scan = self.latest_scan
        if scan is None:
            return None
        mount_yaw = self._get_mount_yaw()
        if mount_yaw is None:
            return None
        # Robot-frame forward is at laser-frame angle -mount_yaw.
        laser_center = -mount_yaw
        half_width = math.pi / 2.0
        best_range: float = -1.0
        best_robot_angle: Optional[float] = None
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            # Signed shortest angle from robot-forward to this beam
            # (handles the ±π wrap). Equal to the beam's robot-frame
            # angle since our center is 0 in the robot frame.
            d = math.atan2(
                math.sin(angle - laser_center),
                math.cos(angle - laser_center),
            )
            # Skip beams outside the forward half-plane.
            if abs(d) > half_width:
                continue
            # Skip NaN / inf / self-hits / max-range phantoms.
            if not math.isfinite(r):
                continue
            if r <= scan.range_min or r >= scan.range_max:
                continue
            if r > best_range:
                best_range = r
                best_robot_angle = d
        return best_robot_angle

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
        # Distance to the wall ahead in a small forward arc.
        front = self.front_distance(center_rad)
        if front is None:
            return None
        # Clamp to caller's hop limit, then pull back by the safety margin.
        cap = self.max_forward_m if max_distance is None else max_distance
        dist = min(front, cap) - self.forward_margin_m
        # Too little room to be worth publishing a target.
        if dist <= 0.1:
            return None
        # Project `dist` metres forward from pose into map frame.
        rx, ry, yaw = pose
        return (rx + dist * math.cos(yaw), ry + dist * math.sin(yaw))


    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        # Overwrite in place — every query reads the most recent scan.
        self.latest_scan = msg

    # ------------------------------------------------------------------
    # Mount yaw (lidar -> robot), cached after first successful lookup
    # ------------------------------------------------------------------
    def _get_mount_yaw(self) -> Optional[float]:
        # Cached: TF lookup only happens once.
        if self._mount_yaw is not None:
            return self._mount_yaw
        # Ask TF for base_link ← base_laser. Times out fast if unavailable.
        try:
            t = self.tf_buffer.lookup_transform(
                self.robot_frame, self.laser_frame, Time(),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return None
        # Quaternion → yaw (rotation about Z).
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._mount_yaw = yaw
        return yaw


    # ------------------------------------------------------------------
    # Queries — angles are in the ROBOT frame (0 = forward)
    # ------------------------------------------------------------------
    def _median_in_arc(
        self,
        center_rad: float,
        half_width_rad: float = DEFAULT_HALF_WIDTH_RAD,
    ) -> Optional[float]:
        """Median finite range in a small arc, or None if no scan / no
        TF yet."""
        # Need both a recent scan and the mount yaw to interpret it.
        scan = self.latest_scan
        if scan is None:
            return None
        yaw = self._get_mount_yaw()
        if yaw is None:
            return None
        # Convert robot-frame center into laser-frame center.
        laser_center = center_rad - yaw
        vals = []
        for i, r in enumerate(scan.ranges):
            # Angle of this beam in the laser frame.
            angle = scan.angle_min + i * scan.angle_increment
            # Signed shortest angle to laser_center (handles the ±π wrap).
            d = math.atan2(
                math.sin(angle - laser_center),
                math.cos(angle - laser_center),
            )
            # Keep beams inside the arc with a real, in-range return.
            if (
                abs(d) <= half_width_rad
                and math.isfinite(r)
                and scan.range_min < r < scan.range_max
            ):
                vals.append(r)
        # Nothing hit: treat the arc as clear out to max range.
        if not vals:
            return scan.range_max
        # Median is robust against a few outlier beams (glass, edges…).
        vals.sort()
        return vals[len(vals) // 2]

    def _min_in_arc(
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
            # Signed shortest angle to laser_center (handles the ±π wrap).
            d = math.atan2(
                math.sin(angle - laser_center),
                math.cos(angle - laser_center),
            )
            # Skip beams outside the arc.
            if abs(d) > half_width_rad:
                continue
            # Skip NaN / inf returns.
            if not math.isfinite(r):
                continue
            # Skip self-hits (too close) and max-range phantoms.
            if r <= min_range_m or r >= scan.range_max:
                continue
            # Track the closest valid hit — a fresh min without the
            # cost of collecting a full list.
            if best is None or r < best:
                best = r
        return best
