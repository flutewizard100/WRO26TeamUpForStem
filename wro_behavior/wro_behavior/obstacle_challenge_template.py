"""WRO Obstacle Challenge — starter skeleton.

Extends the open_challenge state machine with camera-based block detection
and block localization via lidar cross-reference.

Data flow per frame:
  camera image  → HSV mask on red + green
                → largest contour per color (if above MIN_BLOB_AREA)
                → convert pixel_x → bearing (rad) using camera HFOV
                → look up distance in /scan at that bearing
                → store as Block(color, cx, cy, area, bearing_rad, distance_m)

`self.visible_blocks` is refreshed every camera frame (~30 Hz). Read it
inside step() to decide what to do.

Run in sim:
  ros2 launch wro_sim sim.launch.py rviz:=true
  ros2 run wro_behavior obstacle_challenge_template
"""
import math
from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped


# ============================================================================
# Tuning constants — edit these, don't hardcode numbers in the logic below.
# ============================================================================
CRUISE_SPEED = 0.25
KP = 4.0
MAX_ANGULAR_Z = 1.5
TARGET_WALL_DIST = 0.45
CORNERS_PER_RACE = 12          # 4 corners × 3 laps
TURN_ADJUST = 45               # degrees short of a full 90°; residual makes up

# ---- Camera / color detection --------------------------------------------
# OpenCV HSV: H is 0-179, S and V are 0-255.
# Red wraps around 0/180, so it needs two ranges.
RED_HSV_LOW_1  = (0,   100, 80)
RED_HSV_HIGH_1 = (10,  255, 255)
RED_HSV_LOW_2  = (170, 100, 80)
RED_HSV_HIGH_2 = (179, 255, 255)
GREEN_HSV_LOW  = (40,  100, 80)
GREEN_HSV_HIGH = (80,  255, 255)

MIN_BLOB_AREA = 300            # pixels; contours smaller than this are ignored
CAMERA_HFOV_RAD = math.radians(60)   # matches URDF horizontal_fov=1.0472

# ---- Corner waypoint ------------------------------------------------------
CORNER_LOOKAHEAD_MAX = 1.0     # meters ahead the "next corner" waypoint sits


# ============================================================================
# Block record — populated per camera frame, consumed in step()
# ============================================================================
@dataclass
class Block:
    color: str            # 'RED' or 'GREEN'
    cx: int               # pixel x of blob centroid
    cy: int               # pixel y of blob centroid
    area: int             # blob area in pixels (bigger = closer)
    bearing_rad: float    # angle to block in robot body frame (+ = left/CCW)
    distance_m: float     # lidar range in that direction (inf if nothing)


# ============================================================================
# Helpers
# ============================================================================
def median_in_arc(scan: LaserScan, center_rad: float,
                  half_width_rad: float = math.radians(8)) -> float:
    """Median finite lidar range within a small arc, or scan.range_max."""
    idx_center = (center_rad - scan.angle_min) / scan.angle_increment
    idx_half = half_width_rad / scan.angle_increment
    lo = max(0, int(idx_center - idx_half))
    hi = min(len(scan.ranges), int(idx_center + idx_half) + 1)
    vals = [r for r in scan.ranges[lo:hi]
            if math.isfinite(r) and scan.range_min < r < scan.range_max]
    if not vals:
        return scan.range_max
    return sorted(vals)[len(vals) // 2]


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def largest_contour(mask: np.ndarray, min_area: int) -> Optional[dict]:
    """Return {'cx','cy','area'} for the biggest contour in mask, or None."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    biggest = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(biggest))
    if area < min_area:
        return None
    M = cv2.moments(biggest)
    if M['m00'] == 0:
        return None
    return {'cx': int(M['m10'] / M['m00']),
            'cy': int(M['m01'] / M['m00']),
            'area': area}


def pixel_to_bearing(cx: int, image_width: int, hfov: float) -> float:
    """cx=W/2 → 0 rad; cx=0 → +hfov/2 (left); cx=W → -hfov/2 (right)."""
    normalized = (cx / image_width) - 0.5    # -0.5 .. +0.5
    return -normalized * hfov                # positive bearing = left/CCW


# ============================================================================
# The node
# ============================================================================
class ObstacleChallenge(Node):
    def __init__(self):
        super().__init__('obstacle_challenge_template')

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.bridge = CvBridge()

        # ---- Subscribers ----
        self.create_subscription(LaserScan, '/scan', self.on_scan, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, sensor_qos)
        self.create_subscription(Image, '/camera', self.on_image, sensor_qos)

        # ---- Publisher ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Nav2 goal_pose (RViz-compatible single-goal). Nav2 must be up.
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ---- State ----
        self.latest_scan: Optional[LaserScan] = None
        self.yaw = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw_rate = 0.0
        self.state = 'INIT'
        self.corners_done = 0
        self.corner_ticks = 0
        self.corner_yaw = 0.0
        self.visible_blocks: List[Block] = []   # refreshed by on_image()
        self.offset = 0
        self.waypoint_x = 0.0
        self.waypoint_y = 0.0
        self.waypoint_yaw = 0.0

        # ---- Timer ----
        self.create_timer(0.05, self.step)
        self.get_logger().info('obstacle_challenge_template up')

    # ------------------------------------------------------------------------
    # Callbacks — keep short. Real work happens in step().
    # ------------------------------------------------------------------------
    def on_scan(self, msg):
        self.latest_scan = msg

    def on_odom(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_imu(self, msg):
        self.yaw_rate = msg.angular_velocity.z

    def on_image(self, msg):
        """HSV threshold → largest red/green contour → bearing + lidar depth.

        Writes self.visible_blocks (list of Block, 0-2 entries).
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        h, w, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask = (cv2.inRange(hsv, RED_HSV_LOW_1, RED_HSV_HIGH_1)
                    | cv2.inRange(hsv, RED_HSV_LOW_2, RED_HSV_HIGH_2))
        green_mask = cv2.inRange(hsv, GREEN_HSV_LOW, GREEN_HSV_HIGH)

        found: List[Block] = []
        for color, mask in (('RED', red_mask), ('GREEN', green_mask)):
            blob = largest_contour(mask, MIN_BLOB_AREA)
            if blob is None:
                continue
            bearing = pixel_to_bearing(blob['cx'], w, CAMERA_HFOV_RAD)
            distance = self.range_at_bearing(bearing)
            found.append(Block(
                color=color,
                cx=blob['cx'], cy=blob['cy'], area=blob['area'],
                bearing_rad=bearing, distance_m=distance,
            ))

        self.visible_blocks = found

    def range_at_bearing(self, bearing_rad: float) -> float:
        """Look up lidar range in a small arc centered on `bearing_rad`."""
        if self.latest_scan is None:
            return float('inf')
        return median_in_arc(self.latest_scan, bearing_rad, math.radians(4))

    # ------------------------------------------------------------------------
    # Block-query helpers you can call from step()
    # ------------------------------------------------------------------------
    def nearest_block(self) -> Optional[Block]:
        """Return the closest visible block (by lidar distance), or None."""
        if not self.visible_blocks:
            return None
        return min(self.visible_blocks, key=lambda b: b.distance_m)

    def nearest_block_color(self) -> Optional[str]:
        """'RED', 'GREEN', or None if no block visible."""
        b = self.nearest_block()
        return b.color if b else None

    # ------------------------------------------------------------------------
    # Nav2 goal_pose helper
    # ------------------------------------------------------------------------
    def send_nav2_goal(self, x: float, y: float, yaw: float = 0.0,
                       frame_id: str = 'map') -> None:
        """Publish a PoseStamped goal to /goal_pose. Nav2 must be running.

        Args:
          x, y:     goal position, in `frame_id` (default 'map').
          yaw:      goal heading in radians.
          frame_id: usually 'map' when Nav2/AMCL are up. If you don't have
                    map→odom (no Nav2), pass 'odom' and trust your odom
                    to be drift-free enough.
        """
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = frame_id
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        # Yaw → quaternion (roll=pitch=0)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'Sent Nav2 goal: ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.0f}°) '
            f'in {frame_id}')

    # ------------------------------------------------------------------------
    # Control loop — 20 Hz
    # ------------------------------------------------------------------------
    def step(self):
        if self.latest_scan is None:
            return
        scan = self.latest_scan

        front = median_in_arc(scan, 0.0)
        left  = median_in_arc(scan,  math.pi / 2)
        right = median_in_arc(scan, -math.pi / 2)

        # Closest block visible right now (or None)
        block = min(self.visible_blocks, key=lambda b: b.distance_m,
                    default=None)

        block_desc = (f"{block.color}@{block.distance_m:.2f}m/"
                      f"{math.degrees(block.bearing_rad):+.0f}deg"
                      if block else "none")
        print(f"[{self.state}] left={left:.2f} right={right:.2f} "
              f"front={front:.2f} yaw={math.degrees(self.yaw):+.1f} "
              f"block={block_desc} corners={self.corners_done}",
              flush=True)

        cmd = Twist()

        if self.state == 'INIT':
            self.state = 'LANE_FOLLOW'

        elif self.state == 'SET_WAYPOINT':
            b = self.nearest_block()

            if b is not None:
                # ---- BLOCK CASE: offset waypoint sideways of the block ----
                if b.color == 'RED':
                    self.offset = OFFSET_DIST * -1
                elif b.color == 'GREEN':
                    self.offset = OFFSET_DIST
                else:
                    self.offset = 0

                # 1. Aim pixel offset from the block centroid
                target_cx = b.cx + self.offset
                # 2. Pixel → body-frame bearing (0 rad = straight ahead)
                target_bearing = pixel_to_bearing(target_cx, 640, CAMERA_HFOV_RAD)
                # 3. Body-frame bearing → world bearing → (x, y) in odom frame
                world_bearing = self.yaw + target_bearing
                self.waypoint_x = self.pose_x + b.distance_m * math.cos(world_bearing)
                self.waypoint_y = self.pose_y + b.distance_m * math.sin(world_bearing)
                self.waypoint_yaw = world_bearing
            else:
                # ---- CORNER CASE: waypoint at the next corner, rotated 90° ----
                # Position: project forward as far as the front lidar sees,
                # capped by CORNER_LOOKAHEAD_MAX so we don't overshoot.
                dist_ahead = min(front, CORNER_LOOKAHEAD_MAX)
                self.waypoint_x = self.pose_x + dist_ahead * math.cos(self.yaw)
                self.waypoint_y = self.pose_y + dist_ahead * math.sin(self.yaw)
                # Yaw: absolute target for the next corner in a CCW lap.
                # After N corners the heading should be (N+1) * 90° - TURN_ADJUST.
                self.waypoint_yaw = (self.corners_done + 1) * math.pi / 2

            # Hand off to Nav2 and swap states
            self.send_nav2_goal(self.waypoint_x, self.waypoint_y,
                                yaw=self.waypoint_yaw, frame_id='odom')
            self.state = 'DRIVE_TO_WAYPOINT'
        elif self.state == 'DRIVE_TO_WAYPOINT':
            # Vector from robot to waypoint (in odom frame)
            dx = self.waypoint_x - self.pose_x
            dy = self.waypoint_y - self.pose_y
            dist = math.hypot(dx, dy)

            # Bearing to waypoint, and heading error (wrap-safe)
            target_bearing = math.atan2(dy, dx)
            heading_err = math.atan2(math.sin(target_bearing - self.yaw),
                                     math.cos(target_bearing - self.yaw))

            # P-control steering toward the waypoint
            cmd.linear.x = CRUISE_SPEED
            cmd.angular.z = max(-MAX_ANGULAR_Z,
                                min(MAX_ANGULAR_Z, KP * heading_err))

            # Exit conditions — whichever fires first
            if dist < 0.15:                          # within 15 cm of waypoint
                self.state = 'LANE_FOLLOW'
            elif self.nearest_block() is None:       # block passed / out of frame
                self.state = 'LANE_FOLLOW'
            elif left > 1 or right > 1:              # corner arrived
                self.state = 'CORNER'
                self.corner_ticks = 0
                self.corner_yaw = self.yaw

        elif self.state == 'LANE_FOLLOW':
            # --------------------------------------------------------
            # YOUR WALL FOLLOWER GOES HERE.
            # Copy the LANE_FOLLOW body from open_challenge_template.
            # To swerve around blocks, adjust target lane based on `block`:
            #   if block and block.color == 'RED'  and block.distance_m < 1.2:
            #       ... hug right lane
            #   if block and block.color == 'GREEN' and block.distance_m < 1.2:
            #       ... hug left lane
            # --------------------------------------------------------
            cmd.linear.x = CRUISE_SPEED
            cmd.angular.z = 0.0

            if self.corners_done >= CORNERS_PER_RACE:
                self.state = 'PARK'
            if left > 1 or right > 1:
                self.state = 'CORNER'
                self.corner_ticks = 0
                self.corner_yaw = self.yaw

        elif self.state == 'CORNER':
            cmd.angular.z = 1.2
            cmd.linear.x  = 0.15
            self.corner_ticks += 1
            target = (self.corners_done + 1) * math.pi / 2 - math.radians(TURN_ADJUST)
            yaw_err = math.atan2(math.sin(target - self.yaw),
                                 math.cos(target - self.yaw))
            if yaw_err <= 0:
                self.state = 'LANE_FOLLOW'
                self.corners_done += 1

        elif self.state == 'PARK':
            # --------------------------------------------------------
            # YOUR PARKING LOGIC GOES HERE.
            # --------------------------------------------------------
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif self.state == 'STOP':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Safety override: stop if anything in front ±15° is under 0.15 m.
        if self.latest_scan is not None:
            s = self.latest_scan
            idx0 = int((0.0 - s.angle_min) / s.angle_increment)
            half = int(math.radians(15) / s.angle_increment)
            lo = max(0, idx0 - half)
            hi = min(len(s.ranges), idx0 + half + 1)
            front = [r for r in s.ranges[lo:hi]
                     if math.isfinite(r) and s.range_min < r < s.range_max]
            if front and min(front) < 0.15:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ObstacleChallenge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
