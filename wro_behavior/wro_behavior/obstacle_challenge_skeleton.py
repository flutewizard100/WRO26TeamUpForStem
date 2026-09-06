"""WRO Obstacle Challenge — bare skeleton.

Sensors set up, state variables declared, timer running.
No helpers, no logic. Add your own inside step().
"""
import math

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from vision_msgs.msg import Detection2DArray


# ============================================================================
# Tuning constants
# ============================================================================
CRUISE_SPEED = 0.25
KP = 4.0
MAX_ANGULAR_Z = 1.5
TARGET_WALL_DIST = 0.45
CORNERS_PER_RACE = 12
TURN_ADJUST = 45
# FIX: added — addBlocks() referenced these but they were undefined.
THETA_1 = math.radians(30)          # section angular width, radians
DIP_THRESHOLD = 0.15                # meters — jump size that counts as block edge
X_OFFSET = 0.0                      # extra x term in the waypoint formula (tune)


# ============================================================================
# The node
# ============================================================================
class ObstacleChallenge(Node):
    def __init__(self):
        super().__init__('obstacle_challenge_skeleton')

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.bridge = CvBridge()

        # ---- Subscribers ----
        self.create_subscription(LaserScan, '/scan',         self.on_scan,  sensor_qos)
        self.create_subscription(Odometry,  '/odom',         self.on_odom,  10)
        self.create_subscription(Imu,       '/imu/data_raw', self.on_imu,   sensor_qos)
        self.create_subscription(Image,     '/camera',       self.on_image, sensor_qos)

        # ---- Publishers ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ---- Latest sensor data ----
        self.latest_scan = None
        self.latest_frame = None       # numpy BGR image from on_image

        # ---- Robot state ----
        self.yaw = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw_rate = 0.0

        # ---- State machine ----
        self.state = 'INIT'
        self.corners_done = 0
        self.corner_ticks = 0
        self.corner_yaw = 0.0

        # ---- Camera ----
        self.detections = []
        self.create_subscription(Detection2DArray, '/limelight/detections',
                         lambda msg: setattr(self, 'detections', msg.detections),
                         10)

        # ---- Block Positions ----
        self.blockPositionsArray = []
        # FIX: added — addBlocks() referenced self.OFFSET_ANGLE_NUM but it was undefined.
        self.OFFSET_ANGLE_NUM = 10

        # ---- Nav2 goal tracking ----
        self.corner_origin = (0.0, 0.0, 0.0)   # (x, y, yaw) snapshot of last corner
        self.active_goal   = None              # (gx, gy) in odom, None if no active goal
        self.goal_tolerance = 0.15             # meters — "close enough" to a goal
        self.pending_waypoints = []            # queue of remaining (x, y) targets after the current one

        # ---- Timer ----
        self.create_timer(0.05, self.step)     # 20 Hz control loop

        self.get_logger().info('skeleton up')

    # ------------------------------------------------------------------------
    # Callbacks — just store data. Do real work in step().
    # ------------------------------------------------------------------------
    def on_scan(self, msg):
        self.latest_scan = msg

    def on_odom(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def on_imu(self, msg):
        self.yaw_rate = msg.angular_velocity.z

    def on_image(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
    def range_at_angle(self, angle_rad):
        """Lidar distance at a given body-frame angle. inf if nothing there."""
        if self.latest_scan is None:
            return float('inf')
        scan = self.latest_scan
        idx = int((angle_rad - scan.angle_min) / scan.angle_increment)
        if idx < 0 or idx >= len(scan.ranges):
            return float('inf')
        dist = scan.ranges[idx]
        return dist if math.isfinite(dist) else float('inf')

    # ------------------------------------------------------------------------
    # Nav2 helpers (send goals, check completion, corner-relative coords)
    # ------------------------------------------------------------------------
    def goal_from_corner(self, dx, dy):
        """Convert (dx, dy) local to the last corner into odom-frame (x, y).

        `dx` = meters forward from the corner (in the direction the robot faced
        when the corner completed). `dy` = meters left. Returns (odom_x, odom_y).
        """
        cx, cy, cyaw = self.corner_origin
        wx = cx + dx * math.cos(cyaw) - dy * math.sin(cyaw)
        wy = cy + dx * math.sin(cyaw) + dy * math.cos(cyaw)
        return (wx, wy)

    def send_nav2_goal(self, x, y, yaw=0.0):
        """Publish a PoseStamped goal in odom frame and record it as active."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)
        self.active_goal = (x, y)

    def is_at_goal(self):
        """True if within self.goal_tolerance of the active goal, or no active goal."""
        if self.active_goal is None:
            return True
        gx, gy = self.active_goal
        d = math.hypot(self.pose_x - gx, self.pose_y - gy)
        if d < self.goal_tolerance:
            self.active_goal = None    # clear so we don't repeatedly report done
            return True
        return False

    def addBlocks(self):
        currentSectionArray = []
        # CHANGE: filter detections by range (< NEAR meters) and take the closest
        # 2. Matches real Limelight behavior (short useful range) and keeps the
        # 1/2-block branches working when the sim shows 3+ pillars at once.
        NEAR = 1.5
        dets = sorted(
            (d for d in self.detections
             if d.results and d.results[0].pose.pose.position.z < NEAR),
            key=lambda d: d.results[0].pose.pose.position.z,
        )[:2]
        count = len(dets)
        # CHANGE: build blocks into `section` list, then append that whole list
        #         to self.blockPositionsArray once (nested structure).
        section = []
        #If only one block seen
        if count == 1:
            #Make array to store block infor
            closeBlock = []
            #Check the color of block and set scaler + red, - green and appen to start of close block array
            color = dets[0].results[0].hypothesis.class_id
            if color == 'red_pillar':
                colorScaler = 1
            elif color == 'green_pillar':
                colorScaler = -1
            closeBlock.append(colorScaler)
            
            #Find which position its in for x Pose using lidar /scan
            front_wall = self.latest_scan.ranges[
                   int((0.0 - self.latest_scan.angle_min) / self.latest_scan.angle_increment)
                ]
            # CHANGE: xPose removed — color determines pass side, so we don't need it.
            # FIX: yPose init added, and used as an outer-loop break flag below.
            yPose = None
            for i in range(3):
                # FIX: was `last_dist = self.range_at_angle(angle)` — `angle` was
                #      not defined here yet (defined only inside the j-loop).
                last_dist = None
                for j in range(self.OFFSET_ANGLE_NUM):
                    # Angle to sample this tick. Adjust the formula to match your math.
                    angle = i * THETA_1 + j * (THETA_1 / self.OFFSET_ANGLE_NUM)
                    dist = self.range_at_angle(angle)

                    if last_dist is not None and abs(dist - last_dist) > DIP_THRESHOLD:
                        # Big jump → block edge found in section i
                        # CHANGE: removed stray `self.blockPositionsArray.append(i)`
                        #         which was polluting the array with ints.
                        # FIX: capture yPose here (was set to `i` after both loops,
                        #      which always left yPose = 2).
                        yPose = i
                        break
                    # FIX: added — without this, last_dist never updated and the
                    #      dip check never fired.
                    last_dist = dist
                # FIX: outer-loop break so we stop scanning once we've found the
                #      block's section (was implicit; only inner loop broke).
                if yPose is not None:
                    break

            closeBlock.append(yPose)
            # CHANGE: append into local `section` list instead of directly into
            #         self.blockPositionsArray. Whole section is pushed at the end.
            section.append(closeBlock)



        elif count == 2:
            # Two blocks in range — dets is already sorted by z (closer first).
            closer, farther = dets[0], dets[1]

            closeBlock = []
            farBlock = []

            for det, out in ((closer, closeBlock), (farther, farBlock)):
                # colorScaler
                color = det.results[0].hypothesis.class_id
                if color == 'red_pillar':
                    colorScaler = 1
                elif color == 'green_pillar':
                    colorScaler = -1
                out.append(colorScaler)

                # CHANGE: xPose removed — color determines pass side.

                # yPose (lidar section sweep)
                yPose = None
                for i in range(3):
                    last_dist = None
                    for j in range(self.OFFSET_ANGLE_NUM):
                        angle = i * THETA_1 + j * (THETA_1 / self.OFFSET_ANGLE_NUM)
                        dist = self.range_at_angle(angle)
                        if last_dist is not None and abs(dist - last_dist) > DIP_THRESHOLD:
                            yPose = i
                            break
                        last_dist = dist
                    if yPose is not None:
                        break
                out.append(yPose)

            # CHANGE: append into local `section` list instead of directly into
            #         self.blockPositionsArray. Whole section is pushed at the end.
            section.append(closeBlock)
            section.append(farBlock)

        # CHANGE: push the whole section (list of blocks) onto the outer array,
        #         but only if we actually saw any blocks.
        if section:
            self.blockPositionsArray.append(section)
    

    def choosePath(self, section):
        # Waypoint formula for a single block:
        #   dx = colorScaler * (4 + X_OFFSET)
        #   dy = 20 * yPose
        # Each block is [colorScaler, yPose] after xPose was dropped.
        LANE_LEN = 1.0    # meters — how far down the lane the next corner sits

        def waypoint_for(block):
            colorScaler, yPose = block[0], block[1]
            # FIX: yPose can be None if the lidar sweep found no dip. Default to 0
            #      so we still produce a valid waypoint instead of crashing on `20 * None`.
            if yPose is None:
                yPose = 0
            # CHANGE: numbers are now in METERS (were 4 and 20 which are huge
            # compared to a 3×3 m field). Divide by 100 to interpret them as cm.
            dx = colorScaler * (0.04 + X_OFFSET)
            dy = 0.20 * yPose
            return self.goal_from_corner(dx, dy)

        if len(section) == 1:
            # Scenario 1: one block. Waypoint at block pos, then continue to next corner.
            self.send_nav2_goal(*waypoint_for(section[0]))
            self.pending_waypoints = [
                self.goal_from_corner(LANE_LEN, 0.0),
            ]

        elif len(section) == 2:
            close, far = section[0], section[1]

            if close[0] == far[0]:
                # Scenario 2a: same color → one waypoint (the close one), then corner.
                self.send_nav2_goal(*waypoint_for(close))
                self.pending_waypoints = [
                    self.goal_from_corner(LANE_LEN, 0.0),
                ]
            else:
                # Scenario 2b: different colors → close waypoint, then far waypoint, then corner.
                self.send_nav2_goal(*waypoint_for(close))
                self.pending_waypoints = [
                    waypoint_for(far),
                    self.goal_from_corner(LANE_LEN, 0.0),
                ]

    # ------------------------------------------------------------------------
    # Control loop — runs at 20 Hz
    # ------------------------------------------------------------------------
    def step(self):
        if self.latest_scan is None:
            return
        cmd = Twist()

        # DEBUG: verbose per-tick status
        print(f'[tick] detections={len(self.detections)} '
              f'blockArr={len(self.blockPositionsArray)} '
              f'pending={len(self.pending_waypoints)} '
              f'active_goal={self.active_goal}'
              f'blocks={self.blockPositionsArray}',
              flush=True)
# Detect blocks + decide waypoint for the current section
        self.addBlocks()
        if self.blockPositionsArray:
            self.choosePath(self.blockPositionsArray[-1])
            # Advance queued waypoints as Nav2 finishes them
        if self.is_at_goal() and self.pending_waypoints:
            wx, wy = self.pending_waypoints.pop(0)
            self.send_nav2_goal(wx, wy)

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
