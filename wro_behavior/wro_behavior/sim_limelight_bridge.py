"""Sim-only fake Limelight bridge.

Consumes /camera from the Gazebo sim, runs HSV color detection for
red/green pillars, computes a 3D pose in camera_optical_frame using the
same range-from-bbox-height math as WRORobot/limelight_bridge.py, and
publishes a vision_msgs/Detection2DArray on /limelight/detections.

The message format is byte-compatible with the real Limelight bridge:
  header.frame_id  = 'camera_optical_frame'
  detections[i].bbox.center.{x,y} and size_{x,y} in pixels
  detections[i].results[0].hypothesis.class_id = 'red_pillar' | 'green_pillar'
  detections[i].results[0].pose.pose.position  = (x,y,z) in camera_optical_frame
    (X right, Y down, Z forward)

So any downstream node that subscribes to /limelight/detections runs the
same code on sim and real hardware — no branching, no if-sim guards.
"""
import math
from typing import List, Optional, Tuple

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)


# ============================================================================
# Camera intrinsics for the sim camera (URDF: 640x480, hfov 1.0472 rad).
#   fx = width / (2 * tan(hfov/2))
# ============================================================================
IMAGE_WIDTH  = 640
IMAGE_HEIGHT = 480
HFOV_RAD     = 1.0472
FX = IMAGE_WIDTH  / (2.0 * math.tan(HFOV_RAD / 2.0))    # ≈ 554.26
FY = FX                                                  # square pixels
CX = IMAGE_WIDTH  / 2.0
CY = IMAGE_HEIGHT / 2.0

# Real-world height of WRO pillars, used for range-from-bbox-height.
PILLAR_HEIGHT_M = 0.10

# HSV thresholds — same as obstacle_challenge_template.py
RED_HSV_LOW_1  = (0,   100, 80)
RED_HSV_HIGH_1 = (10,  255, 255)
RED_HSV_LOW_2  = (170, 100, 80)
RED_HSV_HIGH_2 = (179, 255, 255)
GREEN_HSV_LOW  = (40,  100, 80)
GREEN_HSV_HIGH = (80,  255, 255)

# Floor lines. Wider H tolerance than pillars because the mat colour differs
# a bit from a solid-plastic pillar.
ORANGE_HSV_LOW  = (5,   120, 100)
ORANGE_HSV_HIGH = (20,  255, 255)
BLUE_HSV_LOW    = (100, 120, 60)
BLUE_HSV_HIGH   = (130, 255, 255)
MIN_LINE_AREA   = 200            # pixels; lines are thin so lower than pillars

MIN_BLOB_AREA = 300
FRAME_ID = 'camera_optical_frame'


class SimLimelightBridge(Node):
    def __init__(self):
        super().__init__('sim_limelight_bridge')
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera', self.on_image, sensor_qos)
        self.pub = self.create_publisher(Detection2DArray, '/limelight/detections', 10)

        self.get_logger().info('sim_limelight_bridge up (publishing to /limelight/detections)')

    # ------------------------------------------------------------------------
    def on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask    = (cv2.inRange(hsv, RED_HSV_LOW_1, RED_HSV_HIGH_1)
                       | cv2.inRange(hsv, RED_HSV_LOW_2, RED_HSV_HIGH_2))
        green_mask  = cv2.inRange(hsv, GREEN_HSV_LOW, GREEN_HSV_HIGH)
        orange_mask = cv2.inRange(hsv, ORANGE_HSV_LOW, ORANGE_HSV_HIGH)
        blue_mask   = cv2.inRange(hsv, BLUE_HSV_LOW,   BLUE_HSV_HIGH)

        detections: List[Detection2D] = []
        stamp = msg.header.stamp    # keep the camera capture stamp

        # Pillars — 3D pose from bbox height (real objects).
        for class_name, mask in (('red_pillar', red_mask),
                                 ('green_pillar', green_mask)):
            for det in self.extract_detections(mask, min_area=MIN_BLOB_AREA):
                detections.append(self.build_detection(det, class_name, stamp))

        # Floor lines — flat markers. Same message format, but pose.z ≈ 0.
        for class_name, mask in (('orange_line', orange_mask),
                                 ('blue_line',   blue_mask)):
            for det in self.extract_detections(mask, min_area=MIN_LINE_AREA):
                detections.append(self.build_line_detection(det, class_name, stamp))

        out = Detection2DArray()
        out.header.stamp = stamp
        out.header.frame_id = FRAME_ID
        out.detections = detections
        self.pub.publish(out)

    # ------------------------------------------------------------------------
    def extract_detections(self, mask: np.ndarray, min_area: int = MIN_BLOB_AREA
                           ) -> List[Tuple[float, float, float, float]]:
        """Return list of (cx, cy, bbox_w, bbox_h) for blobs above min area."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w <= 0 or h <= 0:
                continue
            cx = x + w / 2.0
            cy = y + h / 2.0
            out.append((cx, cy, float(w), float(h)))
        return out

    # ------------------------------------------------------------------------
    def build_line_detection(self, det, class_name: str, stamp) -> Detection2D:
        """Build a Detection2D for a floor line. Same message shape as pillars
        but pose is meaningless (flat marker), so z is set from a rough ground
        projection using the pixel row (cy) and camera intrinsics only in x."""
        cx, cy, bbox_w, bbox_h = det
        # Rough forward distance from the pixel row (assumes camera looking
        # slightly down; you may need to tune). If unused downstream, safe
        # placeholder is (0, 0, 0).
        z_cam = 0.0
        x_cam = (cx - CX) * 0.001    # tiny nonzero so it's not exactly (0,0,0)
        y_cam = (cy - CY) * 0.001

        d = Detection2D()
        d.header.stamp = stamp
        d.header.frame_id = FRAME_ID
        d.bbox = BoundingBox2D()
        d.bbox.center.position.x = float(cx)
        d.bbox.center.position.y = float(cy)
        d.bbox.center.theta = 0.0
        d.bbox.size_x = float(bbox_w)
        d.bbox.size_y = float(bbox_h)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis = ObjectHypothesis()
        hyp.hypothesis.class_id = class_name
        hyp.hypothesis.score = 1.0
        hyp.pose.pose = Pose()
        hyp.pose.pose.position.x = float(x_cam)
        hyp.pose.pose.position.y = float(y_cam)
        hyp.pose.pose.position.z = float(z_cam)
        hyp.pose.pose.orientation.w = 1.0
        # Large covariance — line poses are unreliable.
        hyp.pose.covariance = [0.0] * 36
        hyp.pose.covariance[0]  = 1.0
        hyp.pose.covariance[7]  = 1.0
        hyp.pose.covariance[14] = 1.0
        hyp.pose.covariance[21] = 1e6
        hyp.pose.covariance[28] = 1e6
        hyp.pose.covariance[35] = 1e6

        d.results.append(hyp)
        return d

    def build_detection(self, det, class_name: str, stamp) -> Detection2D:
        cx, cy, bbox_w, bbox_h = det

        # Range from apparent height (same as real bridge).
        z_cam = (PILLAR_HEIGHT_M * FY) / bbox_h
        x_cam = (cx - CX) * z_cam / FX
        y_cam = (cy - CY) * z_cam / FY

        d = Detection2D()
        d.header.stamp = stamp
        d.header.frame_id = FRAME_ID

        d.bbox = BoundingBox2D()
        d.bbox.center.position.x = float(cx)
        d.bbox.center.position.y = float(cy)
        d.bbox.center.theta = 0.0
        d.bbox.size_x = float(bbox_w)
        d.bbox.size_y = float(bbox_h)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis = ObjectHypothesis()
        hyp.hypothesis.class_id = class_name
        hyp.hypothesis.score = 1.0            # sim is ground-truth reliable
        hyp.pose.pose = Pose()
        hyp.pose.pose.position.x = float(x_cam)
        hyp.pose.pose.position.y = float(y_cam)
        hyp.pose.pose.position.z = float(z_cam)
        hyp.pose.pose.orientation.w = 1.0

        # Same coarse covariance model as the real bridge.
        var_z  = max(1e-4, (0.05 * z_cam) ** 2)
        var_xy = max(1e-4, (0.03 * z_cam) ** 2)
        hyp.pose.covariance = [0.0] * 36
        hyp.pose.covariance[0]  = var_xy
        hyp.pose.covariance[7]  = var_xy
        hyp.pose.covariance[14] = var_z
        hyp.pose.covariance[21] = 1e6
        hyp.pose.covariance[28] = 1e6
        hyp.pose.covariance[35] = 1e6

        d.results.append(hyp)
        return d


def main():
    rclpy.init()
    node = SimLimelightBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
