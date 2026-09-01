"""Detect red/green pillars in the camera image and publish their positions.

Outputs:
  /perception/pillars       visualization_msgs/MarkerArray  (RViz overlay)
  /perception/pillar_points sensor_msgs/PointCloud2         (Nav2 obstacle layer)

Pipeline:
  1. Threshold the image in HSV for red and green separately.
  2. Find contours, filter by min area and aspect ratio.
  3. For each contour, take the bounding-box bottom-center pixel — the point
     where the pillar meets the ground.
  4. Ray from camera center through that pixel, intersect the z=0 plane in
     base_link — gives (x, y) of the pillar base.
  5. Publish the resulting points labeled by color.

Assumes /camera_info is being published (needed for camera intrinsics) and
that TF for camera_optical_frame -> base_link is available.
"""
import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

from cv_bridge import CvBridge

import tf2_ros
from tf2_geometry_msgs import do_transform_point


# HSV thresholds. Red wraps around 0/180 so we OR two ranges.
RED_LO_1 = np.array([0, 120, 60])
RED_HI_1 = np.array([10, 255, 255])
RED_LO_2 = np.array([170, 120, 60])
RED_HI_2 = np.array([180, 255, 255])

GREEN_LO = np.array([40, 80, 40])
GREEN_HI = np.array([85, 255, 255])

MIN_AREA_PX = 80      # ignore contours below this — noise
MIN_ASPECT_H_W = 1.2  # pillars are taller than wide

PILLAR_HEIGHT_M = 0.10  # official WRO pillar height


class PillarDetector(Node):
    def __init__(self):
        super().__init__('pillar_detector')

        self.bridge = CvBridge()
        self.K = None  # 3x3 intrinsic matrix, filled from /camera_info
        self.camera_frame = None

        # /camera in sim uses BEST_EFFORT sensor QoS.
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(CameraInfo, '/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera', self.on_image, sensor_qos)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/perception/pillars', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/perception/pillar_points', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('pillar_detector up')

    def on_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.camera_frame = msg.header.frame_id
            self.get_logger().info(f'camera_info received, frame={self.camera_frame}')

    def on_image(self, msg: Image):
        if self.K is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        detections = []  # list of (color, u_px, v_px)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsv, RED_LO_1, RED_HI_1) | \
                   cv2.inRange(hsv, RED_LO_2, RED_HI_2)
        green_mask = cv2.inRange(hsv, GREEN_LO, GREEN_HI)

        for color, mask in (('red', red_mask), ('green', green_mask)):
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < MIN_AREA_PX:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                if h < MIN_ASPECT_H_W * w:
                    continue
                # Bottom-center of the bounding box = pillar base in image.
                u = x + w / 2.0
                v = y + h
                detections.append((color, u, v))

        # Project each pixel to the ground plane (z=0 in base_link).
        points_base = []  # list of (color, x, y)
        for color, u, v in detections:
            p = self._project_to_ground(u, v, msg.header.stamp)
            if p is not None:
                points_base.append((color, p[0], p[1]))

        self._publish(points_base, msg.header.stamp)

    def _project_to_ground(self, u, v, stamp):
        """Pixel -> (x, y) in base_link, assuming z=0 for the pillar base."""
        # Ray direction in camera optical frame (Z forward, X right, Y down).
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])

        # Transform a point on that ray + the camera origin into base_link,
        # then find where the ray crosses z=0.
        # Simplest: transform two points and interpolate.
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', self.camera_frame, rclpy.time.Time())
        except tf2_ros.LookupException:
            return None
        except tf2_ros.ExtrapolationException:
            return None

        # Point at camera origin and one 1 m along the ray, both in camera frame.
        p0 = PointStamped(header=Header(frame_id=self.camera_frame))
        p0.point.x, p0.point.y, p0.point.z = 0.0, 0.0, 0.0
        p1 = PointStamped(header=Header(frame_id=self.camera_frame))
        p1.point.x, p1.point.y, p1.point.z = float(ray[0]), float(ray[1]), float(ray[2])

        p0b = do_transform_point(p0, tf).point
        p1b = do_transform_point(p1, tf).point

        # Parametrize ray in base_link and solve for z = 0.
        dz = p1b.z - p0b.z
        if abs(dz) < 1e-6:
            return None
        t = -p0b.z / dz
        if t <= 0:
            return None  # ground is behind the camera
        x = p0b.x + t * (p1b.x - p0b.x)
        y = p0b.y + t * (p1b.y - p0b.y)

        # Sanity: pillars beyond 3 m are probably noise given our camera FOV.
        if math.hypot(x, y) > 3.0:
            return None
        return (x, y)

    def _publish(self, points, stamp):
        # Markers
        marr = MarkerArray()
        for i, (color, x, y) in enumerate(points):
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = stamp
            m.ns = 'pillars'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = PILLAR_HEIGHT_M / 2
            m.pose.orientation.w = 1.0
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = PILLAR_HEIGHT_M
            if color == 'red':
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.15, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.9, 0.3, 0.9
            m.lifetime.sec = 0
            m.lifetime.nanosec = 300_000_000  # 0.3 s so stale detections disappear
            marr.markers.append(m)
        self.marker_pub.publish(marr)

        # PointCloud2 with one point per pillar at half-height (visible in costmap).
        cloud_points = [
            (float(x), float(y), PILLAR_HEIGHT_M / 2) for _, x, y in points
        ]
        header = Header(frame_id='base_link', stamp=stamp)
        self.cloud_pub.publish(self._make_cloud(cloud_points, header))

    @staticmethod
    def _make_cloud(points, header):
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        buf = np.array(points, dtype=np.float32).tobytes() if points else b''
        cloud.data = buf
        return cloud


def main():
    rclpy.init()
    node = PillarDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
