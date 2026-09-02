#!/usr/bin/env python3
"""ROS 2 bridge node for Limelight 3 running an SSD MobileNet detector.

Data flow:
    Limelight camera -> WebSocket JSON -> parse -> per-detection distance
        from bbox pixel-height + known real-world pillar height -> publish
        vision_msgs/Detection2DArray on /limelight/detections.

Assumes the Limelight is running a neural-detector pipeline whose classes are
labeled "red_pillar" and "green_pillar" (or whatever labels.txt contains).
The label strings are passed through verbatim as Detection2D result class_id;
downstream code decides what a label means.

Pose semantics:
    Per-detection pose is populated in the CAMERA OPTICAL frame
    (X right, Y down, Z forward). URDF must publish TF from base_link to
    the frame configured via the `frame_id` parameter. Downstream nodes
    should tf2 the pose into whatever frame they need.
"""

import math
import threading
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Pose2D,
    Point2D,
)

from WRORobot import limelight
from WRORobot import limelightresults


class LimelightBridge(Node):
    def __init__(self) -> None:
        super().__init__('limelight_bridge')

        # --- parameters -----------------------------------------------------
        # Camera intrinsics. Defaults are the NOMINAL LL3 values computed from
        # its 62.5x48.9 deg FOV at the native 640x480 resolution. For accurate
        # 3D pose, run task #15 (intrinsic calibration) and override these.
        self.declare_parameter('fx', 527.05)
        self.declare_parameter('fy', 528.13)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # Real-world height of the target used for range-from-bbox-height.
        # WRO Future Engineers red/green pillars are 10 cm.
        self.declare_parameter('pillar_height_m', 0.10)

        # Discovery + connection.
        # If discovery fails and `fallback_ip` is set, connect there directly.
        self.declare_parameter('fallback_ip', '')
        self.declare_parameter('discovery_timeout_s', 2.0)
        self.declare_parameter('reconnect_period_s', 2.0)

        # Publish rate for the outgoing Detection2DArray. The WebSocket callback
        # updates latest_results asynchronously; the timer polls that at this
        # rate. Set >= LL frame rate to avoid dropping.
        self.declare_parameter('publish_rate_hz', 30.0)

        # TF frame that per-detection poses are expressed in. Matches the
        # camera_optical_frame published by robot_state_publisher from
        # WRORobot/urdf/wro_bot.urdf.xacro (attached to camera_link, which is
        # the Limelight's physical mount on the chassis).
        self.declare_parameter('frame_id', 'camera_optical_frame')

        # Minimum detection confidence to publish. Below this we discard.
        self.declare_parameter('min_confidence', 0.30)

        self._fx = float(self.get_parameter('fx').value)
        self._fy = float(self.get_parameter('fy').value)
        self._cx = float(self.get_parameter('cx').value)
        self._cy = float(self.get_parameter('cy').value)
        self._image_width = int(self.get_parameter('image_width').value)
        self._image_height = int(self.get_parameter('image_height').value)
        self._pillar_height_m = float(self.get_parameter('pillar_height_m').value)
        self._fallback_ip = str(self.get_parameter('fallback_ip').value)
        self._discovery_timeout_s = float(self.get_parameter('discovery_timeout_s').value)
        self._reconnect_period_s = float(self.get_parameter('reconnect_period_s').value)
        self._publish_period_s = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._min_confidence = float(self.get_parameter('min_confidence').value)

        # --- publishers -----------------------------------------------------
        self._pub = self.create_publisher(
            Detection2DArray, '/limelight/detections', 10
        )

        # --- limelight connection state -------------------------------------
        self._ll: Optional[limelight.Limelight] = None
        self._ll_lock = threading.Lock()
        self._connect_and_start()

        # --- polling timer --------------------------------------------------
        # The websocket receiver in limelight.py stores frames into
        # self._ll.latest_results. We poll and publish at fixed rate. This
        # decouples ROS publishing from the websocket thread and avoids
        # rclpy thread-safety questions.
        self._timer = self.create_timer(self._publish_period_s, self._on_timer)

        # Reconnect watchdog: if ll is None (never connected or dropped),
        # try to re-establish.
        self._reconnect_timer = self.create_timer(
            self._reconnect_period_s, self._reconnect_if_needed
        )

    # ------------------------------------------------------------------ setup

    def _connect_and_start(self) -> None:
        with self._ll_lock:
            if self._ll is not None:
                return
            address = self._discover_or_fallback()
            if not address:
                self.get_logger().warn(
                    'No Limelight found via discovery and no fallback_ip set; '
                    'will keep retrying every %.1fs' % self._reconnect_period_s
                )
                return
            try:
                ll = limelight.Limelight(address)
                ll.enable_websocket()
                self._ll = ll
                self.get_logger().info(f'Connected to Limelight at {address}')
            except Exception as exc:
                self.get_logger().error(f'Failed to connect to {address}: {exc}')
                self._ll = None

    def _discover_or_fallback(self) -> Optional[str]:
        try:
            found = limelight.discover_limelights(
                timeout=self._discovery_timeout_s
            )
        except Exception as exc:
            self.get_logger().warn(f'Discovery raised: {exc}')
            found = []
        if found:
            return found[0]
        if self._fallback_ip:
            self.get_logger().info(
                f'Discovery empty; using fallback_ip={self._fallback_ip}'
            )
            return self._fallback_ip
        return None

    def _reconnect_if_needed(self) -> None:
        with self._ll_lock:
            if self._ll is not None:
                return
        self._connect_and_start()

    # -------------------------------------------------------------- main loop

    def _on_timer(self) -> None:
        with self._ll_lock:
            ll = self._ll
        if ll is None:
            return
        raw = ll.get_latest_results()
        if raw is None:
            return
        try:
            parsed = limelightresults.parse_results(raw)
        except Exception as exc:
            self.get_logger().warn(f'parse_results failed: {exc}', throttle_duration_sec=2.0)
            return
        if parsed is None or not parsed.detectorResults:
            # Publish an empty array so consumers see a fresh "no targets" heartbeat.
            self._pub.publish(self._make_empty_msg())
            return

        msg = Detection2DArray()
        msg.header.stamp = self._stamp_from_result(parsed)
        msg.header.frame_id = self._frame_id

        for det in parsed.detectorResults:
            if det.confidence < self._min_confidence:
                continue
            d2d = self._make_detection(det, msg.header.stamp)
            if d2d is not None:
                msg.detections.append(d2d)

        self._pub.publish(msg)

    def _make_empty_msg(self) -> Detection2DArray:
        m = Detection2DArray()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._frame_id
        return m

    def _stamp_from_result(self, parsed) -> TimeMsg:
        # LL reports capture_latency (ms) + targeting_latency (ms). Back-date
        # the current time by that total to approximate the true capture
        # instant on this host's clock. Not synchronized to LL's own clock
        # (would need PTP for that), but good enough for planners.
        total_latency_ms = float(parsed.capture_latency or 0.0) + \
                           float(parsed.targeting_latency or 0.0) + \
                           float(parsed.parse_latency or 0.0)
        now = self.get_clock().now()
        stamp = (now - Duration(seconds=total_latency_ms / 1000.0)).to_msg()
        return stamp

    # -------------------------------------------------------- one detection

    def _make_detection(self, det, stamp) -> Optional[Detection2D]:
        # LL's DetectorResult.points is a list of [x, y] corner coords in
        # image pixel space. Compute bbox size from min/max.
        try:
            xs = [float(p[0]) for p in det.points]
            ys = [float(p[1]) for p in det.points]
        except (TypeError, ValueError, IndexError):
            self.get_logger().warn(
                'DetectorResult.points malformed; skipping detection',
                throttle_duration_sec=2.0,
            )
            return None

        if not xs or not ys:
            return None

        bbox_w_px = max(xs) - min(xs)
        bbox_h_px = max(ys) - min(ys)
        if bbox_w_px <= 0 or bbox_h_px <= 0:
            return None

        # Prefer LL-provided center; fall back to bbox midpoint.
        u = float(det.target_x_pixels) if det.target_x_pixels is not None \
            else (min(xs) + bbox_w_px / 2.0)
        v = float(det.target_y_pixels) if det.target_y_pixels is not None \
            else (min(ys) + bbox_h_px / 2.0)

        # Range from apparent height: same size gives smaller pixel height
        # the further away it is. Only valid when the sensor sees the FULL
        # height of the target (bbox not clipped by frame edge).
        # z = (real_height_m * fy) / bbox_height_px
        z = (self._pillar_height_m * self._fy) / bbox_h_px

        # Project the center pixel back to a 3D ray at that depth.
        # Camera optical frame: X right, Y down, Z forward.
        x_cam = (u - self._cx) * z / self._fx
        y_cam = (v - self._cy) * z / self._fy
        z_cam = z

        d2d = Detection2D()
        d2d.header.stamp = stamp
        d2d.header.frame_id = self._frame_id

        d2d.bbox = BoundingBox2D()
        d2d.bbox.center = Pose2D()
        d2d.bbox.center.position = Point2D()
        d2d.bbox.center.position.x = u
        d2d.bbox.center.position.y = v
        d2d.bbox.center.theta = 0.0
        d2d.bbox.size_x = bbox_w_px
        d2d.bbox.size_y = bbox_h_px

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis = ObjectHypothesis()
        hyp.hypothesis.class_id = str(det.class_name)
        hyp.hypothesis.score = float(det.confidence)
        hyp.pose.pose = Pose()
        hyp.pose.pose.position.x = x_cam
        hyp.pose.pose.position.y = y_cam
        hyp.pose.pose.position.z = z_cam
        # Identity orientation — we don't know the pillar's yaw, only its
        # position. Consumers should treat orientation as unspecified.
        hyp.pose.pose.orientation.w = 1.0

        # Populate a coarse covariance so downstream fusion knows how much
        # to trust this. Range uncertainty grows with distance^2 (bbox
        # discretization error). Rough model, not calibrated.
        var_z = max(1e-4, (0.05 * z_cam) ** 2)
        var_xy = max(1e-4, (0.03 * z_cam) ** 2)
        hyp.pose.covariance = [0.0] * 36
        hyp.pose.covariance[0]  = var_xy      # X
        hyp.pose.covariance[7]  = var_xy      # Y
        hyp.pose.covariance[14] = var_z       # Z
        hyp.pose.covariance[21] = 1e6         # roll — unknown
        hyp.pose.covariance[28] = 1e6         # pitch — unknown
        hyp.pose.covariance[35] = 1e6         # yaw — unknown

        d2d.results.append(hyp)
        d2d.id = str(det.class_name)
        return d2d

    # ---------------------------------------------------------- cleanup

    def destroy_node(self):
        with self._ll_lock:
            if self._ll is not None:
                try:
                    self._ll.disable_websocket()
                except Exception:
                    pass
                self._ll = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LimelightBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
