
#!/usr/bin/env python3

"""
ROS 2 bridge node for Limelight 3A.

Limelight pipelines:

    Pipeline 0:
        Neural Detector
        Detects:
            Red_Obstacle
            Green_Obstacle

    Pipeline 1:
        SnapScript
        Detects:
            Orange_Line

The SnapScript sends PythonOut:

    [
        detected,
        turnAngle,
        targetX,
        targetY,
        width,
        height,
        pixelError,
        0
    ]

ROS output:

    /limelight/detections

Message type:

    vision_msgs/Detection2DArray

Camera resolution:

    1280 x 960

Camera optical frame:

    X = right
    Y = down
    Z = forward
"""

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

        # ================================================================
        # CAMERA SETTINGS
        # ================================================================

        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 960)

        # Nominal intrinsics scaled from 640x480.
        #
        # Replace these with calibrated values if you have them.
        self.declare_parameter('fx', 1054.10)
        self.declare_parameter('fy', 1056.26)
        self.declare_parameter('cx', 640.0)
        self.declare_parameter('cy', 480.0)

        # Physical height of the red/green obstacles.
        self.declare_parameter('pillar_height_m', 0.10)

        # ================================================================
        # LIMELIGHT CONNECTION
        # ================================================================

        self.declare_parameter('fallback_ip', '')
        self.declare_parameter('discovery_timeout_s', 2.0)
        self.declare_parameter('reconnect_period_s', 2.0)

        # ================================================================
        # ROS SETTINGS
        # ================================================================

        self.declare_parameter('publish_rate_hz', 30.0)

        self.declare_parameter(
            'frame_id',
            'camera_optical_frame'
        )

        self.declare_parameter(
            'min_confidence',
            0.0
        )

        # ================================================================
        # PIPELINE SETTINGS
        # ================================================================

        # Your Limelight configuration:
        #
        #   0 = Neural Detector
        #   1 = SnapScript
        #
        self.neural_pipeline = 0
        self.snapscript_pipeline = 1

        # How long each pipeline remains active.
        #
        # 0.30 seconds is a reasonable starting point.
        #
        # The sequence becomes:
        #
        #   pipeline 0 for 0.30 sec
        #   pipeline 1 for 0.30 sec
        #   pipeline 0 for 0.30 sec
        #   pipeline 1 for 0.30 sec
        #
        self.pipeline_switch_period_s = 0.30

        self._current_pipeline = self.neural_pipeline

        self._last_pipeline_switch_time = (
            self.get_clock().now()
        )

        # ================================================================
        # READ PARAMETERS
        # ================================================================

        self._image_width = int(
            self.get_parameter('image_width').value
        )

        self._image_height = int(
            self.get_parameter('image_height').value
        )

        self._fx = float(
            self.get_parameter('fx').value
        )

        self._fy = float(
            self.get_parameter('fy').value
        )

        self._cx = float(
            self.get_parameter('cx').value
        )

        self._cy = float(
            self.get_parameter('cy').value
        )

        self._pillar_height_m = float(
            self.get_parameter('pillar_height_m').value
        )

        self._fallback_ip = str(
            self.get_parameter('fallback_ip').value
        )

        self._discovery_timeout_s = float(
            self.get_parameter('discovery_timeout_s').value
        )

        self._reconnect_period_s = float(
            self.get_parameter('reconnect_period_s').value
        )

        publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )

        self._publish_period_s = (
            1.0 / publish_rate_hz
        )

        self._frame_id = str(
            self.get_parameter('frame_id').value
        )

        self._min_confidence = float(
            self.get_parameter('min_confidence').value
        )

        # ================================================================
        # ROS PUBLISHER
        # ================================================================

        self._pub = self.create_publisher(
            Detection2DArray,
            '/limelight/detections',
            10
        )

        # ================================================================
        # LIMELIGHT STATE
        # ================================================================

        self._ll: Optional[
            limelight.Limelight
        ] = None

        self._ll_lock = threading.Lock()

        # ================================================================
        # CONNECT
        # ================================================================

        self._connect_and_start()

        # ================================================================
        # MAIN TIMER
        # ================================================================

        self._timer = self.create_timer(
            self._publish_period_s,
            self._on_timer
        )

        # ================================================================
        # RECONNECT TIMER
        # ================================================================

        self._reconnect_timer = self.create_timer(
            self._reconnect_period_s,
            self._reconnect_if_needed
        )

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'Limelight Bridge Started'
        )

        self.get_logger().info(
            'Pipeline 0 = Neural Detector'
        )

        self.get_logger().info(
            'Pipeline 1 = SnapScript / Orange Line'
        )

        self.get_logger().info(
            'Camera = 1280 x 960'
        )

        self.get_logger().info(
            '================================================'
        )

    # ====================================================================
    # CONNECTION
    # ====================================================================

    def _connect_and_start(self) -> None:

        with self._ll_lock:

            if self._ll is not None:
                return

            address = self._discover_or_fallback()

            if not address:

                self.get_logger().warn(
                    'No Limelight found. '
                    'Will retry.'
                )

                return

            try:

                ll = limelight.Limelight(address)

                ll.enable_websocket()

                self._ll = ll

                # Start on Neural Detector.
                try:

                    ll.pipeline_switch(
                        self.neural_pipeline
                    )

                    self._current_pipeline = (
                        self.neural_pipeline
                    )

                    self._last_pipeline_switch_time = (
                        self.get_clock().now()
                    )

                    self.get_logger().info(
                        'Initial pipeline = 0 '
                        '(Neural Detector)'
                    )

                except Exception as exc:

                    self.get_logger().warn(
                        f'Could not select initial pipeline: '
                        f'{exc}'
                    )

                self.get_logger().info(
                    f'Connected to Limelight at {address}'
                )

            except Exception as exc:

                self.get_logger().error(
                    f'Failed to connect to Limelight: {exc}'
                )

                self._ll = None

    def _discover_or_fallback(
        self
    ) -> Optional[str]:

        try:

            found = (
                limelight.discover_limelights(
                    timeout=self._discovery_timeout_s
                )
            )

        except Exception as exc:

            self.get_logger().warn(
                f'Discovery raised: {exc}'
            )

            found = []

        if found:

            return found[0]

        if self._fallback_ip:

            self.get_logger().info(
                f'Using fallback IP: '
                f'{self._fallback_ip}'
            )

            return self._fallback_ip

        return None

    def _reconnect_if_needed(self) -> None:

        with self._ll_lock:

            if self._ll is not None:
                return

        self._connect_and_start()

    # ====================================================================
    # PIPELINE SWITCHING
    # ====================================================================

    def _maybe_switch_pipeline(self) -> None:

        with self._ll_lock:
            ll = self._ll

        if ll is None:
            return

        now = self.get_clock().now()

        elapsed = (
            now
            - self._last_pipeline_switch_time
        ).nanoseconds / 1e9

        if elapsed < self.pipeline_switch_period_s:

            return

        # ------------------------------------------------------------
        # Neural -> SnapScript
        # ------------------------------------------------------------

        if (
            self._current_pipeline
            == self.neural_pipeline
        ):

            new_pipeline = (
                self.snapscript_pipeline
            )


        # ------------------------------------------------------------
        # SnapScript -> Neural
        # ------------------------------------------------------------

        else:

            new_pipeline = (
                self.neural_pipeline
            )


        try:

            ll.pipeline_switch(
                new_pipeline
            )

            self._current_pipeline = (
                new_pipeline
            )

            self._last_pipeline_switch_time = now

        except Exception as exc:

            self.get_logger().error(
                f'Pipeline switch failed: {exc}'
            )

    # ====================================================================
    # MAIN LOOP
    # ====================================================================

    def _on_timer(self) -> None:

        with self._ll_lock:
            ll = self._ll

        if ll is None:
            return

        # ============================================================
        # GET RAW LIMELIGHT DATA
        # ============================================================

        raw = ll.get_latest_results()

        if raw is None:

            return


        # ============================================================
        # PARSE GENERAL RESULT
        # ============================================================

        try:

            parsed = (
                limelightresults.parse_results(
                    raw
                )
            )

        except Exception as exc:

            self.get_logger().warn(
                f'parse_results failed: {exc}',
                throttle_duration_sec=2.0
            )

            self._maybe_switch_pipeline()

            return

        if parsed is None:

            self._maybe_switch_pipeline()

            return

        # ============================================================
        # CREATE ROS MESSAGE
        # ============================================================

        msg = Detection2DArray()

        msg.header.stamp = (
            self._stamp_from_result(parsed)
        )

        msg.header.frame_id = (
            self._frame_id
        )

        # ============================================================
        # PIPELINE 0
        # NEURAL DETECTOR
        # ============================================================

        if (
            self._current_pipeline
            == self.neural_pipeline
        ):

            self._process_neural_pipeline(
                parsed,
                msg
            )

        # ============================================================
        # PIPELINE 1
        # SNAP SCRIPT
        # ============================================================

        elif (
            self._current_pipeline
            == self.snapscript_pipeline
        ):

            self._process_snapscript_pipeline(
                raw,
                msg
            )

        # ============================================================
        # PUBLISH
        # ============================================================


        self._pub.publish(msg)

        # ============================================================
        # POSSIBLY SWITCH
        # ============================================================

        self._maybe_switch_pipeline()

    # ====================================================================
    # NEURAL PIPELINE
    # ====================================================================

    def _process_neural_pipeline(
        self,
        parsed,
        msg
    ) -> None:

        detector_results = (
            parsed.detectorResults
        )

        for det in detector_results:

            if (
                det.confidence
                < self._min_confidence
            ):

                continue

            detection = (
                self._make_neural_detection(
                    det,
                    msg.header.stamp
                )
            )

            if detection is not None:

                msg.detections.append(
                    detection
                )

    # ====================================================================
    # SNAP SCRIPT PIPELINE
    # ====================================================================

    def _process_snapscript_pipeline(
        self,
        raw,
        msg
    ) -> None:

        # ------------------------------------------------------------
        # IMPORTANT:
        #
        # GeneralResult does NOT expose PythonOut as an attribute in
        # your version of limelightresults.py.
        #
        # Therefore we read it directly from the raw JSON dictionary.
        # ------------------------------------------------------------

        if not isinstance(raw, dict):

            self.get_logger().error(
                'SnapScript pipeline returned '
                'non-dictionary raw data.'
            )

            return

        python_out = raw.get(
            'PythonOut',
            []
        )


        # ------------------------------------------------------------
        # Limelight can sometimes give us [].
        # ------------------------------------------------------------

        if (
            python_out is None
            or len(python_out) == 0
        ):


            return

        # ------------------------------------------------------------
        # Some Limelight versions may wrap the values.
        #
        # Handle both:
        #
        #   [1, 2, 3, ...]
        #
        # and:
        #
        #   [[1, 2, 3, ...]]
        # ------------------------------------------------------------

        if (
            isinstance(python_out, list)
            and len(python_out) == 1
            and isinstance(
                python_out[0],
                list
            )
        ):

            python_out = python_out[0]

        # ------------------------------------------------------------
        # We expect 8 values.
        # ------------------------------------------------------------

        if len(python_out) < 8:

            self.get_logger().error(
                f'PythonOut has '
                f'{len(python_out)} values. '
                f'Expected at least 8.'
            )

            return


        # ------------------------------------------------------------
        # Your SnapScript format:
        #
        # [0] detected
        # [1] turnAngle
        # [2] targetX
        # [3] targetY
        # [4] width
        # [5] height
        # [6] pixelError
        # [7] unused
        # ------------------------------------------------------------

        try:

            detected = float(
                python_out[0]
            )

            turn_angle = float(
                python_out[1]
            )

            target_x = float(
                python_out[2]
            )

            target_y = float(
                python_out[3]
            )

            bbox_w = float(
                python_out[4]
            )

            bbox_h = float(
                python_out[5]
            )

            pixel_error = float(
                python_out[6]
            )

        except (
            TypeError,
            ValueError,
            IndexError
        ) as exc:

            self.get_logger().error(
                f'Could not parse PythonOut: '
                f'{exc}'
            )

            return


        # ------------------------------------------------------------
        # No Orange Line.
        # ------------------------------------------------------------

        if detected <= 0:

            return

        # ------------------------------------------------------------
        # Validate bbox.
        # ------------------------------------------------------------

        if (
            bbox_w <= 0
            or bbox_h <= 0
        ):

            return

        # ------------------------------------------------------------
        # Create Orange Line detection.
        # ------------------------------------------------------------

        detection = (
            self._make_orange_line_detection(
                target_x,
                target_y,
                bbox_w,
                bbox_h,
                turn_angle,
                pixel_error,
                msg.header.stamp
            )
        )

        if detection is not None:

            msg.detections.append(
                detection
            )

    # ====================================================================
    # NEURAL DETECTION -> ROS
    # ====================================================================

    def _make_neural_detection(
        self,
        det,
        stamp
    ) -> Optional[Detection2D]:

        # ------------------------------------------------------------
        # Get bounding-box corner points.
        # ------------------------------------------------------------

        try:

            xs = [
                float(p[0])
                for p in det.points
            ]

            ys = [
                float(p[1])
                for p in det.points
            ]

        except (
            TypeError,
            ValueError,
            IndexError
        ):

            self.get_logger().error(
                'Malformed neural detector points.'
            )

            return None

        if not xs or not ys:

            self.get_logger().error(
                f'Neural detector has no points: '
                f'{det.points}'
            )

            return None

        # ------------------------------------------------------------
        # Calculate bbox.
        # ------------------------------------------------------------

        bbox_w_px = (
            max(xs) - min(xs)
        )

        bbox_h_px = (
            max(ys) - min(ys)
        )

        if (
            bbox_w_px <= 0
            or bbox_h_px <= 0
        ):

            self.get_logger().error(
                f'Invalid neural bbox: '
                f'w={bbox_w_px}, '
                f'h={bbox_h_px}'
            )

            return None

        # ------------------------------------------------------------
        # Center.
        # ------------------------------------------------------------

        if det.target_x_pixels is not None:

            u = float(
                det.target_x_pixels
            )

        else:

            u = (
                min(xs)
                + bbox_w_px / 2.0
            )

        if det.target_y_pixels is not None:

            v = float(
                det.target_y_pixels
            )

        else:

            v = (
                min(ys)
                + bbox_h_px / 2.0
            )

        # ------------------------------------------------------------
        # Distance from known obstacle height.
        # ------------------------------------------------------------

        z = (
            self._pillar_height_m
            * self._fy
            / bbox_h_px
        )

        # ------------------------------------------------------------
        # Camera optical coordinates.
        # ------------------------------------------------------------

        x_cam = (
            (u - self._cx)
            * z
            / self._fx
        )

        y_cam = (
            (v - self._cy)
            * z
            / self._fy
        )

        z_cam = z

        # ------------------------------------------------------------
        # Detection2D.
        # ------------------------------------------------------------

        d2d = Detection2D()

        d2d.header.stamp = stamp
        d2d.header.frame_id = (
            self._frame_id
        )

        # ------------------------------------------------------------
        # Bounding box.
        # ------------------------------------------------------------

        d2d.bbox = BoundingBox2D()

        d2d.bbox.center = Pose2D()

        d2d.bbox.center.position = Point2D()

        d2d.bbox.center.position.x = u
        d2d.bbox.center.position.y = v
        d2d.bbox.center.theta = 0.0

        d2d.bbox.size_x = bbox_w_px
        d2d.bbox.size_y = bbox_h_px

        # ------------------------------------------------------------
        # Hypothesis.
        # ------------------------------------------------------------

        hyp = ObjectHypothesisWithPose()

        hyp.hypothesis = ObjectHypothesis()

        hyp.hypothesis.class_id = str(
            det.class_name
        )

        hyp.hypothesis.score = float(
            det.confidence
        )

        # ------------------------------------------------------------
        # Pose.
        # ------------------------------------------------------------

        hyp.pose.pose = Pose()

        hyp.pose.pose.position.x = (
            x_cam
        )

        hyp.pose.pose.position.y = (
            y_cam
        )

        hyp.pose.pose.position.z = (
            z_cam
        )

        # Unknown orientation.
        hyp.pose.pose.orientation.w = 1.0

        # ------------------------------------------------------------
        # Covariance.
        # ------------------------------------------------------------

        var_z = max(
            1e-4,
            (0.05 * z_cam) ** 2
        )

        var_xy = max(
            1e-4,
            (0.03 * z_cam) ** 2
        )

        hyp.pose.covariance = [0.0] * 36

        hyp.pose.covariance[0] = var_xy
        hyp.pose.covariance[7] = var_xy
        hyp.pose.covariance[14] = var_z

        # Unknown orientation.
        hyp.pose.covariance[21] = 1e6
        hyp.pose.covariance[28] = 1e6
        hyp.pose.covariance[35] = 1e6

        d2d.results.append(
            hyp
        )

        d2d.id = str(
            det.class_name
        )

        return d2d

    # ====================================================================
    # ORANGE LINE -> ROS
    # ====================================================================

    def _make_orange_line_detection(
        self,
        target_x,
        target_y,
        bbox_w,
        bbox_h,
        turn_angle,
        pixel_error,
        stamp
    ) -> Optional[Detection2D]:

        d2d = Detection2D()

        d2d.header.stamp = stamp

        d2d.header.frame_id = (
            self._frame_id
        )

        # ------------------------------------------------------------
        # Bounding box.
        # ------------------------------------------------------------

        d2d.bbox = BoundingBox2D()

        d2d.bbox.center = Pose2D()

        d2d.bbox.center.position = Point2D()

        d2d.bbox.center.position.x = (
            target_x
        )

        d2d.bbox.center.position.y = (
            target_y
        )

        d2d.bbox.center.theta = 0.0

        d2d.bbox.size_x = bbox_w
        d2d.bbox.size_y = bbox_h

        # ------------------------------------------------------------
        # Hypothesis.
        # ------------------------------------------------------------

        hyp = ObjectHypothesisWithPose()

        hyp.hypothesis = ObjectHypothesis()

        hyp.hypothesis.class_id = (
            'Orange_Line'
        )

        # SnapScript does not provide a confidence.
        hyp.hypothesis.score = 1.0

        # ------------------------------------------------------------
        # We do NOT know the physical distance to the Orange Line.
        #
        # Therefore don't pretend that x/y/z are metric coordinates.
        #
        # Instead, provide the normalized camera ray:
        #
        #   x = horizontal ray
        #   y = vertical ray
        #   z = 1
        #
        # This gives downstream code a direction to the Orange Line.
        # ------------------------------------------------------------

        ray_x = (
            target_x - self._cx
        ) / self._fx

        ray_y = (
            target_y - self._cy
        ) / self._fy

        hyp.pose.pose = Pose()

        hyp.pose.pose.position.x = (
            ray_x
        )

        hyp.pose.pose.position.y = (
            ray_y
        )

        hyp.pose.pose.position.z = 1.0

        # Unknown orientation.
        hyp.pose.pose.orientation.w = 1.0

        # Position is not metric.
        hyp.pose.covariance = [0.0] * 36

        hyp.pose.covariance[0] = 1e6
        hyp.pose.covariance[7] = 1e6
        hyp.pose.covariance[14] = 1e6

        hyp.pose.covariance[21] = 1e6
        hyp.pose.covariance[28] = 1e6
        hyp.pose.covariance[35] = 1e6

        d2d.results.append(
            hyp
        )

        d2d.id = 'Orange_Line'

        return d2d

    # ====================================================================
    # TIMESTAMP
    # ====================================================================

    def _stamp_from_result(
        self,
        parsed
    ) -> TimeMsg:

        capture_latency = float(
            getattr(
                parsed,
                'capture_latency',
                0.0
            ) or 0.0
        )

        targeting_latency = float(
            getattr(
                parsed,
                'targeting_latency',
                0.0
            ) or 0.0
        )

        parse_latency = float(
            getattr(
                parsed,
                'parse_latency',
                0.0
            ) or 0.0
        )

        total_latency_ms = (
            capture_latency
            + targeting_latency
            + parse_latency
        )

        now = self.get_clock().now()

        stamp = (
            now
            - Duration(
                seconds=(
                    total_latency_ms / 1000.0
                )
            )
        ).to_msg()

        return stamp

    # ====================================================================
    # CLEANUP
    # ====================================================================

    def destroy_node(self):

        with self._ll_lock:

            if self._ll is not None:

                try:

                    self._ll.disable_websocket()

                except Exception:

                    pass

                self._ll = None

        super().destroy_node()


# ========================================================================
# MAIN
# ========================================================================

def main(args=None):

    rclpy.init(args=args)

    node = LimelightBridge()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
