"""Unified WRO Future Engineers mission node.

Explicit state machine:

    NOT_READY --> WAITING_FOR_START --> DRIVING --+--> PARKING --> DONE
                                                  \-------------> DONE  (open)

Auto-decides the endgame at the finish:
  - If any red/green pillar was ever seen during the run  ->  PARKING
  - Otherwise                                             ->  DONE (just stop)

Perception is factored out:
  - lidar_perception.LidarPerception    — /scan, forward-visibility
  - camera_perception.CameraPerception  — /camera_info, /limelight/detections

Run:
  ros2 run wro_behavior wro_mission
"""
import math
import os
from enum import IntEnum
from typing import Optional

import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState

import tf2_ros
from ament_index_python.packages import get_package_share_directory

from wro_behavior.lidar_perception import LidarPerception
from wro_behavior.camera_perception import CameraPerception, PILLAR_RED


PILLAR_PASS_OFFSET = 0.30    # how far off-path to swing around a pillar

# Front-lidar threshold that trips the first-corner guard (a wall this
# close means we've reached the first corner).
FIRST_CORNER_FRONT_M = 0.80


class State(IntEnum):
    NOT_READY = 0             # startup gates not yet cleared
    WAITING_FOR_START = 1     # Nav2 active, direction resolved, waiting for button
    DRIVING = 2               # following waypoints (config or exploration)
    PARKING = 3               # endgame — heading into the parking bay
    DONE = 4                  # mission complete, robot stopped


class WROMission(Node):

    def __init__(self):
        super().__init__('wro_mission')

        # ============================================================
        # Parameters
        # ============================================================
        default_config = os.path.join(
            get_package_share_directory('wro_behavior'),
            'config', 'mission.yaml',
        )
        self.declare_parameter('config', default_config)
        self.declare_parameter('pillar_confirm_count', 5)
        self.declare_parameter('wait_for_start', True)

        # Direction is always resolved at startup from the first line the
        # camera sees (Orange = CCW, Blue = CW). No manual override.
        self.direction = None
        pillar_confirm = int(self.get_parameter('pillar_confirm_count').value)
        cfg_path = self.get_parameter('config').value

        with open(cfg_path) as f:
            self.cfg = yaml.safe_load(f)['common']
        self.corners_per_race = int(self.cfg.get('corners_per_race', 12))

        # ============================================================
        # State machine
        # ============================================================
        self.state = State.NOT_READY

        # Data owned by NOT_READY (gate progress)
        self.nav_state_future = None

        # Data owned by DRIVING
        self.i = 0
        self.corners_done = 0
        self.waiting_for_arrival = False
        self.dynamic_pending = True   # first waypoint is computed from lidar
        self.first_corner_guard_fired = False

        # Button state
        self.wait_for_start = bool(self.get_parameter('wait_for_start').value)
        self.start_pressed = not self.wait_for_start

        # ============================================================
        # ROS I/O + perception
        # ============================================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.lidar = LidarPerception(self, self.tf_buffer)
        self.camera = CameraPerception(
            self, self.tf_buffer, pillar_confirm_count=pillar_confirm)

        self.pub = self.create_publisher(PoseStamped, '/waypoint', 10)

        self.create_subscription(Bool, '/waypoint_arrived', self.on_arrived, 10)
        self.create_subscription(Bool, '/start_button', self.on_start_button, 10)

        self.nav_state_client = self.create_client(
            GetState, '/bt_navigator/get_state')

        # ============================================================
        # Timers
        # ============================================================
        self.tick_timer = self.create_timer(0.1, self._tick)

        self.get_logger().info('wro_mission — state=NOT_READY')

    # ================================================================
    # State transition helper
    # ================================================================
    def _transition(self, new_state: State) -> None:
        if new_state == self.state:
            return
        self.get_logger().info(
            f'state {self.state.name}  ->  {new_state.name}')
        self.state = new_state

    # ================================================================
    # Main tick — dispatches by state
    # ================================================================
    def _tick(self) -> None:
        if self.state == State.NOT_READY:
            self._tick_not_ready()
        elif self.state == State.WAITING_FOR_START:
            self._tick_waiting_for_start()
        elif self.state == State.DRIVING:
            self._first_corner_guard()
        elif self.state == State.PARKING:
            pass   # arrival handled by on_arrived
        elif self.state == State.DONE:
            pass   # nothing to do

    # ================================================================
    # NOT_READY — poll Nav2 lifecycle + resolve direction
    # ================================================================
    def _tick_not_ready(self) -> None:
        if self.pub.get_subscription_count() == 0:
            return
        if not self.nav_state_client.service_is_ready():
            return
        if self.nav_state_future is None:
            self.nav_state_future = self.nav_state_client.call_async(
                GetState.Request())
            return
        if not self.nav_state_future.done():
            return

        fut, self.nav_state_future = self.nav_state_future, None
        try:
            resp = fut.result()
        except Exception as exc:
            self.get_logger().warn(f'get_state failed: {exc}')
            return
        if resp is None or resp.current_state.id != LifecycleState.PRIMARY_STATE_ACTIVE:
            return

        # Direction is resolved opportunistically once we start moving
        # (the robot may need to drive forward before the camera can see
        # a line). No gate on it here.
        self._transition(State.WAITING_FOR_START)

        self.get_logger().info("DORINA: transition to WAITING_FOR_START")
        self.get_logger().info("DORINA: fwd distance" + str(self.lidar_perception.front_distance()))
        self.get_logger().info("DORINA: left distance" + str(self.lidar_perception.left_min_distance()))
        self.get_logger().info("DORINA: right distance" + str(self.lidar_perception.right_min_distance()))

    # ================================================================
    # WAITING_FOR_START — button gate
    # ================================================================
    def _tick_waiting_for_start(self) -> None:
        if not self.start_pressed:
            return
        self.get_logger().info("DORINA: fwd distance" + str(self.lidar_perception.front_distance()))
        self.get_logger().info("DORINA: left distance" + str(self.lidar_perception.left_min_distance()))
        self.get_logger().info("DORINA: right distance" + str(self.lidar_perception.right_min_distance()))
        self.get_logger().info("DORINA: transition to DRIVING")
        self._transition(State.DRIVING)
        self._send_next_waypoint()

    # ================================================================
    # Waypoint dispatch — event-driven (called from tick transitions
    # and from on_arrived).
    # ================================================================
    def _send_next_waypoint(self) -> None:
        # Opportunistic direction resolution each dispatch.
        self.get_logger().info("DORINA: fwd distance" + str(self.lidar_perception.front_distance()))
        self.get_logger().info("DORINA: left distance" + str(self.lidar_perception.left_min_distance()))
        self.get_logger().info("DORINA: right distance" + str(self.lidar_perception.right_min_distance()))
        self.get_logger().info("DORINA: detections", str(camera.detections))
        if self.direction is None:
            d = self.camera.detect_direction()
            if d is not None:
                self.get_logger().info("DORINA: direction", d)
                self.direction = d
                self.i = 0          # first config waypoint starts fresh
                self.get_logger().info(f'direction detected: {d}')

        wps = self._waypoints()
        if not wps:
            # No pose/scan yet, or list empty. Try again next arrival.
            self.get_logger().warn('no waypoint available yet')
            return
        if self.i >= len(wps):
            self._finish_lap()
            return

        x, y = wps[self.i]

        # Pillar avoidance only when following the configured route.
        if self.direction is not None:
            adj = self._pillar_avoidance_offset(x, y)
            if adj is not None:
                x, y = adj
                self.get_logger().info(
                    f'  detour around pillar -> ({x:.2f}, {y:.2f})')

        label = 'exploring' if self.direction is None else 'waypoint'
        self.get_logger().info(
            f'{label} {self.i + 1}/{len(wps)}: ({x:.2f}, {y:.2f})')
        self._publish_waypoint(x, y)

    def _publish_waypoint(self, x, y) -> None:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.w = 1.0
        self.waiting_for_arrival = True
        self.pub.publish(p)

    def on_arrived(self, msg: Bool) -> None:
        if not self.waiting_for_arrival:
            return
        self.waiting_for_arrival = False
        if not msg.data:
            self.get_logger().error(
                f'waypoint {self.i + 1} failed — mission paused')
            return

        if self.state == State.PARKING:
            self._transition(State.DONE)
            self.get_logger().info(
                f'FINISHED (parked) corners={self.corners_done}')
            return

        # STRAIGHT / TURNING branch — dispatch next waypoint
        if self.direction is None:
            # Still exploring — keep re-sending forward-edge waypoint.
            # Do NOT advance self.i (there's only "the forward direction").
            pass
        elif self.dynamic_pending:
            self.dynamic_pending = False
            self.get_logger().info(
                'arrived at dynamic waypoint, resuming configured route')
        else:
            self.i += 1
            self.corners_done += 1
            self.get_logger().info(
                f'corner {self.corners_done}/{self.corners_per_race}')

        if self.corners_done >= self.corners_per_race:
            self._finish_lap()
        else:
            self._send_next_waypoint()

    def _finish_lap(self) -> None:
        if self.camera.pillar_seen_ever:
            park = self.cfg.get('parking_pose', {}).get(self.direction)
            if park:
                self._transition(State.PARKING)
                self.get_logger().info(
                    f'FINISHED (obstacle: {self.camera.pillar_detections_total} '
                    f'pillar detections) — heading to parking')
                self._publish_waypoint(park[0], park[1])
                return
        self._transition(State.DONE)
        self.get_logger().info(
            f'FINISHED (open: no pillars seen) corners={self.corners_done}')

    # ================================================================
    # First-corner guard
    # ================================================================
    def _first_corner_guard(self) -> None:
        """One-shot: the first time the front lidar shows a wall within
        FIRST_CORNER_FRONT_M, check how many line detections have landed.
        If <=1, the initial direction guess was probably wrong — flip it
        and re-dispatch."""
        if self.first_corner_guard_fired:
            return
        front = self.lidar.front_distance()
        if front is None or front >= FIRST_CORNER_FRONT_M:
            return
        self.first_corner_guard_fired = True
        if self.direction is None:
            return
        if self.camera.line_detections_total > 1:
            return
        old = self.direction
        self.direction = 'cw' if old == 'ccw' else 'ccw'
        self.i = 0
        self.get_logger().warn(
            f'first-corner guard: only '
            f'{self.camera.line_detections_total} line detection(s); '
            f'flipping direction {old} -> {self.direction}')
        self._send_next_waypoint()

    # ================================================================
    # Callbacks
    # ================================================================
    def on_start_button(self, msg: Bool) -> None:
        if msg.data and not self.start_pressed:
            self.start_pressed = True
            self.get_logger().info('start button pressed')

    # ================================================================
    # Pillar avoidance
    # ================================================================
    def _pillar_avoidance_offset(self, wp_x, wp_y):
        pose = self._robot_pose()
        if pose is None:
            return None
        pillar = self.camera.closest_pillar_in_map(pose)
        if pillar is None:
            return None
        cls, px, py = pillar
        rx, ry, _ = pose

        seg_dist = math.hypot(wp_x - rx, wp_y - ry)
        pillar_dist = math.hypot(px - rx, py - ry)
        if pillar_dist > seg_dist + 0.2 or pillar_dist > 1.5:
            return None

        path_yaw = math.atan2(wp_y - ry, wp_x - rx)
        side = +1 if cls == PILLAR_RED else -1
        perp = path_yaw + side * math.pi / 2
        return (px + PILLAR_PASS_OFFSET * math.cos(perp),
                py + PILLAR_PASS_OFFSET * math.sin(perp))

    # ================================================================
    # Pose + waypoint helpers
    # ================================================================
    def _robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _waypoints(self):
        """Current waypoint list.

        - Direction known: configured lane centres from mission.yaml.
        - Direction unknown: a single forward-edge point (drive straight
          ahead to the limit of what the lidar can see, so the robot
          moves and the camera has a chance to spot a line).
        """
        if self.direction is None:
            pose = self._robot_pose()
            if pose is None:
                return []
            dyn = self.lidar.forward_edge_waypoint(pose)
            self.get_logger().info("DORINA: fwd distance" + str(self.self.lidar_perception.front_distance()))
            self.get_logger().info("DORINA: left distance" + str(self.self.lidar_perception.left_min_distance()))
            self.get_logger().info("DORINA: right distance" + str(self.self.lidar_perception.right_min_distance()))
            self.get_logger().info("DORINA: detections", str(camera.detections))
            return [dyn] if dyn is not None else []
        return self.cfg.get(f'waypoints_{self.direction}', [])


def main():
    rclpy.init()
    node = WROMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
