"""WRO Obstacle Challenge mission node.

Self-contained — no shared base with the Open Challenge. Changes here
must not touch open_mission.py.

Strategy:

  - State machine: NOT_READY → WAITING_FOR_START → DRIVING → PARKING → DONE.
  - Direction commit: first line ever seen sets turn_dir
      (orange → CCW/+1, blue → CW/-1). Lidar left/right fallback if
      no lines appear and front wall is close.
  - Straight-following: publish the camera's `far` line (or `close`
      if `far` is None) as the next target, nudged laterally when a
      pillar is in the way.
  - Pillar rule (WRO 9.19): RED passes on robot's right (waypoint
      offset LEFT of pillar); GREEN passes on robot's left
      (waypoint offset RIGHT).
  - Corner trigger: line-close threshold — camera's `close` line
      within CORNER_TURN_DIST_M.
  - Corner strategy: TWO-STAGE geometric Ackermann arc.
      Stage 1 — arc anchor on the 45° bisector, forcing Nav2's planner
                to sweep through the corner instead of a hard bend.
      Stage 2 — final leg past the corner. Recomputed from the arc-
                anchor pose so it sits on the actual arc continuation.
    Arc radius fitted to lidar clearance; anchor scaled by delta yaw.
  - Replay: lap-1 stage-2 targets are recorded and dispatched verbatim
      on laps 2 and 3.
  - Endgame: 12th corner triggers PARKING (placeholder — transitions
      to DONE; wire up when field-frame parking pose is calibrated).
  - Failure recovery: /waypoint_arrived == False re-runs dispatch.

Run:
  ros2 run wro_behavior obstacle_mission
"""
import math
from enum import IntEnum
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState

import tf2_ros

from wro_behavior.lidar_perception import LidarPerception
from wro_behavior.camera_perception import (
    CameraPerception, LINE_ORANGE, PILLAR_RED,
)


# WRO Future Engineers: 3 laps × 4 corners.
CORNERS_PER_RACE = 12
CORNERS_PER_LAP = 4

# Front-lidar threshold below which we treat the wall ahead as "corner
# reached" — both the lidar-only fallback and the general reach check.
FIRST_CORNER_FRONT_M = 0.80

# Line-projected corner trigger — when the camera's closest line is
# this close in map frame, we've reached the corner.
CORNER_TURN_DIST_M = 0.4

# When no line is visible, publish short forward hops instead of one
# long forward-edge target. Keeps the dispatch loop tight.
EXPLORATION_STEP_M = 0.4

# Ackermann arc floor — matches nav2_params.yaml `min_turning_r`.
MIN_TURNING_RADIUS_M = 0.35
# Safety margin between the arc and the wall it's skirting.
CORNER_SAFETY_MARGIN_M = 0.15

# How far off-path to swing when avoiding a pillar.
PILLAR_PASS_OFFSET = 0.30


class State(IntEnum):
    NOT_READY = 0             # startup gates not yet cleared
    WAITING_FOR_START = 1     # Nav2 active, waiting for start button
    DRIVING = 2               # following waypoints
    PARKING = 3               # endgame — heading into the parking bay
    DONE = 4                  # mission complete, robot stopped


class ObstacleMission(Node):
    def __init__(self):
        super().__init__('obstacle_mission')

        # ============================================================
        # Parameters
        # ============================================================
        self.declare_parameter('pillar_confirm_count', 3)
        self.declare_parameter('wait_for_start', True)
        pillar_confirm = int(self.get_parameter('pillar_confirm_count').value)

        # ============================================================
        # State machine
        # ============================================================
        self.state = State.NOT_READY

        # Data owned by NOT_READY
        self.nav_state_future = None

        # Data owned by DRIVING
        self.corners_done = 0
        self.waiting_for_arrival = False
        self.last_waypoint: Optional[Tuple[float, float]] = None
        self.last_closest_line = None
        # +1 = left/CCW, -1 = right/CW, 0 = not yet decided.
        self.turn_dir = 0
        # Recorded stage-2 targets from lap 1, replayed on laps 2-3.
        self._corner_history: list = []

        # Two-stage corner state:
        #   0 = normal line-following
        #   1 = arc-anchor waypoint sent, awaiting arrival
        #   2 = final-leg waypoint sent, awaiting arrival
        self._corner_stage = 0
        # (tx, ty, new_yaw) stashed at corner trigger, dispatched on
        # stage-1 arrival (recomputed from arc-anchor pose then).
        self._corner_final: Optional[Tuple[float, float, float]] = None

        # Button state
        self.wait_for_start = bool(
            self.get_parameter('wait_for_start').value)
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

        self.get_logger().info(
            f'obstacle_mission — state=NOT_READY '
            f'(wait_for_start={self.wait_for_start})')

    # ================================================================
    # State transition
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
    # NOT_READY — poll Nav2 lifecycle
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
        if (resp is None
                or resp.current_state.id != LifecycleState.PRIMARY_STATE_ACTIVE):
            return

        self._transition(State.WAITING_FOR_START)

    # ================================================================
    # WAITING_FOR_START — button gate
    # ================================================================
    def _tick_waiting_for_start(self) -> None:
        if not self.start_pressed:
            return
        self._transition(State.DRIVING)
        self._send_next_waypoint()

    # ================================================================
    # Waypoint dispatch — event-driven
    # ================================================================
    def _send_next_waypoint(self) -> None:
        if self.state != State.DRIVING:
            return

        pose = self._robot_pose()
        if pose is not None:
            rx, ry, yaw = pose
            self.get_logger().info(
                f'-- dispatch: pose=({rx:.2f}, {ry:.2f}) '
                f'yaw={math.degrees(yaw):.0f}° '
                f'corners={self.corners_done}/{CORNERS_PER_RACE} '
                f'turn_dir={self.turn_dir}')

        # Stage-1 arrival → dispatch stage-2 final leg. Recompute from
        # the CURRENT (arc-anchor) pose so the final target lies on
        # the actual arc trajectory continuation, not the pre-arc pose
        # we stashed.
        if self._corner_stage == 1 and self._corner_final is not None:
            _, _, new_yaw = self._corner_final
            if pose is None:
                return
            arc_x, arc_y, _ = pose
            step = self._turn_step_m()
            tx = arc_x + step * math.cos(new_yaw)
            ty = arc_y + step * math.sin(new_yaw)
            self._corner_stage = 2
            self._record_replay_target(tx, ty, new_yaw)
            self.get_logger().info(
                f'corner turn stage 2 -> final ({tx:.2f}, {ty:.2f}) '
                f'yaw={math.degrees(new_yaw):.0f}°')
            self._publish_waypoint(tx, ty, yaw=new_yaw)
            return
        if self._corner_stage == 2:
            self._corner_stage = 0
            self._corner_final = None

        # Replay mode: after lap 1, dispatch one recorded corner target
        # per arrival.
        if (self.corners_done >= CORNERS_PER_LAP
                and len(self._corner_history) == CORNERS_PER_LAP):
            self._dispatch_replay()
            return

        lines = self.camera.detect_direction()

        # No line in view. Sub-cases:
        #   (a) lidar close ahead + turn_dir known → lidar-only corner.
        #   (b) otherwise → short forward-edge hop.
        if lines is None:
            if pose is None:
                return
            rx, ry, yaw = pose
            front = self.lidar.front_distance()

            if (self.turn_dir != 0
                    and front is not None
                    and front < FIRST_CORNER_FRONT_M):
                self.get_logger().warn(
                    f'lidar-only corner (front={front:.2f}m, '
                    f'turn_dir={self.turn_dir:+d})')
                self._execute_corner_turn(rx, ry, yaw, None)
                return

            dyn = self.lidar.forward_edge_waypoint(
                pose, max_distance=EXPLORATION_STEP_M)
            if dyn is None:
                self.get_logger().info(
                    f'no line in view (turn_dir={self.turn_dir}) — '
                    f'no progress can be made')
                return
            self.get_logger().info(
                f'no line in view (turn_dir={self.turn_dir}) — '
                f'forward-edge at ({dyn[0]:.2f}, {dyn[1]:.2f})')
            self._publish_waypoint(*dyn)
            return

        close, far = lines

        # First line ever seen commits driving direction.
        if self.turn_dir == 0:
            self.turn_dir = +1 if close[0] == LINE_ORANGE else -1
            self.get_logger().info(
                f'direction set from {close[0]}: turn_dir={self.turn_dir}')

        if pose is None:
            return
        rx, ry, yaw = pose
        close_dist = math.hypot(close[1] - rx, close[2] - ry)

        # Line-triggered corner: close line right on top of us.
        if close_dist < CORNER_TURN_DIST_M and self.turn_dir != 0:
            self.get_logger().info(
                f'line-triggered corner (close_dist={close_dist:.2f}m, '
                f'turn_dir={self.turn_dir:+d})')
            self._execute_corner_turn(rx, ry, yaw, close)
            return

        # Straight-following: prefer the far line as the target; fall
        # back to the close one. Then apply pillar detour.
        target = far if far is not None else close
        kind = 'far' if far is not None else 'close'
        x, y = target[1], target[2]
        self.last_closest_line = close

        adjusted = self._pillar_detour(x, y)
        if adjusted != (x, y):
            x, y = adjusted
            self.get_logger().info(
                f'  target adjusted -> ({x:.2f}, {y:.2f})')

        self.get_logger().info(f'{kind} waypoint: ({x:.2f}, {y:.2f})')
        self._publish_waypoint(x, y)

    # ================================================================
    # Corner strategy: two-stage arc
    # ================================================================
    def _execute_corner_turn(
        self,
        rx: float, ry: float, yaw: float,
        close: Optional[Tuple[str, float, float]],
    ) -> None:
        if self._corner_stage != 0:
            return   # already mid-turn

        new_yaw = yaw + self.turn_dir * math.pi / 2
        new_yaw = round(new_yaw / (math.pi / 2)) * (math.pi / 2)
        # Actual arc angle to sweep — smaller when the robot's incoming
        # yaw is already partway to the new cardinal.
        delta_yaw = math.atan2(math.sin(new_yaw - yaw),
                               math.cos(new_yaw - yaw))
        mid_yaw = (yaw + new_yaw) / 2.0
        anchor = self._arc_anchor_m(delta_yaw)
        step = self._turn_step_m()
        mid_x = rx + anchor * math.cos(mid_yaw)
        mid_y = ry + anchor * math.sin(mid_yaw)
        tx = rx + step * math.cos(new_yaw)
        ty = ry + step * math.sin(new_yaw)

        self._corner_final = (tx, ty, new_yaw)
        self._corner_stage = 1
        self.get_logger().info(
            f'corner turn stage 1 (turn_dir={self.turn_dir:+d}, '
            f'delta={math.degrees(delta_yaw):.0f}°) -> '
            f'arc ({mid_x:.2f}, {mid_y:.2f}) '
            f'yaw={math.degrees(mid_yaw):.0f}°')

        self._advance_corner(rx, ry, yaw, new_yaw)
        if close is not None:
            self.last_closest_line = close

        if self.state != State.DRIVING:
            return   # `_finish_lap` already published the endgame target
        self._publish_waypoint(mid_x, mid_y, yaw=mid_yaw)

    # ================================================================
    # Pillar detour
    # ================================================================
    def _pillar_detour(
        self, x: float, y: float,
    ) -> Tuple[float, float]:
        pose = self._robot_pose()
        if pose is None:
            return x, y
        pillar = self.camera.closest_pillar_in_map(pose)
        if pillar is None:
            return x, y
        cls, px, py = pillar
        rx, ry, _ = pose

        seg_dist = math.hypot(x - rx, y - ry)
        pillar_dist = math.hypot(px - rx, py - ry)
        # Only detour if the pillar is between the robot and the target,
        # and within a couple metres — otherwise it's not in the way.
        if pillar_dist > seg_dist + 0.2 or pillar_dist > 1.5:
            return x, y

        path_yaw = math.atan2(y - ry, x - rx)
        # WRO rule 9.19:
        #   RED pillar   → pass on the robot's right (waypoint offset LEFT).
        #   GREEN pillar → pass on the robot's left  (waypoint offset RIGHT).
        side = +1 if cls == PILLAR_RED else -1
        perp = path_yaw + side * math.pi / 2
        return (px + PILLAR_PASS_OFFSET * math.cos(perp),
                py + PILLAR_PASS_OFFSET * math.sin(perp))

    # ================================================================
    # Endgame: parking (placeholder until field-frame calibration is done)
    # ================================================================
    def _finish_lap(self) -> None:
        self._transition(State.DONE)
        self.get_logger().info(
            f'FINISHED (obstacle: parking placeholder) '
            f'corners={self.corners_done}')

    # ================================================================
    # Arc geometry helpers
    # ================================================================
    def _arc_radius_m(self) -> float:
        """Fit the arc radius to the lidar's actual free-space readings.
        Floors at MIN_TURNING_RADIUS_M so we never ask for a physically-
        impossible turn."""
        front = self.lidar.front_distance()
        if front is None:
            front = FIRST_CORNER_FRONT_M
        lateral = None
        if self.turn_dir > 0:
            lateral = self.lidar.left_min_distance()
        elif self.turn_dir < 0:
            lateral = self.lidar.right_min_distance()
        if lateral is None:
            lateral = 1.0
        r = min(front, lateral) - CORNER_SAFETY_MARGIN_M
        return max(r, MIN_TURNING_RADIUS_M)

    def _arc_anchor_m(self, delta_yaw: float = math.pi / 2) -> float:
        """Robot-to-arc-midpoint distance along the bisector. Chord
        length = 2·R·sin(delta/4). At delta=pi/2 this reduces to the
        classic 0.765·R."""
        return 2.0 * self._arc_radius_m() * math.sin(abs(delta_yaw) / 4.0)

    def _turn_step_m(self) -> float:
        """Post-corner leg distance: full arc radius past the corner
        so the robot lands well into the next straight."""
        return self._arc_radius_m() + CORNER_SAFETY_MARGIN_M

    # ================================================================
    # Waypoint publishing
    # ================================================================
    def _publish_waypoint(
        self, x: float, y: float, yaw: Optional[float] = None,
    ) -> None:
        # Skip if the new target is within 5 cm of the last one.
        if self.last_waypoint is not None:
            rx, ry = self.last_waypoint
            if math.hypot(y - ry, x - rx) <= 0.05:
                self.get_logger().info('Too close to publish new waypoint')
                return

        # Default yaw: face the direction of travel.
        if yaw is None:
            pose = self._robot_pose()
            if pose is not None:
                rx, ry, _ = pose
                yaw = math.atan2(y - ry, x - rx)
            else:
                yaw = 0.0

        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)

        self.waiting_for_arrival = True
        self.pub.publish(p)
        self.last_waypoint = (x, y)
        self.get_logger().info(
            f'published waypoint: ({x:.2f}, {y:.2f}) '
            f'yaw={math.degrees(yaw):.0f}°')

    # ================================================================
    # Corner counting + replay
    # ================================================================
    def _advance_corner(
        self, x: float, y: float,
        incoming_yaw: float, outgoing_yaw: float,
    ) -> None:
        self.corners_done += 1
        self.get_logger().info(
            f'corner {self.corners_done}/{CORNERS_PER_RACE} '
            f'at ({x:.2f}, {y:.2f}) '
            f'yaw {math.degrees(incoming_yaw):.0f}° -> '
            f'{math.degrees(outgoing_yaw):.0f}°')
        if self.corners_done >= CORNERS_PER_RACE:
            self._finish_lap()

    def _record_replay_target(
        self, tx: float, ty: float, yaw: float,
    ) -> None:
        if len(self._corner_history) < CORNERS_PER_LAP:
            self._corner_history.append((tx, ty, yaw))
            if len(self._corner_history) == CORNERS_PER_LAP:
                summary = ', '.join(
                    f'({rx:.2f}, {ry:.2f})'
                    for rx, ry, _ in self._corner_history)
                self.get_logger().info(
                    f'lap 1 mapped — replay targets: {summary}')

    def _dispatch_replay(self) -> None:
        idx = (self.corners_done - CORNERS_PER_LAP) % CORNERS_PER_LAP
        tx, ty, tyaw = self._corner_history[idx]
        self.corners_done += 1
        self.get_logger().info(
            f'replay corner {self.corners_done}/{CORNERS_PER_RACE} '
            f'-> ({tx:.2f}, {ty:.2f}) yaw={math.degrees(tyaw):.0f}°')
        if self.corners_done >= CORNERS_PER_RACE:
            self._finish_lap()
            return
        self._publish_waypoint(tx, ty, yaw=tyaw)

    # ================================================================
    # Arrival + start-button callbacks
    # ================================================================
    def on_arrived(self, msg: Bool) -> None:
        if not self.waiting_for_arrival:
            return
        self.waiting_for_arrival = False
        if not msg.data:
            self.get_logger().warn(
                'waypoint failed — recomputing next target')
            self._send_next_waypoint()
            return

        if self.state == State.PARKING:
            self._transition(State.DONE)
            self.get_logger().info(
                f'FINISHED (parked) corners={self.corners_done}')
            return

        self._send_next_waypoint()

    def on_start_button(self, msg: Bool) -> None:
        if msg.data and not self.start_pressed:
            self.start_pressed = True
            self.get_logger().info('start button pressed')

    # ================================================================
    # First-corner guard — pick turn_dir from lidar if no line ever seen
    # ================================================================
    def _first_corner_guard(self) -> None:
        if self.turn_dir != 0:
            return
        front = self.lidar.front_distance()
        if front is None or front >= FIRST_CORNER_FRONT_M:
            return
        if self.camera.line_detections_total > 0:
            return

        left = self.lidar.left_min_distance()
        right = self.lidar.right_min_distance()
        if left is None or right is None:
            self.get_logger().warn(
                'first-corner guard: no lines and lidar left/right unavailable')
            return
        self.turn_dir = +1 if left > right else -1
        self.get_logger().warn(
            f'first-corner guard: no lines; turn '
            f'{"left" if self.turn_dir > 0 else "right"} '
            f'(left={left:.2f}m, right={right:.2f}m)')
        self._send_next_waypoint()

    # ================================================================
    # Pose helper
    # ================================================================
    def _robot_pose(self) -> Optional[Tuple[float, float, float]]:
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


def main():
    rclpy.init()
    node = ObstacleMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
