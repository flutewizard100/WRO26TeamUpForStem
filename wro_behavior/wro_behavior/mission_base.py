"""Shared mission-node base class for WRO Future Engineers.

Implements the parts of the mission that are identical across the Open
and Obstacle challenges:

  - State machine (NOT_READY → WAITING_FOR_START → DRIVING → endgame → DONE).
  - Nav2 lifecycle poll, start-button gate.
  - Direction resolution from the first line seen (Orange=CCW, Blue=CW).
  - First-corner lidar fallback if no line ever appears.
  - Line-following (straight targets) with optional subclass hook.
  - Corner triggers (line-based when close_dist small, lidar-based when
    front lidar close and no line visible).
  - Failure-recovery retry on `/waypoint_arrived == False`.
  - Waypoint publishing with 5 cm dedup and auto-yaw.
  - Corner-history recording during lap 1, replay dispatch for laps 2-3.

Subclasses implement:
  - `_execute_corner_turn(rx, ry, yaw, close)` — the corner strategy.
  - `_finish_lap()` — the endgame (return home / park).
  - `_adjust_straight_target(x, y)` — optional (e.g. pillar detour).
  - `_handle_pre_dispatch(pose)` — optional (e.g. two-stage arc arrivals).

Concrete nodes: OpenMission (open_mission.py), ObstacleMission
(obstacle_mission.py).
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
    CameraPerception, LINE_ORANGE, LINE_BLUE,
)


# WRO Future Engineers: 3 laps × 4 corners.
CORNERS_PER_RACE = 12
CORNERS_PER_LAP = 4

# Front-lidar threshold below which we treat the wall ahead as "corner
# reached" — both the lidar-only fallback and the general reach check.
FIRST_CORNER_FRONT_M = 0.80

# Line-projected corner trigger — when the camera's closest line is this
# close in map frame, we've reached the corner.
CORNER_TURN_DIST_M = 0.4

# When no line is visible, publish short forward hops instead of one
# long forward-edge target. Keeps the dispatch loop tight so a line
# reappearing gets picked up quickly.
EXPLORATION_STEP_M = 0.4


class State(IntEnum):
    NOT_READY = 0             # startup gates not yet cleared
    WAITING_FOR_START = 1     # Nav2 active, waiting for start button
    DRIVING = 2               # following waypoints
    PARKING = 3               # endgame — heading into the parking bay (obstacle)
    RETURNING_HOME = 4        # endgame — driving back to (0, 0) (open)
    DONE = 5                  # mission complete, robot stopped


class MissionBase(Node):
    """Shared base class. Subclass and override the abstract hooks."""

    # Subclasses override this for their node name.
    NODE_NAME = 'wro_mission'

    def __init__(self):
        super().__init__(self.NODE_NAME)

        # ============================================================
        # Parameters
        # ============================================================
        self.declare_parameter('pillar_confirm_count', 3)
        self.declare_parameter('wait_for_start', True)
        pillar_confirm = int(self.get_parameter('pillar_confirm_count').value)
        self.corners_per_race = CORNERS_PER_RACE

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
        # Recorded stage-final targets from lap 1 corners, used for
        # replay dispatch on laps 2-3.
        self._corner_history: list = []

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
            f'{self.NODE_NAME} — state=NOT_READY '
            f'(wait_for_start={self.wait_for_start})')

    # ================================================================
    # Subclass hooks
    # ================================================================
    def _execute_corner_turn(
        self,
        rx: float, ry: float, yaw: float,
        close: Optional[Tuple[str, float, float]],
    ) -> None:
        """Take a corner. Called when either the camera's `close` line
        detection is within CORNER_TURN_DIST_M (then `close` is set)
        OR the lidar sees a wall within FIRST_CORNER_FRONT_M with no
        line visible (then `close` is None). Subclass must publish a
        waypoint and call `_advance_corner`."""
        raise NotImplementedError

    def _finish_lap(self) -> None:
        """Endgame — publish the return-home / parking waypoint and
        transition state. Called from `_advance_corner` when the corner
        target count is reached."""
        raise NotImplementedError

    def _adjust_straight_target(
        self, x: float, y: float,
    ) -> Tuple[float, float]:
        """Optional hook to nudge the straight-following target (e.g.
        pillar avoidance in the obstacle challenge). Default: return
        the target unchanged."""
        return x, y

    def _handle_pre_dispatch(
        self, pose: Optional[Tuple[float, float, float]],
    ) -> bool:
        """Optional hook to handle any subclass-specific state at the
        top of `_send_next_waypoint` (e.g. mid-corner stage arrivals).
        Return True to short-circuit the rest of dispatch."""
        return False

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
        elif self.state == State.RETURNING_HOME:
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
    # Waypoint dispatch — event-driven (called from tick transitions
    # and from on_arrived).
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
                f'corners={self.corners_done}/{self.corners_per_race} '
                f'turn_dir={self.turn_dir}')

        # Subclass-specific pre-dispatch (e.g., obstacle's stage-1/2
        # arrivals). Short-circuit if handled.
        if self._handle_pre_dispatch(pose):
            return

        # Replay mode: after lap 1, dispatch one recorded corner target
        # per arrival. Nav2's SmacPlannerHybrid plans the ackermann arc
        # from wherever the robot is to the next stored waypoint.
        if (self.corners_done >= CORNERS_PER_LAP
                and len(self._corner_history) == CORNERS_PER_LAP):
            self._dispatch_replay()
            return

        lines = self.camera.detect_direction()

        # No line in view. Two sub-cases:
        #   (a) lidar sees a wall close ahead AND we know the driving
        #       direction — camera lost the line right at the corner,
        #       so fire the geometric corner turn using turn_dir.
        #   (b) otherwise — fall back to lidar's forward-edge waypoint
        #       so the robot keeps moving until a line reappears.
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

        # Line-triggered corner: close line is right on top of us.
        if close_dist < CORNER_TURN_DIST_M and self.turn_dir != 0:
            self.get_logger().info(
                f'line-triggered corner (close_dist={close_dist:.2f}m, '
                f'turn_dir={self.turn_dir:+d})')
            self._execute_corner_turn(rx, ry, yaw, close)
            return

        # Straight-following: prefer the far line as the target; fall
        # back to the close one.
        target = far if far is not None else close
        kind = 'far' if far is not None else 'close'
        x, y = target[1], target[2]
        self.last_closest_line = close

        # Subclass hook (e.g., pillar detour for the obstacle challenge).
        adjusted = self._adjust_straight_target(x, y)
        if adjusted != (x, y):
            x, y = adjusted
            self.get_logger().info(
                f'  target adjusted -> ({x:.2f}, {y:.2f})')

        self.get_logger().info(f'{kind} waypoint: ({x:.2f}, {y:.2f})')
        self._publish_waypoint(x, y)

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
        """Count a completed corner. `_finish_lap` fires when count
        hits the race total."""
        self.corners_done += 1
        self.get_logger().info(
            f'corner {self.corners_done}/{self.corners_per_race} '
            f'at ({x:.2f}, {y:.2f}) '
            f'yaw {math.degrees(incoming_yaw):.0f}° -> '
            f'{math.degrees(outgoing_yaw):.0f}°')
        if self.corners_done >= self.corners_per_race:
            self._finish_lap()

    def _record_replay_target(
        self, tx: float, ty: float, yaw: float,
    ) -> None:
        """Store one corner's target for replay on laps 2-3. Subclasses
        call this from `_execute_corner_turn` at whatever moment their
        strategy has computed a valid "start of next straight" point."""
        if len(self._corner_history) < CORNERS_PER_LAP:
            self._corner_history.append((tx, ty, yaw))
            if len(self._corner_history) == CORNERS_PER_LAP:
                summary = ', '.join(
                    f'({rx:.2f}, {ry:.2f})'
                    for rx, ry, _ in self._corner_history)
                self.get_logger().info(
                    f'lap 1 mapped — replay targets: {summary}')

    def _dispatch_replay(self) -> None:
        """Publish the next pre-recorded corner target."""
        idx = (self.corners_done - CORNERS_PER_LAP) % CORNERS_PER_LAP
        tx, ty, tyaw = self._corner_history[idx]
        self.corners_done += 1
        self.get_logger().info(
            f'replay corner {self.corners_done}/{self.corners_per_race} '
            f'-> ({tx:.2f}, {ty:.2f}) yaw={math.degrees(tyaw):.0f}°')
        if self.corners_done >= self.corners_per_race:
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

        if self.state == State.RETURNING_HOME:
            self._transition(State.DONE)
            self.get_logger().info(
                f'FINISHED (home) corners={self.corners_done}')
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
