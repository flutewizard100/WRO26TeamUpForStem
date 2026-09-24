"""WRO Open Challenge mission node.

Self-contained — no shared base with the Obstacle Challenge. 
Obstacle work must not touch this file.

Run:
  ros2 run wro_behavior open_mission
"""
import math
from dataclasses import dataclass
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
    CameraPerception, LINE_ORANGE,
)


# WRO Future Engineers: 3 laps × 4 corners.
CORNERS_PER_RACE = 12
CORNERS_PER_LAP = 4

# All tunable distances / thresholds are ROS parameters. Defaults live
# next to their declare_parameter() calls in __init__; production values
# come from wro_behavior/config/open_tuning.yaml (loaded by the launch).


class State(IntEnum):
    NOT_READY = 0             # startup gates not yet cleared
    WAITING_FOR_START = 1     # Nav2 active, waiting for start button
    FIRST_CORNER = 2               # following waypoints
    FIRST_LAP = 3
    RETURNING_HOME = 4        # endgame — driving back to (0, 0)
    DONE = 5                  # mission complete, robot stopped



class FirstKnownLocation:
    """Lidar + pose snapshot."""
    pose:  Tuple[float, float, float]           # (x, y, yaw) in map frame
    front: Optional[float]                      # metres to wall ahead
    left:  Optional[float]                      # metres to wall on +Y side
    right: Optional[float]                      # metres to wall on -Y side
    angle_of_longest_distance: Optional[float]


class OpenMission(Node):
    def __init__(self):
        super().__init__('open_mission')


        # ============================================================
        # Parameters — see wro_behavior/config/open_tuning.yaml for
        # the tuned values and per-knob documentation.
        # ============================================================
        self.declare_parameter('wait_for_start', True)

        # # Corner trigger.
        # self.declare_parameter('inner_corner_front_max_m', 1.5)
        # self.declare_parameter('inner_corner_diag_margin_m', 0.35)

        # # Corner waypoint geometry (distances from the estimated corner).
        # self.declare_parameter('corner_lead_m', 0.80)
        # self.declare_parameter('corner_lateral_m', 0.00)

        # # Straight-line + fallback behaviours.
        # self.declare_parameter('first_corner_front_m', 0.80)
        # self.declare_parameter('exploration_step_m', 0.40)

        # # Corner K-turn recovery.
        # self.declare_parameter('corner_recovery_backup_m', 0.35)
        # self.declare_parameter('corner_recovery_max_retries', 2)

        # # Snapshot tunables into instance attributes for fast access.
        # self.inner_corner_front_max_m = float(
        #     self.get_parameter('inner_corner_front_max_m').value)
        # self.inner_corner_diag_margin_m = float(
        #     self.get_parameter('inner_corner_diag_margin_m').value)
        # self.corner_lead_m = float(
        #     self.get_parameter('corner_lead_m').value)
        # self.corner_lateral_m = float(
        #     self.get_parameter('corner_lateral_m').value)
        # self.first_corner_front_m = float(
        #     self.get_parameter('first_corner_front_m').value)
        # self.exploration_step_m = float(
        #     self.get_parameter('exploration_step_m').value)
        # self.corner_recovery_backup_m = float(
        #     self.get_parameter('corner_recovery_backup_m').value)
        # self.corner_recovery_max_retries = int(
        #     self.get_parameter('corner_recovery_max_retries').value)

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
        self.first_known_location = FirstKnownLocation()
        # Recorded corner targets from lap 1, replayed on laps 2-3.
        self._corner_history: list = []

        # Corner K-turn recovery state.
        #   `_pending_corner` — target of the most recent corner
        #      dispatch (or None if the last dispatch wasn't a corner
        #      or has been fully acknowledged as arrived).
        #   `_recovery_stage` — 0 = idle, 1 = backup published, 2 = corner
        #      re-published after a backup.
        #   `_corner_retries` — number of backup+retry cycles spent on
        #      the current corner (reset when a new corner is dispatched).
        # self._pending_corner: Optional[Tuple[float, float, float]] = None
        # self._recovery_stage: int = 0
        # self._corner_retries: int = 0

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
        self.camera = CameraPerception(self, self.tf_buffer)

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
            f'open_mission — state=NOT_READY '
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
        elif self.state == State.FIRST_CORNER:
            self._tick_first_corner()
        elif self.state == State.FIRST_LAP:
            self._tick_first_lap()
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
        self._transition(State.FIRST_CORNER)

    def _tick_first_corner(self) -> None:
        pose = self._robot_pose()
        if pose is None:
            return
        rx, ry, yaw = pose
        self.first_known_location.front = self.lidar.front_distance()
        self.first_known_location.left = self.lidar.left_min_distance()
        self.first_known_location.right = self.lidar.right_min_distance()
        self.first_known_location.angle_of_longest_distance = self.lidar.longest_forward_ray_angle()
        
        # now we check if we know the direction
        lines = self.camera.detect_direction()
        if lines is not None:
            close, far = lines
            self.turn_dir = +1 if close[0] == LINE_ORANGE else -1\
        else:
            self.turn_dir = self.first_known_location.angle_of_longest_distance / math.abs(self.first_known_location.angle_of_longest_distance) 
        
        self._publish_waypoint(self.first_known_location.front - 0.6, self.first_known_location.left + 0.3)
        self.get_logger().info(
            f'front-0.6={self.first_known_location.front - 0.6:.2f}m, '
            f'left+0.3={self.first_known_location.left + 0.3:.2f}m')
        self._publish_waypoint(self.first_known_location.front - 0.3, self.first_known_location.left + 0.6, math.pi/2)
        self.get_logger().info(
            f'front-0.3={self.first_known_location.front - 0.3:.2f}m, '
            f'left+0.6={self.first_known_location.left + 0.6:.2f}m')
        
        # TODO: logic for checking if the sensors are in agreement



    def _tick_first_lap(self) -> None:
        # Stub — no logic yet.
        pass

    def _send_next_waypoint(self) -> None:
        # Stub — the previous line-following body is commented out
        # below. Leaves callers (on_arrived, _attempt_corner_recovery,
        # _first_corner_guard) safe to invoke while the new state
        # machine is being built.
        pass

    # ================================================================
    # Waypoint dispatch — event-driven (called from tick transitions
    # and from on_arrived).
    # ================================================================
    # def _send_next_waypoint(self) -> None:
    #     if self.state != State.DRIVING:
    #         return

    #     pose = self._robot_pose()
    #     if pose is not None:
    #         rx, ry, yaw = pose
    #         self.get_logger().info(
    #             f'-- dispatch: pose=({rx:.2f}, {ry:.2f}) '
    #             f'yaw={math.degrees(yaw):.0f}° '
    #             f'corners={self.corners_done}/{CORNERS_PER_RACE} '
    #             f'turn_dir={self.turn_dir}')

    #     # Replay mode: after lap 1, dispatch one recorded corner target
    #     # per arrival. Nav2's SmacPlannerHybrid plans the ackermann arc
    #     # from wherever the robot is to the next stored waypoint.
    #     if (self.corners_done >= CORNERS_PER_LAP
    #             and len(self._corner_history) == CORNERS_PER_LAP):
    #         self._dispatch_replay()
    #         return

    #     lines = self.camera.detect_direction()

    #     # No line in view. Two sub-cases:
    #     #   (a) lidar sees a wall close ahead AND we know the driving
    #     #       direction — camera lost the line right at the corner,
    #     #       so fire the geometric corner turn using turn_dir.
    #     #   (b) otherwise — fall back to lidar's forward-edge waypoint
    #     #       so the robot keeps moving until a line reappears.
    #     if lines is None:
    #         if pose is None:
    #             return
    #         rx, ry, yaw = pose
    #         front = self.lidar.front_distance()

    #         if (self.turn_dir != 0
    #                 and front is not None
    #                 and front < self.first_corner_front_m):
    #             self.get_logger().warn(
    #                 f'lidar-only corner (front={front:.2f}m, '
    #                 f'turn_dir={self.turn_dir:+d})')
    #             self._execute_corner_turn(rx, ry, yaw, None)
    #             return

    #         dyn = self.lidar.forward_edge_waypoint(
    #             pose, max_distance=self.exploration_step_m)
    #         if dyn is None:
    #             self.get_logger().info(
    #                 f'no line in view (turn_dir={self.turn_dir}) — '
    #                 f'no progress can be made')
    #             return
    #         self.get_logger().info(
    #             f'no line in view (turn_dir={self.turn_dir}) — '
    #             f'forward-edge at ({dyn[0]:.2f}, {dyn[1]:.2f})')
    #         self._publish_waypoint(*dyn)
    #         return

    #     close, far = lines

    #     # First line ever seen commits driving direction.
    #     if self.turn_dir == 0:
    #         self.turn_dir = +1 if close[0] == LINE_ORANGE else -1
    #         self.get_logger().info(
    #             f'direction set from {close[0]}: turn_dir={self.turn_dir}')

    #     if pose is None:
    #         return
    #     rx, ry, yaw = pose

    #     # Lidar-based corner trigger — fires when the inner-wall corner
    #     # is visible in the scan. Independent of line proximity.
    #     if self.turn_dir != 0 and self._inner_corner_visible():
    #         self.get_logger().info(
    #             f'lidar inner-corner visible, '
    #             f'turn_dir={self.turn_dir:+d})')
    #         # self._execute_corner_turn(rx, ry, yaw, close)
    #         # return

    #     # Straight-following: prefer the far line as the target; fall
    #     # back to the close one.
    #     target = far if far is not None else close
    #     kind = 'far' if far is not None else 'close'
    #     x, y = target[1], target[2]
    #     self.last_closest_line = close

    #     self.get_logger().info(f'{kind} waypoint: ({x:.2f}, {y:.2f})')
    #     self._publish_waypoint(x, y)

    # # ================================================================
    # # Corner trigger: lidar sees the inner-wall corner
    # # ================================================================
    # def _inner_corner_visible(self) -> bool:
    #     """True when the lidar sees an opening at the turn-side
    #     diagonal materially larger than the side-wall range — i.e.
    #     the robot is aligned with the inner wall's convex corner and
    #     Nav2 can plan around it. Guarded by a front-distance gate so
    #     far-away detections don't fire the trigger prematurely."""
    #     if self.turn_dir == 0:
    #         return False
    #     front = self.lidar.front_distance()
    #     if front is None or front > self.inner_corner_front_max_m:
    #         return False
    #     diag = self.lidar.median_in_arc(self.turn_dir * math.pi / 4.0)
    #     side = self.lidar.median_in_arc(self.turn_dir * math.pi / 2.0)
    #     if diag is None or side is None:
    #         return False
    #     return diag > side + self.inner_corner_diag_margin_m

    # ================================================================
    # Corner strategy: one waypoint offset from the estimated corner
    # ================================================================
    # def _execute_corner_turn(
    #     self,
    #     rx: float, ry: float, yaw: float,
    #     close: Optional[Tuple[str, float, float]],
    # ) -> None:
    #     # Snap outgoing yaw to the nearest cardinal — WRO straights run
    #     # along the map axes.
    #     new_yaw = yaw + self.turn_dir * math.pi / 2.0
    #     new_yaw = round(new_yaw / (math.pi / 2.0)) * (math.pi / 2.0)

    #     # Estimate the corner point from the lidar range at the
    #     # turn-side diagonal (the ray that hits the inner obstacle's
    #     # convex corner). Falls back to the front distance ahead of
    #     # the robot if the diagonal reading isn't available.
    #     corner_angle = self.turn_dir * math.pi / 4.0
    #     corner_range = self.lidar.median_in_arc(corner_angle)
    #     corner_axis = yaw + corner_angle
    #     if corner_range is None:
    #         corner_range = self.lidar.front_distance()
    #         corner_axis = yaw
    #     if corner_range is None:
    #         corner_range = 1.0
    #     cx = rx + corner_range * math.cos(corner_axis)
    #     cy = ry + corner_range * math.sin(corner_axis)

    #     # Waypoint = corner + LEAD along new heading
    #     #                   + LATERAL perpendicular on the safe side.
    #     # Safe side (away from the inner obstacle) is opposite the
    #     # turn direction relative to the new heading.
    #     lateral_yaw = new_yaw - self.turn_dir * math.pi / 2.0
    #     tx = (cx
    #           + self.corner_lead_m * math.cos(new_yaw)
    #           + self.corner_lateral_m * math.cos(lateral_yaw))
    #     ty = (cy
    #           + self.corner_lead_m * math.sin(new_yaw)
    #           + self.corner_lateral_m * math.sin(lateral_yaw))

    #     self.get_logger().info(
    #         f'corner (turn_dir={self.turn_dir:+d}) '
    #         f'estimated at ({cx:.2f}, {cy:.2f}) -> '
    #         f'({tx:.2f}, {ty:.2f}) yaw={math.degrees(new_yaw):.0f}° '
    #         f'[lead={self.corner_lead_m:.2f}m, '
    #         f'lateral={self.corner_lateral_m:.2f}m]')

    #     # Record for lap 2-3 replay.
    #     self._record_replay_target(tx, ty, new_yaw)

    #     self._advance_corner(rx, ry, yaw, new_yaw)
    #     if close is not None:
    #         self.last_closest_line = close

    #     # `_finish_lap` may have published the endgame target already.
    #     if self.state != State.DRIVING:
    #         return
    #     # Stash the corner target so on_arrived can K-turn recover if
    #     # Nav2 reports it unreachable. Reset the retry counter for the
    #     # new corner.
    #     self._pending_corner = (tx, ty, new_yaw)
    #     self._recovery_stage = 0
    #     self._corner_retries = 0
    #     self._publish_waypoint(tx, ty, yaw=new_yaw)

    # # ================================================================
    # # Endgame: return to origin
    # # ================================================================
    # def _finish_lap(self) -> None:
    #     self._transition(State.RETURNING_HOME)
    #     self.get_logger().info(
    #         f'lap complete — returning to origin. '
    #         f'corners={self.corners_done}')
    #     self._publish_waypoint(0.0, 0.0, yaw=0.0)

    # # ================================================================
    # # Waypoint publishing
    # # ================================================================
    # def _publish_waypoint(
    #     self, x: float, y: float, yaw: Optional[float] = None,
    # ) -> None:
    #     # Skip if the new target is within 5 cm of the last one.
    #     if self.last_waypoint is not None:
    #         rx, ry = self.last_waypoint
    #         if math.hypot(y - ry, x - rx) <= 0.05:
    #             self.get_logger().info('Too close to publish new waypoint')
    #             return

    #     # Default yaw: face the direction of travel.
    #     if yaw is None:
    #         pose = self._robot_pose()
    #         if pose is not None:
    #             rx, ry, _ = pose
    #             yaw = math.atan2(y - ry, x - rx)
    #         else:
    #             yaw = 0.0

    #     p = PoseStamped()
    #     p.header.frame_id = 'map'
    #     p.header.stamp = self.get_clock().now().to_msg()
    #     p.pose.position.x = float(x)
    #     p.pose.position.y = float(y)
    #     p.pose.orientation.z = math.sin(yaw / 2.0)
    #     p.pose.orientation.w = math.cos(yaw / 2.0)

    #     self.waiting_for_arrival = True
    #     self.pub.publish(p)
    #     self.last_waypoint = (x, y)
    #     self.get_logger().info(
    #         f'published waypoint: ({x:.2f}, {y:.2f}) '
    #         f'yaw={math.degrees(yaw):.0f}°')

    # # ================================================================
    # # Corner counting + replay
    # # ================================================================
    # def _advance_corner(
    #     self, x: float, y: float,
    #     incoming_yaw: float, outgoing_yaw: float,
    # ) -> None:
    #     """Count a completed corner. `_finish_lap` fires when count
    #     hits the race total."""
    #     self.corners_done += 1
    #     self.get_logger().info(
    #         f'corner {self.corners_done}/{CORNERS_PER_RACE} '
    #         f'at ({x:.2f}, {y:.2f}) '
    #         f'yaw {math.degrees(incoming_yaw):.0f}° -> '
    #         f'{math.degrees(outgoing_yaw):.0f}°')
    #     if self.corners_done >= CORNERS_PER_RACE:
    #         self._finish_lap()

    # def _record_replay_target(
    #     self, tx: float, ty: float, yaw: float,
    # ) -> None:
    #     """Store one corner's target for replay on laps 2-3."""
    #     if len(self._corner_history) < CORNERS_PER_LAP:
    #         self._corner_history.append((tx, ty, yaw))
    #         if len(self._corner_history) == CORNERS_PER_LAP:
    #             summary = ', '.join(
    #                 f'({rx:.2f}, {ry:.2f})'
    #                 for rx, ry, _ in self._corner_history)
    #             self.get_logger().info(
    #                 f'lap 1 mapped — replay targets: {summary}')

    # def _dispatch_replay(self) -> None:
    #     """Publish the next pre-recorded corner target."""
    #     idx = (self.corners_done - CORNERS_PER_LAP) % CORNERS_PER_LAP
    #     tx, ty, tyaw = self._corner_history[idx]
    #     self.corners_done += 1
    #     self.get_logger().info(
    #         f'replay corner {self.corners_done}/{CORNERS_PER_RACE} '
    #         f'-> ({tx:.2f}, {ty:.2f}) yaw={math.degrees(tyaw):.0f}°')
    #     if self.corners_done >= CORNERS_PER_RACE:
    #         self._finish_lap()
    #         return
    #     self._publish_waypoint(tx, ty, yaw=tyaw)

    # # ================================================================
    # # Arrival + start-button callbacks
    # # ================================================================
    def on_arrived(self, msg: Bool) -> None:
        pass
    #     if not self.waiting_for_arrival:
    #         return
    #     self.waiting_for_arrival = False

    #     # -----------------------------------------------------------------
    #     # Failure path: K-turn recovery for corners.
    #     # -----------------------------------------------------------------
    #     if not msg.data:
    #         if (self._pending_corner is not None
    #                 and self._corner_retries < self.corner_recovery_max_retries):
    #             self._attempt_corner_recovery()
    #             return
    #         # Give up recovery: clear state and try normal dispatch.
    #         if self._pending_corner is not None:
    #             self.get_logger().warn(
    #                 f'corner recovery exhausted after '
    #                 f'{self._corner_retries} retries — continuing')
    #         self._pending_corner = None
    #         self._recovery_stage = 0
    #         self._corner_retries = 0
    #         self.get_logger().warn(
    #             'waypoint failed — recomputing next target')
    #         self._send_next_waypoint()
    #         return

    #     # -----------------------------------------------------------------
    #     # Success path.
    #     # -----------------------------------------------------------------

    #     # Backup leg of a K-turn arrived → re-publish the stashed corner.
    #     if self._recovery_stage == 1 and self._pending_corner is not None:
    #         tx, ty, new_yaw = self._pending_corner
    #         self.get_logger().warn(
    #             f'corner recovery: backup complete, re-attempting corner '
    #             f'({tx:.2f}, {ty:.2f}) yaw={math.degrees(new_yaw):.0f}°')
    #         self._recovery_stage = 2
    #         # Bypass the 5cm dedup so re-publish always goes out.
    #         self.last_waypoint = None
    #         self._publish_waypoint(tx, ty, yaw=new_yaw)
    #         return

    #     # Corner re-attempt succeeded → clear recovery state.
    #     if self._recovery_stage == 2:
    #         self.get_logger().info(
    #             f'corner succeeded after {self._corner_retries} '
    #             f'K-turn recovery attempt(s)')

    #     # Any successful arrival clears the pending corner — either it
    #     # was the corner itself succeeding, or a later waypoint means
    #     # we've moved past it and no longer want to retry it.
    #     self._pending_corner = None
    #     self._recovery_stage = 0
    #     self._corner_retries = 0

    #     if self.state == State.RETURNING_HOME:
    #         self._transition(State.DONE)
    #         self.get_logger().info(
    #             f'FINISHED (home) corners={self.corners_done}')
    #         return

    #     self._send_next_waypoint()

    # def _attempt_corner_recovery(self) -> None:
    #     """Publish a straight-line backup waypoint, then wait for
    #     arrival before re-publishing the stashed corner. K-turn
    #     recovery relies on Nav2's REEDS_SHEPP motion primitives (sim)
    #     or a similar Ackermann-with-reverse mode being available; if
    #     the base planner refuses to reverse, the backup will itself
    #     fail and we'll exhaust retries."""
    #     pose = self._robot_pose()
    #     if pose is None:
    #         # No pose, no recovery. Fall through to normal retry.
    #         self.get_logger().warn(
    #             'corner recovery: pose unavailable, cannot back up')
    #         self._pending_corner = None
    #         self._recovery_stage = 0
    #         self._corner_retries = 0
    #         self._send_next_waypoint()
    #         return
    #     rx, ry, yaw = pose
    #     back_x = rx - self.corner_recovery_backup_m * math.cos(yaw)
    #     back_y = ry - self.corner_recovery_backup_m * math.sin(yaw)
    #     self._corner_retries += 1
    #     self._recovery_stage = 1
    #     self.get_logger().warn(
    #         f'corner failed — K-turn retry '
    #         f'{self._corner_retries}/{self.corner_recovery_max_retries}: '
    #         f'backing up {self.corner_recovery_backup_m:.2f}m '
    #         f'to ({back_x:.2f}, {back_y:.2f})')
    #     # Bypass dedup: a backup waypoint may be within 5cm of the
    #     # last-published corner target if the robot barely moved.
    #     self.last_waypoint = None
    #     # Keep the CURRENT yaw so Nav2 plans a straight reverse rather
    #     # than a curved one.
    #     self._publish_waypoint(back_x, back_y, yaw=yaw)

    def on_start_button(self, msg: Bool) -> None:
        if msg.data and not self.start_pressed:
            self.start_pressed = True
            self.get_logger().info('start button pressed')

    # # ================================================================
    # # First-corner guard — pick turn_dir from lidar if no line ever seen
    # # ================================================================
    # def _first_corner_guard(self) -> None:
    #     if self.turn_dir != 0:
    #         return
    #     front = self.lidar.front_distance()
    #     if front is None or front >= self.first_corner_front_m:
    #         return
    #     if self.camera.line_detections_total > 0:
    #         return

    #     left = self.lidar.left_min_distance()
    #     right = self.lidar.right_min_distance()
    #     if left is None or right is None:
    #         self.get_logger().warn(
    #             'first-corner guard: no lines and lidar left/right unavailable')
    #         return
    #     self.turn_dir = +1 if left > right else -1
    #     self.get_logger().warn(
    #         f'first-corner guard: no lines; turn '
    #         f'{"left" if self.turn_dir > 0 else "right"} '
    #         f'(left={left:.2f}m, right={right:.2f}m)')
    #     self._send_next_waypoint()

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
    node = OpenMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
