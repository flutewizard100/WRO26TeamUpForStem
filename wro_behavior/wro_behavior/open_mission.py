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
    FIND_FIRST_CORNER = 2
    FIRST_CORNER = 3   
    FIND_NEXT_CORNER = 4
    NEXT_CORNER = 5
    NEXT_LAPS = 6
    RETURNING_HOME = 7        # endgame — driving back to (0, 0)
    DONE = 8                  # mission complete, robot stopped

class OpenMission(Node):
    def __init__(self):
        super().__init__('open_mission')

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
        # Recorded corner targets from lap 1, replayed on laps 2-3.
        self._corner_history: list = []

        # Button state
        self.declare_parameter('wait_for_start', True)
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
        # Clear dedup memory so the next state's first waypoint isn't
        # silently blocked for being within 5cm of the previous state's.
        self.last_waypoint = None

    # ================================================================
    # Main tick — dispatches by state
    # ================================================================
    def _tick(self) -> None:
        if self.state == State.NOT_READY:
            self._tick_not_ready()
        elif self.state == State.WAITING_FOR_START:
            self._tick_waiting_for_start()
        elif self.state == State.FIND_FIRST_CORNER:
            self._tick_find_first_corner()
        elif self.state == State.FIRST_CORNER:
            self._tick_first_corner()
        elif self.state == State.FIND_NEXT_CORNER:
            self._tick_find_next_corner()
        elif self.state == State.NEXT_CORNER:
            self._tick_next_corner()
        elif self.state == State.NEXT_LAPS:
            self._tick_next_laps()
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
        self._transition(State.FIND_FIRST_CORNER)

    def _tick_find_first_corner(self) -> None:
        if (self.waiting_for_arrival):
            return
        pose = self._robot_pose()
        if pose is None:
            return
        front = self.lidar.front_distance()
        left = self.lidar.left_min_distance()
        right = self.lidar.right_min_distance()
        angle = self.lidar.longest_forward_ray_angle()

        # now we check if we know the direction
        lines = self.camera.detect_direction()
        if lines is not None:
            close, far = lines
            self.turn_dir = +1 if close[0] == LINE_ORANGE else -1
            self._transition(State.FIRST_CORNER)
            return
        if angle is not None and abs(angle) > math.pi / 18:
            # if the widest-opening angle is greater than 10 degrees
            # then I see a corner
            self.turn_dir = +1 if angle > 0 else -1
            self._transition(State.FIRST_CORNER)
            return
        # Neither line nor lidar-corner yet — keep creeping forward
        # (short hop) and try again next tick.
        dyn = self.lidar.forward_edge_waypoint(pose, max_distance=0.3)
        if dyn is None:
            self.get_logger().info(
                f'no line in view (turn_dir={self.turn_dir}) — '
                f'no progress can be made')
            return
        self._publish_waypoint(*dyn)


    def _tick_first_corner(self) -> None:
        # One-shot: don't republish while Nav2 is still driving toward
        # the current goal. on_arrived clears the flag when Nav2 reports
        # success or failure.
        if self.waiting_for_arrival:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        rx, ry, yaw = pose
        front = self.lidar.front_distance()
        if front is None:
            return
        # Pick the wall on the turn side.
        side_dist = self.lidar.left_min_distance() if self.turn_dir > 0 else self.lidar.right_min_distance()
        if side_dist is None:
            return
        # Offsets in the robot's own frame.
        # Lateral target: 0.38 m from the turn-side wall (i.e. hugging
        # the OUTER wall by ~0.22 m, giving room for the arc).
        f = front - 0.6                         # metres forward
        l = self.turn_dir * (side_dist - 0.38)      # +left / -right
        # Project into map frame.
        wp_x = rx + f * math.cos(yaw) - l * math.sin(yaw)
        wp_y = ry + f * math.sin(yaw) + l * math.cos(yaw)
        # Explicit goal heading: 90° in turn_dir, snapped to the nearest
        # cardinal so drift doesn't accumulate. Nav2 plans an Ackermann
        # arc that finishes in this heading.
        new_yaw = yaw + self.turn_dir * math.pi / 2.0
        new_yaw = round(new_yaw / (math.pi / 2.0)) * (math.pi / 2.0)
        # _publish_waypoint sets self.last_waypoint after the actual
        # publish; don't set it here or the 5cm dedup skips us.
        self._publish_waypoint(wp_x, wp_y, new_yaw)


    def _tick_find_next_corner(self) -> None:
        if (self.waiting_for_arrival):
            return
        pose = self._robot_pose()
        if pose is None:
            return
        rx, ry, yaw = pose
        front = self.lidar.front_distance()

        # Lateral hug: bias every forward hop toward the outer wall so
        # the robot arrives at the corner with room for the arc. Same
        # 0.38 m target from the turn-side wall used at the corner.
        side_dist = self.lidar.left_min_distance() if self.turn_dir > 0 else self.lidar.right_min_distance()
        l = self.turn_dir * (side_dist - 0.38) if side_dist is not None else 0.0

        if front is None:
            # No front reading yet — small forward creep.
            step = 0.3
        elif front > 1:
            # Big hop: aim 1 m short of the wall ahead.
            step = front - 1.0
        else:
            # Wall is close → hand off to NEXT_CORNER.
            self._transition(State.NEXT_CORNER)
            return

        # Project (forward, lateral) into map frame.
        wp_x = rx + step * math.cos(yaw) - l * math.sin(yaw)
        wp_y = ry + step * math.sin(yaw) + l * math.cos(yaw)
        self._publish_waypoint(wp_x, wp_y)


    def _tick_next_corner(self) -> None:
        # Same shape as _tick_first_corner: read the walls, offset
        # forward and toward the turn side, project into map frame.
        if self.waiting_for_arrival:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        rx, ry, yaw = pose
        front = self.lidar.front_distance()
        if front is None:
            return
        side_dist = self.lidar.left_min_distance() if self.turn_dir > 0 else self.lidar.right_min_distance()
        if side_dist is None:
            return
        # Offsets in the robot's own frame.
        # Same 0.38 m hug target as _tick_first_corner and the straight.
        f = front - 0.6                             # metres forward
        l = self.turn_dir * (side_dist - 0.38)      # +left / -right
        # Project into map frame.
        wp_x = rx + f * math.cos(yaw) - l * math.sin(yaw)
        wp_y = ry + f * math.sin(yaw) + l * math.cos(yaw)
        # Explicit goal heading: 90° in turn_dir, snapped to cardinal.
        new_yaw = yaw + self.turn_dir * math.pi / 2.0
        new_yaw = round(new_yaw / (math.pi / 2.0)) * (math.pi / 2.0)
        self._publish_waypoint(wp_x, wp_y, new_yaw)
            
            


    def _tick_next_laps(self) -> None:
        pass

    def _send_next_waypoint(self) -> None:
        # Stub — the previous line-following body is commented out
        # below. Leaves callers (on_arrived, _attempt_corner_recovery,
        # _first_corner_guard) safe to invoke while the new state
        # machine is being built.
        if self.state == State.FIND_FIRST_CORNER:
            pose = self._robot_pose()
            if pose is None:
                return
            dyn = self.lidar.forward_edge_waypoint(pose, max_distance=0.3)
            if dyn is None:
                self.get_logger().info(
                    f'no line in view (turn_dir={self.turn_dir}) — '
                    f'no progress can be made')
                return
            self._publish_waypoint(*dyn)

    # ================================================================
    # Endgame: return to origin
    # ================================================================
    def _finish_lap(self) -> None:
        self._transition(State.RETURNING_HOME)
        self.get_logger().info(
            f'lap complete — returning to origin. '
            f'corners={self.corners_done}')
        self._publish_waypoint(0.0, 0.0, yaw=0.0)

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
                self.get_logger().info(
                    f'Too close to publish new waypoint: ({x:.2f}, {y:.2f})')
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
    # Arrival + start-button callbacks
    # ================================================================
    def on_arrived(self, msg: Bool) -> None:
        # Always clear the in-flight flag first — every tick guards on
        # it, so leaving it True would deadlock the mission.
        self.waiting_for_arrival = False

        if self.state == State.FIND_FIRST_CORNER:
            if not msg.data:
                # TODO plan recovery
                # failed to reach destination
                return

        if self.state == State.FIRST_CORNER:
            if not msg.data:
                # TODO plan recovery
                # failed to reach destination
                return
            if self.last_waypoint is None:
                # Should never happen (arrival implies we published),
                # but guard anyway so we don't TypeError on unpack.
                self._transition(State.FIND_NEXT_CORNER)
                return
            wp_x, wp_y = self.last_waypoint
            self._corner_history.append([wp_x, wp_y])
            self.corners_done += 1
            self.get_logger().info(
                f'first-corner arrived at ({wp_x:.2f}, {wp_y:.2f}, {self.corners_done}) '
                f'[turn_dir={self.turn_dir:+d}]')
            self._transition(State.FIND_NEXT_CORNER)

        if self.state == State.FIND_NEXT_CORNER:
            pass

        if self.state == State.NEXT_CORNER:
            if not msg.data:
                # TODO plan recovery
                # failed to reach destination
                return
            if self.last_waypoint is None:
                # Should never happen (arrival implies we published),
                # but guard anyway so we don't TypeError on unpack.
                self._transition(State.FIND_NEXT_CORNER)
                return
            wp_x, wp_y = self.last_waypoint
            self._corner_history.append([wp_x, wp_y])
            self.corners_done += 1
            self.get_logger().info(
                f'next-corner arrived at ({wp_x:.2f}, {wp_y:.2f}) '
                f'[turn_dir={self.turn_dir:+d}, corners_done={self.corners_done}]')
            if self.corners_done == 4:
                self._transition(State.NEXT_LAPS)
            else:
                self._transition(State.FIND_NEXT_CORNER)

    def on_start_button(self, msg: Bool) -> None:
        if msg.data and not self.start_pressed:
            self.start_pressed = True
            self.get_logger().info('start button pressed')

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
