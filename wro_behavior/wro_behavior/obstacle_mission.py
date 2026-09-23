"""WRO Obstacle Challenge mission node.

Obstacle Challenge specifics vs the shared base:

  - Corner strategy: TWO-STAGE geometric arc.
      Stage 1 — arc anchor on the 45° bisector, forcing Nav2's planner
                to sweep through the corner instead of a hard 90° bend.
      Stage 2 — final leg past the corner, landing on the next straight.
    Arc radius fitted to lidar clearance; anchor distance scaled by
    actual delta yaw so partial turns (drift compensation) work.

  - Pillar avoidance: on straight-following, nudge the target laterally
    if a red/green pillar is between the robot and the waypoint.
    Rule: red passes on robot's right (waypoint offset LEFT of pillar);
    green passes on robot's left (waypoint offset RIGHT of pillar).

  - Endgame: PARKING state (currently a placeholder — transitions to
    DONE without a proper parking pose; wire up when field-frame
    alignment is available).

Run:
  ros2 run wro_behavior obstacle_mission
"""
import math
from typing import Optional, Tuple

import rclpy

from wro_behavior.mission_base import (
    CORNER_TURN_DIST_M, FIRST_CORNER_FRONT_M,
    MissionBase, State,
)
from wro_behavior.camera_perception import PILLAR_RED


# Ackermann arc floor — matches nav2_params_sim.yaml `min_turning_r`.
MIN_TURNING_RADIUS_M = 0.35
# Safety margin between the arc and the wall it's skirting.
CORNER_SAFETY_MARGIN_M = 0.15

# How far off-path to swing when avoiding a pillar.
PILLAR_PASS_OFFSET = 0.30


class ObstacleMission(MissionBase):
    NODE_NAME = 'obstacle_mission'

    def __init__(self):
        super().__init__()
        # Two-stage corner state:
        #   0 = normal line-following / straight
        #   1 = arc-anchor waypoint sent, awaiting arrival
        #   2 = final-leg waypoint sent, awaiting arrival
        self._corner_stage = 0
        # (tx, ty, new_yaw) stashed at corner trigger, dispatched on
        # stage-1 arrival (recomputed from arc-anchor pose then).
        self._corner_final: Optional[Tuple[float, float, float]] = None

    # ------------------------------------------------------------------
    # Pre-dispatch: handle stage-1 / stage-2 arrivals
    # ------------------------------------------------------------------
    def _handle_pre_dispatch(
        self, pose: Optional[Tuple[float, float, float]],
    ) -> bool:
        # Stage-1 arrival → dispatch stage-2 final leg. Recompute from
        # the CURRENT (arc-anchor) pose so the final target lies on the
        # actual arc trajectory continuation, not the pre-arc pose we
        # stashed.
        if self._corner_stage == 1 and self._corner_final is not None:
            _, _, new_yaw = self._corner_final
            if pose is None:
                return True
            arc_x, arc_y, _ = pose
            step = self._turn_step_m()
            tx = arc_x + step * math.cos(new_yaw)
            ty = arc_y + step * math.sin(new_yaw)
            self._corner_stage = 2

            # Record the stage-2 target for lap-1 corners.
            self._record_replay_target(tx, ty, new_yaw)

            self.get_logger().info(
                f'corner turn stage 2 -> final ({tx:.2f}, {ty:.2f}) '
                f'yaw={math.degrees(new_yaw):.0f}°')
            self._publish_waypoint(tx, ty, yaw=new_yaw)
            return True

        # Stage-2 arrival → done with the corner; resume normal flow.
        if self._corner_stage == 2:
            self._corner_stage = 0
            self._corner_final = None
        return False

    # ------------------------------------------------------------------
    # Corner strategy: two-stage arc
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Straight-target adjust: pillar detour
    # ------------------------------------------------------------------
    def _adjust_straight_target(
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

    # ------------------------------------------------------------------
    # Endgame: parking (placeholder until field-frame calibration is done)
    # ------------------------------------------------------------------
    def _finish_lap(self) -> None:
        self._transition(State.DONE)
        self.get_logger().info(
            f'FINISHED (obstacle: parking placeholder) '
            f'corners={self.corners_done}')

    # ------------------------------------------------------------------
    # Arc geometry helpers
    # ------------------------------------------------------------------
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
