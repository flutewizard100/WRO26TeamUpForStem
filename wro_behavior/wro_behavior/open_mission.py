"""WRO Open Challenge mission node.

Open Challenge specifics vs the shared base:

  - Corner strategy: SINGLE waypoint on the middle of the next straight.
    No two-stage arc — Nav2's SmacPlannerHybrid (Reeds-Shepp) plans the
    ackermann curve itself. Step distance adapts to corridor width from
    lidar left+right readings.

  - Endgame: return to (0, 0) via the RETURNING_HOME state.

  - No pillar avoidance (Open Challenge has no traffic signs).

Run:
  ros2 run wro_behavior open_mission
"""
import math
from typing import Optional, Tuple

import rclpy

from wro_behavior.mission_base import (
    CORNERS_PER_LAP, MissionBase, State,
)


# Corridor-width thresholds and step distances for the corner target.
# Narrow (600 mm) → shorter step to avoid overshooting the far wall.
# Wide (1000 mm) → longer step so the target sits well into the next
# straight.
NARROW_CORRIDOR_THRESHOLD_M = 0.8
STEP_NARROW_M = 0.7
STEP_WIDE_M = 1.2


class OpenMission(MissionBase):
    NODE_NAME = 'open_mission'

    # ------------------------------------------------------------------
    # Corner strategy: single waypoint on the next straight's centerline
    # ------------------------------------------------------------------
    def _execute_corner_turn(
        self,
        rx: float, ry: float, yaw: float,
        close: Optional[Tuple[str, float, float]],
    ) -> None:
        # Snap outgoing yaw to the nearest cardinal — WRO straights run
        # along the map axes.
        new_yaw = yaw + self.turn_dir * math.pi / 2
        new_yaw = round(new_yaw / (math.pi / 2)) * (math.pi / 2)

        # Adapt step distance to the corridor width. Both left and right
        # lidar readings may be None (before scan) — fall back to a
        # midding step.
        left = self.lidar.left_min_distance()
        right = self.lidar.right_min_distance()
        if left is None or right is None:
            step = STEP_WIDE_M
            corridor = None
        else:
            corridor = left + right
            step = (STEP_WIDE_M if corridor > NARROW_CORRIDOR_THRESHOLD_M
                    else STEP_NARROW_M)

        tx = rx + step * math.cos(new_yaw)
        ty = ry + step * math.sin(new_yaw)

        corridor_str = (f'{corridor:.2f}m' if corridor is not None
                        else 'unknown')
        self.get_logger().info(
            f'corner (turn_dir={self.turn_dir:+d}, '
            f'corridor={corridor_str}, step={step:.2f}m) -> '
            f'next-straight ({tx:.2f}, {ty:.2f}) '
            f'yaw={math.degrees(new_yaw):.0f}°')

        # Record for lap 2-3 replay.
        self._record_replay_target(tx, ty, new_yaw)

        # Count the corner. `_finish_lap` fires if we've hit the total.
        self._advance_corner(rx, ry, yaw, new_yaw)
        if close is not None:
            self.last_closest_line = close

        # If the corner count triggered `_finish_lap`, the state has
        # already changed and the endgame waypoint is published — don't
        # overwrite it with our corner target.
        if self.state != State.DRIVING:
            return
        self._publish_waypoint(tx, ty, yaw=new_yaw)

    # ------------------------------------------------------------------
    # Endgame: return to origin
    # ------------------------------------------------------------------
    def _finish_lap(self) -> None:
        self._transition(State.RETURNING_HOME)
        self.get_logger().info(
            f'lap complete — returning to origin. '
            f'corners={self.corners_done}')
        self._publish_waypoint(0.0, 0.0, yaw=0.0)


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
