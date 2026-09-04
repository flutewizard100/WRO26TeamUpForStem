"""WRO Open Challenge — 3 autonomous laps around the field.

Pure reactive wall-follower + corner detector + lap counter. No Nav2, no
map, no AMCL. Subscribes to /scan, publishes /cmd_vel. Runs identically
on sim (against wro_sim/sim.launch.py) and real hardware
(WRORobot/launch/hardware.launch.py).

State machine:
  INIT           — one scan to decide follow direction (CW vs CCW)
  LANE_FOLLOW    — P controller on distance to chosen side wall
  CORNERING      — fixed arc until front distance opens back up
  STOP           — 12 corners done, cmd_vel = 0

Tune these constants on the real robot before the run.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# ----- tuning knobs. Change one at a time; run three times per change. -----
CRUISE_SPEED = 0.25          # m/s forward on straights
CORNER_SPEED = 0.15          # m/s forward while cornering
CORNER_THRESHOLD = 0.55      # m; front distance that triggers a corner
CORNER_EXIT_FRONT = 0.8      # m; front distance that ends the corner
CORNER_ANGULAR_Z = 1.2       # rad/s; magnitude of yaw command in corner
TARGET_WALL_DIST = 0.25      # m; how far to hold off the followed wall
KP = 2.0                     # gain on wall-distance error
MAX_ANGULAR_Z = 1.5          # rad/s; clamp on commanded yaw rate
CORNERS_PER_RACE = 12        # 4 corners × 3 laps
CORNER_MIN_DURATION_S = 1.25 # s; ~86° at 1.2 rad/s — full 90° with a tiny safety margin
# ----- end tuning knobs. --------------------------------------------------


def median_in_arc(scan: LaserScan, center_rad: float,
                  half_width_rad: float = math.radians(8)) -> float:
    """Median finite range within a small arc centered at center_rad.

    Uses median rather than min/mean to be robust to a stray inf or a
    single spurious close reading.
    """
    idx_center = (center_rad - scan.angle_min) / scan.angle_increment
    idx_half = half_width_rad / scan.angle_increment
    lo = max(0, int(idx_center - idx_half))
    hi = min(len(scan.ranges), int(idx_center + idx_half) + 1)
    vals = [r for r in scan.ranges[lo:hi]
            if math.isfinite(r) and scan.range_min < r < scan.range_max]
    if not vals:
        return scan.range_max
    return sorted(vals)[len(vals) // 2]


class OpenChallenge(Node):
    def __init__(self):
        super().__init__('open_challenge')

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, sensor_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.state = 'INIT'
        self.direction = 1        # +1 = follow left wall (CCW); -1 = right (CW)
        self.corners_done = 0
        self.latest_scan = None
        self.corner_start_time = None
        self.log_counter = 0

        self.create_timer(0.05, self.step)  # 20 Hz control loop
        self.get_logger().info('open_challenge up — waiting for /scan')

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def step(self) -> None:
        print(f"[tick] state={self.state} corners={self.corners_done}", flush=True)

        if self.latest_scan is None:
            print("  waiting for /scan", flush=True)
            return
        scan = self.latest_scan

        front = median_in_arc(scan, 0.0)
        left = median_in_arc(scan, math.pi / 2)
        right = median_in_arc(scan, -math.pi / 2)

        print(f"  [{self.state}] left={left:.2f} right={right:.2f} "
              f"front={front:.2f}", flush=True)

        cmd = Twist()

        if self.state == 'INIT':
            # Follow the wall we're already closer to. Robust to spawn side.
            self.direction = +1 if left < right else -1
            side = 'left' if self.direction == +1 else 'right'
            self.get_logger().info(
                f"direction={self.direction} ({side} wall)  "
                f"l={left:.2f} r={right:.2f} f={front:.2f}")
            self.state = 'LANE_FOLLOW'

        elif self.state == 'LANE_FOLLOW':
            # Symmetric follower: try to be equidistant from left and right walls.
            # error > 0 → more room on left → turn LEFT (positive angular.z)
            # error < 0 → more room on right → turn RIGHT
            # When one wall disappears at a corner, its distance jumps up,
            # error jumps hard toward that side, and the robot turns into
            # the open corridor.
            error = left - right
            steer = KP * error
            cmd.linear.x = CRUISE_SPEED
            cmd.angular.z = max(-MAX_ANGULAR_Z, min(MAX_ANGULAR_Z, steer))

        elif self.state == 'STOP':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = OpenChallenge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
