"""WRO Open Challenge — starter skeleton.

Empty node. Subscribes to sensors, publishes /cmd_vel, does nothing useful.
Fill in `step()` with your control logic.

Run in sim:
  ros2 launch wro_sim sim.launch.py rviz:=true
  ros2 run wro_behavior open_challenge_template

Rebuild after every edit:
  colcon build --packages-select wro_behavior
  source install/setup.bash
"""
import math

# ============================================================================
# ROS 2 imports
# ============================================================================
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Sensor messages you'll receive:
#   LaserScan  — lidar ring; ranges + angles
#   Odometry   — pose and twist from the drive plugin (has yaw in quaternion)
#   Imu        — angular velocity + linear acceleration (higher rate than odom)
#   Image      — camera image (for future pillar detection)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# The message you'll SEND to drive the robot:
#   Twist.linear.x  = forward speed in m/s   (positive = forward)
#   Twist.angular.z = yaw rate in rad/s      (positive = turn LEFT / CCW)
# Anything else in Twist is ignored — no strafing on Ackermann.
from geometry_msgs.msg import Twist


# ============================================================================
# Tuning constants — edit these, don't hardcode numbers in the logic below.
# ============================================================================
CRUISE_SPEED = 0.25            # m/s
KP = 4.0                       # gain on wall-follow error
MAX_ANGULAR_Z = 1.5            # rad/s clamp
TARGET_WALL_DIST = 0.25        # m; if you follow one wall
CORNERS_PER_RACE = 12          # 4 corners × 3 laps
CORNER_THRESHOLD = 0.6

# ============================================================================
# Helpers
# ============================================================================
def median_in_arc(scan: LaserScan, center_rad: float,
                  half_width_rad: float = math.radians(8)) -> float:
    """Median finite range within a small arc.

    center_rad is the angle in the lidar's frame:
      0        = straight forward (+X in base_link)
      +pi/2    = left side  (+Y)
      -pi/2    = right side (-Y)
      +pi      = straight back
    half_width_rad picks how wide the arc is (default 8° each side = 16° total).
    Returns scan.range_max if every beam in the arc is inf/nan.
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


def yaw_from_quaternion(q) -> float:
    """Extract yaw (rotation about Z) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ============================================================================
# The node
# ============================================================================
class OpenChallenge(Node):
    def __init__(self):
        super().__init__('open_challenge_template')

        self.last_left = None
        self.last_right = None
        # Sensor topics use "sensor QoS": BEST_EFFORT reliability, small depth.
        # /cmd_vel uses default (RELIABLE) — Twist is small and infrequent.
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- Subscribers ----
        # Callback fires whenever a new message arrives on the topic.
        self.create_subscription(LaserScan, '/scan', self.on_scan, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # self.create_subscription(Imu, '/imu/data_raw', self.on_imu, sensor_qos)

        # ---- Publisher ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- State you store between ticks ----
        self.latest_scan = None    # most recent LaserScan
        self.yaw = 0.0             # most recent yaw from /odom, radians
        self.pose_x = 0.0          # most recent x in odom frame
        self.pose_y = 0.0          # most recent y in odom frame
        self.state = 'INIT'        # your state-machine label
        self.corners_done = 0

        # ---- Timer ----
        # step() runs every 0.05 s (20 Hz). This is your control loop.
        self.create_timer(0.05, self.step)

        self.get_logger().info('node up — waiting for /scan')

    # ------------------------------------------------------------------------
    # Subscription callbacks. Keep these SHORT — just store data. Do the
    # real work in step(), which runs on the fixed timer.
    # ------------------------------------------------------------------------
    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def on_odom(self, msg: Odometry) -> None:
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    # ------------------------------------------------------------------------
    # The control loop. Fires 20 times per second.
    # ------------------------------------------------------------------------
    def step(self) -> None:
        # Bail out until we have data.
        if self.latest_scan is None:
            return
        scan = self.latest_scan

        # ---- Sensor readings you probably want ----
        # front, left, right — median range in a small arc.
        front = median_in_arc(scan, 0.0)
        left = median_in_arc(scan, math.pi / 2)
        right = median_in_arc(scan, -math.pi / 2)
        # Diagonals often help for smoother wall-following:
        # front_left  = median_in_arc(scan, math.radians(45))
        # front_right = median_in_arc(scan, math.radians(-45))

        # ---- Debug print. Prints 20 lines/sec. Use flush=True. ----
        print(f"[{self.state}] left={left:.2f} right={right:.2f} "
              f"front={front:.2f} yaw={math.degrees(self.yaw):.1f} "
              f"corners={self.corners_done}", flush=True)

        # ---- Build the command you'll publish ----
        # Twist default is zeros — safe if you forget to set something.
        cmd = Twist()

        if (left > self.last_left + LEFT_DROP_VALUE):


        # ==================================================================
        #  YOUR LOGIC GOES HERE.
        #
        #  Cheat sheet:
        #    cmd.linear.x  = forward m/s     (0 = stop, CRUISE_SPEED = go)
        #    cmd.angular.z = yaw rad/s       (+ = turn left, - = turn right)
        #    max(-MAX_ANGULAR_Z, min(MAX_ANGULAR_Z, x))     # clamp
        #    self.state = 'NEXT_STATE'       # transition
        #    self.corners_done += 1          # count something
        #    self.get_logger().info("...")   # official ROS log (with stamp)
        #    print("...", flush=True)        # plain stdout
        #    self.get_clock().now()          # sim-time-aware timestamp
        #    (t2 - t1).nanoseconds / 1e9     # elapsed seconds between Time
        # ==================================================================

        self.last_left = left
        self.last_right = right
        # (skeleton: publish zeros so the robot doesn't drive.)
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
