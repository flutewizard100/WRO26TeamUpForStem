import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan          # FIX 1: was missing
from std_msgs.msg import Bool

WAYPOINTS = [
    (2.7, 0.3),
    (2.7, 2.7),
    (0.3, 2.7),
    (0.3, 0.3),
]


def median_in_arc(scan: LaserScan, center_rad, half_width_rad=math.radians(8)):
    idx_center = (center_rad - scan.angle_min) / scan.angle_increment
    idx_half = half_width_rad / scan.angle_increment
    lo = max(0, int(idx_center - idx_half))
    hi = min(len(scan.ranges), int(idx_center + idx_half) + 1)
    vals = [r for r in scan.ranges[lo:hi] if math.isfinite(r)]
    if not vals:
        return scan.range_max
    return sorted(vals)[len(vals) // 2]


class OpenNav(Node):
    def __init__(self):
        super().__init__('my_logic')
        self.detections = []                   # FIX 2: was never set
        self.pub = self.create_publisher(PoseStamped, '/waypoint', 10)
        self.create_subscription(Bool, '/waypoint_arrived', self.on_arrived, 10)
        self.i = 0
        self.create_timer(2.0, self.kick)

    def kick(self):
        self._timers[0].cancel()
        self.send(*WAYPOINTS[self.i])

    def on_arrived(self, msg):
        if msg.data:
            self.i += 1
            if self.i < len(WAYPOINTS):
                self.send(*WAYPOINTS[self.i])

    def getDirection(self):
        for i in self.detections:
            if i.results[0].hypothesis.class_id == 'CW_Corner':
                return 'CW_Corner'
            if i.results[0].hypothesis.class_id == 'CCW_Corner':
                return 'CCW_Corner'

    def findStartPose(self):
        front = median_in_arc(scan, 0.0)
        left = median_in_arc(scan, 7 * math.pi / 18)
        right = median_in_arc(scan, -7 * math.pi / 18)   # FIX 4: was 'rigth'
        frontOffset = front - 0.3
        if self.getDirection() == 'CCW_Corner':
            sideOffset = left - 0.3
        else:
            sideOffset = right - 0.3
        return frontOffset, sideOffset

    def send(self, x, y):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.w = 1.0
        self.pub.publish(p)


def main():
    rclpy.init()
    rclpy.spin(OpenNav())
    # FIX 3: removed the unreachable/broken lines that were here


if __name__ == '__main__':
    main()
