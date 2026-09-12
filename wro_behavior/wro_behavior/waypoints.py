import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tf2_ros

ARRIVED_RADIUS = 0.15   # metres


class WaypointRunner(Node):
    def __init__(self):
        super().__init__('waypoint_runner')
        self.current = None
        self.arrived = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.arrived_pub = self.create_publisher(Bool, '/waypoint_arrived', 10)
        self.create_subscription(PoseStamped, '/waypoint', self.on_waypoint, 10)
        self.create_timer(0.2, self.check_arrival)

    def on_waypoint(self, msg):
        # stamp it fresh so Nav2 doesn't reject as too old
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = msg.header.frame_id or 'map'
        self.current = (msg.pose.position.x, msg.pose.position.y)
        self.arrived = False
        self.goal_pub.publish(msg)
        self.get_logger().info(f'waypoint -> ({self.current[0]:.2f}, {self.current[1]:.2f})')

    def check_arrival(self):
        if self.current is None or self.arrived:
            return
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time(),
                                                timeout=Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return
        dx = t.transform.translation.x - self.current[0]
        dy = t.transform.translation.y - self.current[1]
        if math.hypot(dx, dy) < ARRIVED_RADIUS:
            self.arrived = True
            self.arrived_pub.publish(Bool(data=True))
            self.get_logger().info('arrived')


def main():
    rclpy.init()
    rclpy.spin(WaypointRunner())


if __name__ == '__main__':
    main()

