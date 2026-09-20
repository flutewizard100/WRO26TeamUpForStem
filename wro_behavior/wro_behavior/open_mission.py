import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState


WAYPOINTS = [
    (0.9, 0)
]


def median_in_arc( scan: LaserScan,
    center_rad, half_width_rad=math.radians(8), 
):
    vals = []

    for index, distance in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment

        # Angular difference wrapped to [-pi, pi].
        difference = math.atan2(
            math.sin(angle - center_rad),
            math.cos(angle - center_rad),
        )

        if (
            abs(difference) <= half_width_rad
            and math.isfinite(distance)
            and scan.range_min <= distance <= scan.range_max
        ):
            vals.append(distance)

    if not vals:
        return scan.range_max

    vals.sort()
    return vals[len(vals) // 2]


class OpenNav(Node):
    def __init__(self):
        super().__init__('open_mission')

        self.detections = []
        self.i = 0
        self.waiting_for_arrival = False

        self.pub = self.create_publisher(
            PoseStamped,
            '/waypoint',
            10,
        )

        self.arrived_sub = self.create_subscription(
            Bool,
            '/waypoint_arrived',
            self.on_arrived,
            10,
        )

        self.nav_state_client = self.create_client(
            GetState,
            '/bt_navigator/get_state',
        )
        self.nav_state_future = None

        self.start_timer = self.create_timer(
            0.2,
            self.start_when_connected,
        )

        self.get_logger().info(
            'Waiting for the waypoint bridge and active bt_navigator...'
        )

    def start_when_connected(self):
        # First, wait for the bridge to subscribe.
        if self.pub.get_subscription_count() == 0:
            return

        # Then wait for the lifecycle service to appear.
        if not self.nav_state_client.service_is_ready():
            return

        # Start one asynchronous state request at a time.
        if self.nav_state_future is None:
            self.nav_state_future = self.nav_state_client.call_async(
                GetState.Request()
            )
            return

        if not self.nav_state_future.done():
            return

        future = self.nav_state_future
        self.nav_state_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Could not query bt_navigator state: {exc}'
            )
            return

        if response is None:
            return

        # Being discoverable is not enough: require ACTIVE.
        if response.current_state.id != State.PRIMARY_STATE_ACTIVE:
            return

        # Only start the mission once.
        self.start_timer.cancel()

        if not WAYPOINTS:
            self.get_logger().info('No waypoints configured.')
            return

        self.get_logger().info(
            'bt_navigator is active and bridge connected. '
            'Starting mission.'
        )
        self.send(*WAYPOINTS[self.i])

    def on_arrived(self, msg: Bool):
        if not self.waiting_for_arrival:
            return

        self.waiting_for_arrival = False

        if not msg.data:
            self.get_logger().error(
                f'Waypoint {self.i + 1} failed. '
                'Mission paused; no automatic retry.'
            )
            return

        self.get_logger().info(
            f'Waypoint {self.i + 1} reached.'
        )

        self.i += 1

        if self.i < len(WAYPOINTS):
            self.send(*WAYPOINTS[self.i])
        else:
            self.get_logger().info(
                'All waypoints completed.'
            )

    def send(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.waiting_for_arrival = True
        self.pub.publish(pose)

        self.get_logger().info(
            f'Sent waypoint {self.i + 1}/{len(WAYPOINTS)}: '
            f'x={x:.2f}, y={y:.2f}'
        )

    def getDirection(self):
        for detection in self.detections:
            if not detection.results:
                continue

            class_id = detection.results[0].hypothesis.class_id

            if class_id in ('CW_Corner', 'CCW_Corner'):
                return class_id

        return None

    def findStartPose(self, scan: LaserScan):
        front = median_in_arc(scan, 0.0)
        left = median_in_arc(scan, 7 * math.pi / 18)
        right = median_in_arc(scan, -7 * math.pi / 18)

        front_offset = front - 0.3

        if self.getDirection() == 'CCW_Corner':
            side_offset = left - 0.3
        else:
            side_offset = right - 0.3

        return front_offset, side_offset


def main(args=None):
    rclpy.init(args=args)
    node = OpenNav()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
