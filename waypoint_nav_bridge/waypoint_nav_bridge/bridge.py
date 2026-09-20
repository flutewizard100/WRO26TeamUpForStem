import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool


class WaypointNavBridge(Node):
    def __init__(self):
        super().__init__('waypoint_nav_bridge')

        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )

        self.arrived_pub = self.create_publisher(
            Bool, '/waypoint_arrived', 10
        )

        self.waypoint_sub = self.create_subscription(
            PoseStamped,
            '/waypoint',
            self.waypoint_callback,
            10,
        )

        self.pending_pose = None
        self.busy = False

        # Check readiness without blocking subscription callbacks.
        self.timer = self.create_timer(0.5, self.try_send_goal)

        self.get_logger().info('Bridge ready. Waiting for /waypoint.')

    def waypoint_callback(self, msg):
        if self.busy:
            self.get_logger().warning(
                'Already handling a waypoint; ignoring this message.'
            )
            return

        if not msg.header.frame_id:
            self.finish(False, 'Waypoint has no frame_id.')
            return

        p = msg.pose.position
        q = msg.pose.orientation

        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(value) for value in values):
            self.finish(False, 'Waypoint contains non-finite values.')
            return

        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6:
            self.finish(
                False,
                'Invalid orientation: set orientation.w = 1.0 '
                'for zero yaw.',
            )
            return

        # Normalize the supplied quaternion.
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm

        self.pending_pose = msg
        self.busy = True

        self.get_logger().info(
            f'Waypoint received: frame={msg.header.frame_id}, '
            f'x={p.x:.2f}, y={p.y:.2f}'
        )

        self.try_send_goal()

    def try_send_goal(self):
        if self.pending_pose is None:
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().info(
                'Waiting for /navigate_to_pose...',
                throttle_duration_sec=5.0,
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.pending_pose

        # This adapter treats waypoints as fixed targets in their frame.
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.pending_pose = None

        try:
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(self.goal_response_callback)
        except Exception as exc:
            self.finish(False, f'Could not send navigation goal: {exc}')

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.finish(False, 'Nav2 rejected the waypoint.')
                return

            self.get_logger().info('Nav2 accepted the waypoint.')

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

        except Exception as exc:
            self.finish(False, f'Goal response error: {exc}')

    def result_callback(self, future):
        try:
            response = future.result()

            if response.status == GoalStatus.STATUS_SUCCEEDED:
                self.finish(True, 'Waypoint reached.')
            else:
                self.finish(
                    False,
                    f'Navigation did not succeed. '
                    f'Action status: {response.status}',
                )

        except Exception as exc:
            self.finish(False, f'Navigation result error: {exc}')

    def finish(self, success, message):
        self.pending_pose = None
        self.busy = False

        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

        self.arrived_pub.publish(Bool(data=success))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavBridge()

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
