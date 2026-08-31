#!/usr/bin/env python3
"""Minimal NavigateToPose action-client node.

Sends a single goal to Nav2's /navigate_to_pose action and prints feedback +
result. This is the 'hello world' for the behavior layer — it proves the
sim engineer's stack talks to Nav2 correctly. Real mission logic (state
machines, per-round sequences, vision reactions) will replace this node.

Run:
  ros2 run wro_behavior goto_pose --ros-args \
    -p x:=1.0 -p y:=0.5 -p yaw:=0.0
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GotoPose(Node):
    def __init__(self):
        super().__init__('wro_goto_pose')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send(self):
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value
        frame_id = self.get_parameter('frame_id').value

        self.get_logger().info(
            f'Waiting for /navigate_to_pose action server...')
        self._client.wait_for_server()

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f'Sending goal: frame={frame_id} x={x} y={y} yaw={yaw}')
        future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'distance remaining: {fb.distance_remaining:.2f} m, '
            f'ETA: {fb.estimated_time_remaining.sec} s')

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted; waiting for result...')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 reached the goal.')
        else:
            self.get_logger().warn(f'Nav2 terminated with status {status}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = GotoPose()
    node.send()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
