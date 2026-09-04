"""Simple open-loop hardware test.

Publishes /cmd_vel on a fixed schedule — no sensors, no feedback. Use this to
verify the ESC + servo + Teensy chain works before running anything smart.

Sequence:
  0.0 - 2.0 s   stopped   (safety pause after launch)
  2.0 - 4.0 s   forward   (linear.x = TEST_SPEED, angular.z = 0)
  4.0 - 5.0 s   stopped
  5.0 - 7.0 s   turning   (linear.x = TEST_SPEED, angular.z = TEST_TURN)
  7.0 +         stopped forever

Run:
  ros2 run wro_behavior hardware_test

Kill:
  Ctrl+C in the terminal, and the last message published will be zero
  (see stop-on-exit below) so the motor doesn't keep running.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


TEST_SPEED = 0.2      # m/s  — start small; bump up once you trust the wiring
TEST_TURN = 0.8       # rad/s  — positive turns left


class HardwareTest(Node):
    def __init__(self):
        super().__init__('hardware_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start_time = self.get_clock().now()
        self.create_timer(0.05, self.step)   # 20 Hz
        self.get_logger().info(
            'hardware_test up — sequence starts in 2 s: forward, stop, turn, stop')

    def elapsed_s(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def step(self) -> None:
        t = self.elapsed_s()
        cmd = Twist()

        if t < 2.0:
            phase = 'wait'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif t < 4.0:
            phase = 'forward'
            cmd.linear.x = TEST_SPEED
            cmd.angular.z = 0.0
        elif t < 5.0:
            phase = 'stop'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif t < 7.0:
            phase = 'turn'
            cmd.linear.x = TEST_SPEED
            cmd.angular.z = TEST_TURN
        else:
            phase = 'done'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        # Prints twice a second so the terminal isn't flooded.
        if int(t * 2) != int((t - 0.05) * 2):
            print(f"[t={t:5.2f}] phase={phase:7s} "
                  f"linear.x={cmd.linear.x:+.2f} angular.z={cmd.angular.z:+.2f}",
                  flush=True)

    def stop(self) -> None:
        """Publish a zero Twist so the robot brakes when we exit."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = HardwareTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
