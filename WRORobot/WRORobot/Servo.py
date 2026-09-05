import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


class Servo(Node):
    MIN_ANGLE = 40
    CENTER_ANGLE = 80
    MAX_ANGLE = 150

    def __init__(self):
        super().__init__('servo')

        self.servo_publisher = self.create_publisher(
            Int32,
            'servo_angle',
            10
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def scale_steering(self, steering_value):

        steering_value = max(-1.0, min(1.0, steering_value))

        if steering_value >= 0.0:
            angle = self.CENTER_ANGLE + steering_value * (self.MIN_ANGLE - self.CENTER_ANGLE)
        else:
            angle = self.CENTER_ANGLE + steering_value * (self.CENTER_ANGLE - self.MAX_ANGLE)

        return int(round(angle))

    def cmd_vel_callback(self, msg):
        angle = self.scale_steering(msg.angular.z)
        self.write(angle)

    def write(self, angle):
        servo_msg = Int32()
        servo_msg.data = int(
            max(self.MIN_ANGLE, min(self.MAX_ANGLE, angle))
        )

        self.servo_publisher.publish(servo_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Servo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
