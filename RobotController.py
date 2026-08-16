import rclpy
import serial
import time
from rclpy.node import Node
from std_msgs.msg import Int32 
from geometry_msgs.msg import Twist
from WRORobot.Motor import Motor
from WRORobot.Servo import Servo

class controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.send = self.create_publisher(Twist, 'cmd_vel', 10)

    def sendMovement(self, msg):
        self.send.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
