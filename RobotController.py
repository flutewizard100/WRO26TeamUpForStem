import serial
import time
from rclpy.node import Node
from std_msgs.msg import Int32 
from geometry_msgs.msg import Twist
class controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.send = self.create_publisher(Twist, 'cmd_vel', 10)

    def sendMovement(self, msg):
        self.send.publish(msg)
