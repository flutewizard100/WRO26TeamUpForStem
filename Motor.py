import serial
import time
from rclpy.node import Node
from std_msgs.msg import Int32 

class Motor(Node):

    def __init__(self):

        super().__init__("Motor")
        self.motor = self.create_publisher(Int32, 'Motor', 10)
        self.subscription = self.create_subscription(Int32, 'motor_speed', self.motor_callback, 10)
        self.direction = True

    def motor_callback(self, msg):
        self.setSpeed(msg.data)

    def setSpeed(self, speed):
        motor_msg = Int32()

        if not self.direction:
           speed = -speed

        motor_msg.data = int(max(-255, min(255, speed)))
        self.motor.publish(motor_msg)

    def setDirection(self, direction):
        if direction == "REVERSE":
            self.direction = False
        else:
            self.direction = True