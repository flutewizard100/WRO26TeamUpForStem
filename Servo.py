#!/usr/bin/env python3
import serial
import time
from rclpy.node import Node
from std_msgs.msg import Int32 
from geometry_msgs.msg import Twist

class Servo(Node):
    def __init__(self):

        super().__init__('Servo')
        self.servo = self.create_publisher(Int32, 'servo_angle', 10)
        self.cmd = self.create_subscription(Twist, 'cmd_vel', self.sendCommand, 10)
        

    def write(self, angle):
        servo_msg = Int32()
        servo_msg.data = int(max(0, min(360, angle)))
        self.servo.publish(servo_msg)
    
    def sendCommand(self, msg):
        servo_msg = Int32()
        servo_msg.data = int(max(0, min(360, msg.angular.z)))
        self.servo.publish(servo_msg)


