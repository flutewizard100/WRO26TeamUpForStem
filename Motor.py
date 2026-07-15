import serial
import time
from rclpy.node import Node
from std_msgs.msg import Int32 
import rclpy

class Motor(Node):

    def __init__(self):
        super().__init__("Motor")
        self.motor = self.create_publisher(Int32, 'motor_speed', 10)


    def setSpeed(self, speed):


        final_us = max(1000, min(2000, int(speed)))

        motor_msg = Int32()
        motor_msg.data = final_us
        self.motor.publish(motor_msg)


    def setTime(self, target_speed, duration_seconds):
       
                
        self.setSpeed(target_speed)
            
        rclpy.spin_once(self, timeout_sec=0.01)
        time.sleep(duration_seconds)
        self.setSpeed(1500)
        rclpy.spin_once(self, timeout_sec=0.01)
