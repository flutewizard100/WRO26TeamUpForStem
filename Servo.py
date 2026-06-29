
import serial
import time
from rclpy.node import Node
from std_msgs.msg import Int32 

class Servo(Node):
    def __init__(self):

        super().__init__('Servo')
        self.servo = self.create_publisher(Int32, 'Servo', 10)
        self.subscription = self.create_subscription(
            Int32,
            'servo_angle',
            self.servo_callback,
            10
        )
 

    def servo_callback(self, msg):
        self.write(msg.data)

    def write(self, angle):
        servo_msg = Int32()
        servo_msg.data = int(max(0, min(360, angle)))
        self.servo.publish(servo_msg)



