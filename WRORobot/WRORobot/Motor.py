import serial
import time
from rclpy.node import Node
from std_msgs.msg import Int32 
from geometry_msgs.msg import Twist
import rclpy

class Motor(Node):

    def __init__(self):
        super().__init__("Motor")

        self.motor = self.create_publisher(Int32, 'motor_speed', 10)
        self.cmd = self.create_subscription(Twist, 'cmd_vel', self.sendCommand, 10)


    def sendCommand(self, msg):

        motor_msg = Int32()

        direction = msg.linear.x

        MAX_SPEED = 1.0

        direction = cmd_vel.linear.x / MAX_SPEED
        direction = max(-1.0, min(1.0, direction))

        if direction > 0.0:
            motor_msg.data = int(1585 + direction * 415)
        elif direction < 0.0:
            motor_msg.data = int(1450 + direction * 450)
        else:
            motor_msg.data = 1500

        # Clamp to valid ESC range
        motor_msg.data = max(1000, min(2000, motor_msg.data))

        self.motor.publish(motor_msg)


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

def main(args=None):
    rclpy.init(args=args)

    node = Motor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
      main()
