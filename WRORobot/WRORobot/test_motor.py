import rclpy
import time
from std_msgs.msg import Int32

rclpy.init()

node = rclpy.create_node("motor_test")
publisher = node.create_publisher(Int32, "/motor_speed", 10)

print("Waiting for Motor.py...")

while publisher.get_subscription_count() < 1:
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.1)

print("Motor.py detected! Running motor...")

msg = Int32()
msg.data = 1585

start = time.time()

while time.time() - start < 1.68:
    publisher.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.1)

msg.data = 1500
publisher.publish(msg)

print("Stopped.")

node.destroy_node()
rclpy.shutdown()
