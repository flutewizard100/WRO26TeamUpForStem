from Servo import Servo
from Motor import Motor
import time
import rclpy
import signal
import sys
from rclpy.executors import MultiThreadedExecutor



rclpy.init()
motor = Motor()
servo = Servo()

executor = MultiThreadedExecutor()
executor.add_node(motor)
executor.add_node(servo)
# def custom_signal_handler(sig, frame):


#    motor.setSpeed(1500)
#    servo.write(80)


#    raise KeyboardInterrupt


# signal.signal(signal.SIGINT, custom_signal_handler)
# signal.signal(signal.SIGTERM, custom_signal_handler)


try:
   while rclpy.ok():

      executor.spin()
      continue

except KeyboardInterrupt:
   print("Program ended")
   pass


finally:
   # Fallback cleanup block for non-interrupt closures (like normal loops finishing)
   if rclpy.ok():
       motor.setSpeed(1500)
       motor.destroy_node()
       servo.write(80)
       servo.destroy_node()
       rclpy.shutdown()



