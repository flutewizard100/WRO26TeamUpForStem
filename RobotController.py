
from Servo import Servo
import time
servo = Servo()

try:
    while True:
        servo.write_angle(179)
        time.sleep(0.5)
        servo.write_angle(0)
        time.sleep(0.5)
except KeyboardInterrupt:
    servo.close()
    print("Stopped cleanly")