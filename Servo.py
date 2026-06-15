import serial
import time

class Servo:
    def __init__(self, port='/dev/ttyACM0', baud=115200, timeout=1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # Teensy reset delay

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.last_angle = None

    def write_angle(self, angle: int):
        """
        Sends servo angle only if it changed.
        Prevents spam + jitter.
        """
        angle = max(0, min(180, int(angle)))  # clamp safety

        if angle == self.last_angle:
            return

        msg = f"{angle}\n".encode()
        self.ser.write(msg)
        self.last_angle = angle

        print(f"Sent angle: {angle}")

    def close(self):
        self.ser.close()
        print("Serial closed")