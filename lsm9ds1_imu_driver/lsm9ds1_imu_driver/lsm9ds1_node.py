#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import time
import board
import busio
import adafruit_lsm9ds1

class LSM9DS1DriverNode(Node):
    def __init__(self):
        super().__init__('lsm9ds1_driver_node')
        
        # Create standard ROS 2 publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        # Initialize your working Jetson hardware I2C bus
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_lsm9ds1.LSM9DS1_I2C(self.i2c)
            self.get_logger().info("LSM9DS1 IMU successfully connected via I2C!")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to LSM9DS1: {e}")
            raise e

        # Publish at 50Hz (Standard for EKF / Odometry fusion)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        try:
            # Convert the raw map streams into standard, indexable Python lists
            accel = list(self.sensor.acceleration)
            gyro = list(self.sensor.gyro)
            mag = list(self.sensor.magnetic)

            now = self.get_clock().now().to_msg()

            # 1. Pack the standard IMU Message
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = 'imu_link'
            
            # Accelerometer (m/s²)
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]
            
            # Gyroscope (Convert deg/s to ROS 2 standard radians/s)
            deg_to_rad = 0.01745329251
            imu_msg.angular_velocity.x = gyro[0] * deg_to_rad
            imu_msg.angular_velocity.y = gyro[1] * deg_to_rad
            imu_msg.angular_velocity.z = gyro[2] * deg_to_rad
            
            # Setting the first covariance index to -1 tells ROS 2 orientation is uncalculated
            imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            # 2. Pack the Magnetic Field Message
            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = 'imu_link'
            
            # Convert Gauss to Tesla (1 Gauss = 1e-4 Tesla)
            mag_msg.magnetic_field.x = mag[0] * 1e-4
            mag_msg.magnetic_field.y = mag[1] * 1e-4
            mag_msg.magnetic_field.z = mag[2] * 1e-4

            # Publish topics out to the ROS 2 network
            self.imu_pub.publish(imu_msg)
            self.mag_pub.publish(mag_msg)

        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LSM9DS1DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
