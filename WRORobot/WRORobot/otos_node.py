import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

import tf2_ros
import math

import qwiic_otos
from qwiic_i2c.linux_i2c import LinuxI2C


class OtosI2C(LinuxI2C):
    """
    OTOS-specific I2C driver.

    qwiic_i2c's normal Linux driver uses SMBus Quick Write for
    isDeviceConnected(), which does not work correctly with this
    hardware/interface. We instead verify the OTOS product ID directly.
    """

    def isDeviceConnected(self, devAddress):
        try:
            product_id = self.readByte(devAddress, 0x00)
            return product_id == 0x5F
        except Exception:
            return False

    def is_device_connected(self, devAddress):
        return self.isDeviceConnected(devAddress)

    def ping(self, devAddress):
        return self.isDeviceConnected(devAddress)


class OtosOdometryNode(Node):

    def __init__(self):
        super().__init__('otos_odometry_node')

        self.declare_parameter('publish_tf', False)
        self._publish_tf = bool(
            self.get_parameter('publish_tf').value
        )

        # --------------------------------------------------------------
        # ROS publishers
        # --------------------------------------------------------------

        self.odom_pub = self.create_publisher(
            Odometry,
            'otos_raw',
            10
        )

        self.imu_pub = self.create_publisher(
            Imu,
            'imu/data',
            10
        )

        self.tf_broadcaster = (
            tf2_ros.TransformBroadcaster(self)
            if self._publish_tf
            else None
        )

        # --------------------------------------------------------------
        # Initialize OTOS
        # --------------------------------------------------------------

        self.get_logger().info(
            "Initializing SparkFun OTOS PAA5160E1..."
        )

        i2c = OtosI2C(iBus=7)

        self.sensor = qwiic_otos.QwiicOTOS(
            address=0x17,
            i2c_driver=i2c
        )

        if not self.sensor.is_connected():
            self.get_logger().error(
                "PAA5160E1 Sensor not detected on I2C bus! "
                "Check Qwiic connections."
            )
            raise RuntimeError("OTOS not connected")

        if not self.sensor.begin():
            self.get_logger().error(
                "OTOS begin() failed."
            )
            raise RuntimeError("OTOS initialization failed")

        # Use SI units.
        self.sensor.setLinearUnit(
            qwiic_otos.QwiicOTOS.kLinearUnitMeters
        )

        self.sensor.setAngularScalar(0.99995448187)
        self.sensor.setLinearScalar(0.99554497057)

        self.sensor.setAngularUnit(
            qwiic_otos.QwiicOTOS.kAngularUnitRadians
        )

        # --------------------------------------------------------------
        # Calibrate IMU
        # --------------------------------------------------------------

        self.get_logger().info(
            "Calibrating IMU... Keep the robot still."
        )

        if not self.sensor.calibrateImu():
            self.get_logger().error(
                "OTOS IMU calibration failed."
            )

        self.sensor.resetTracking()

        # --------------------------------------------------------------
        # Main loop: 50 Hz
        # --------------------------------------------------------------

        self.timer = self.create_timer(
            0.02,
            self.update_callback
        )

        self.get_logger().info(
            "OTOS Python Node successfully started."
        )

    def update_callback(self):

        try:
            # Read position, velocity and acceleration in one I2C burst.
            #
            # These are returned in SI units:
            #   position:     m, m, rad
            #   velocity:     m/s, m/s, rad/s
            #   acceleration: m/s^2, m/s^2, rad/s^2
            pos, vel, acc = self.sensor.getPosVelAcc()

        except Exception as e:
            self.get_logger().error(
                f"Failed to read OTOS: {e}"
            )
            return

        current_time = self.get_clock().now().to_msg()

        # --------------------------------------------------------------
        # Convert OTOS heading to ROS quaternion
        # --------------------------------------------------------------

        q = self.euler_to_quaternion(
            0.0,
            0.0,
            pos.h
        )

        # --------------------------------------------------------------
        # 1. Publish Odometry
        # --------------------------------------------------------------

        odom = Odometry()

        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = pos.x
        odom.pose.pose.position.y = pos.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Velocity
        odom.twist.twist.linear.x = vel.x
        odom.twist.twist.linear.y = vel.y
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = vel.h

        self.odom_pub.publish(odom)

        # --------------------------------------------------------------
        # 2. Publish IMU
        # --------------------------------------------------------------

        imu = Imu()

        imu.header.stamp = current_time
        imu.header.frame_id = "base_link"

        # OTOS acceleration.
        #
        # In the OTOS Pose2D representation:
        #   acc.x = acceleration along X
        #   acc.y = acceleration along Y
        #
        # OTOS does not expose a separate raw gyro API in this
        # qwiic_otos version. Its velocity heading (vel.h) is the
        # angular velocity estimate.
        imu.linear_acceleration.x = acc.x
        imu.linear_acceleration.y = acc.y
        imu.linear_acceleration.z = 0.0

        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = vel.h

        # We are not publishing an independent IMU orientation
        # measurement here. Let the EKF obtain orientation from
        # the OTOS odometry.
        imu.orientation_covariance[0] = -1.0

        self.imu_pub.publish(imu)

        # --------------------------------------------------------------
        # 3. Optional direct TF
        # --------------------------------------------------------------

        if self.tf_broadcaster is not None:

            t = TransformStamped()

            t.header.stamp = current_time
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"

            t.transform.translation.x = pos.x
            t.transform.translation.y = pos.y
            t.transform.translation.z = 0.0

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0] * 4

        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy

        return q


def main(args=None):

    rclpy.init(args=args)

    node = OtosOdometryNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
