#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

# Import official SparkFun OTOS library
import qwiic_otos

class OtosOdometryNode(Node):
    def __init__(self):
        super().__init__('otos_odometry_node')

        # publish_tf: broadcast odom -> base_link directly. Set False when
        # ekf_filter_node is running (fusion pipeline) — the EKF becomes the
        # sole publisher of that TF edge, using OTOS as one of several inputs.
        # Set True when running OTOS standalone (no fusion). Default is False
        # to match hardware.launch.py's fusion setup.
        self.declare_parameter('publish_tf', False)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)

        # Initialize publishers and TF broadcasters
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self._publish_tf else None
        
        # Initialize the hardware sensor via I2C (Fixed Case Sensitivity)
        self.get_logger().info("Initializing SparkFun OTOS PAA5160E1...")
        self.sensor = qwiic_otos.QwiicOTOS() 
        
        if not self.sensor.is_connected():
            self.get_logger().error("PAA5160E1 Sensor not detected on I2C bus! Check Qwiic connections.")
            return

        self.sensor.begin()
        
        # Configure Sensor Options (Fixed Case Sensitivity constants)
        self.sensor.setLinearUnit(qwiic_otos.METER)       
        self.sensor.setAngularUnit(qwiic_otos.RADIAN)    
        
        # Calibrate IMU (Fixed camelCase syntax)
        self.get_logger().info("Calibrating IMU... Keep the robot still.")
        self.sensor.calibrateImu()
        self.sensor.resetTracking()
        
        # Create timer loop (50 Hz / every 0.02 seconds)
        self.timer = self.create_timer(0.02, self.update_callback)
        self.get_logger().info("OTOS Python Node successfully started.")

    def update_callback(self):
        # Fetch data using camelCase API functions
        pos = self.sensor.getPosition()
        vel = self.sensor.getVelocity()
        
        current_time = self.get_clock().now().to_msg()
        
        # 1. Publish standard ROS2 Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Position Data
        odom.pose.pose.position.x = pos.x
        odom.pose.pose.position.y = pos.y
        odom.pose.pose.position.z = 0.0
        
        # Convert Heading (Yaw) to Quaternion
        # SparkFun's Pose2D class exposes the heading angle as '.h' instead of '.heading'
        q = self.euler_to_quaternion(0, 0, pos.h)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Velocity Data
        odom.twist.twist.linear.x = vel.x
        odom.twist.twist.linear.y = vel.y
        odom.twist.twist.angular.z = vel.h
        
        self.odom_pub.publish(odom)

        # 2. Broadcast /tf Transform (odom -> base_link) — only when acting
        # as sole odom publisher. Under EKF fusion the filter owns this edge.
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

    def euler_to_quaternion(self, roll, pitch, yaw):
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
