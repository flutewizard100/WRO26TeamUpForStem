#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32

import limelight


class LimelightNode(Node):

    def __init__(self):
        super().__init__('limelight_node')

        # ---------------------------------------------------------
        # ROS 2 parameters
        # ---------------------------------------------------------
        self.declare_parameter('limelight_ip', '')
        self.declare_parameter('poll_rate', 20.0)

        self.limelight_ip = (
            self.get_parameter('limelight_ip')
            .get_parameter_value()
            .string_value
        )

        poll_rate = (
            self.get_parameter('poll_rate')
            .get_parameter_value()
            .double_value
        )

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------
        self.target_found_pub = self.create_publisher(
            Bool,
            '/limelight/target_found',
            10
        )

        self.tx_pub = self.create_publisher(
            Float32,
            '/limelight/tx',
            10
        )

        self.ty_pub = self.create_publisher(
            Float32,
            '/limelight/ty',
            10
        )

        # ---------------------------------------------------------
        # Limelight setup
        # ---------------------------------------------------------
        self.ll = None

        if self.limelight_ip:
            self.connect_to_limelight(self.limelight_ip)
        else:
            self.discover_limelight()

        # ---------------------------------------------------------
        # Timer
        # ---------------------------------------------------------
        self.timer = self.create_timer(
            1.0 / poll_rate,
            self.publish_results
        )

    # =============================================================
    # Limelight discovery
    # =============================================================

    def discover_limelight(self):

        self.get_logger().info(
            'Scanning the network for Limelights...'
        )

        try:
            discovered_ips = limelight.discover_limelights(
                debug=True
            )

            if not discovered_ips:
                self.get_logger().warn(
                    'No Limelights found.'
                )
                return

            self.get_logger().info(
                f'Found Limelight(s): {discovered_ips}'
            )

            # Use the first discovered Limelight
            self.connect_to_limelight(discovered_ips[0])

        except Exception as e:

            self.get_logger().error(
                f'Limelight discovery failed: {e}'
            )

    # =============================================================
    # Connect
    # =============================================================

    def connect_to_limelight(self, ip):

        try:

            self.ll = limelight.Limelight(ip)

            self.get_logger().info(
                f'Connected to Limelight at {ip}'
            )

            # Print camera name if available
            name = self.ll.get_name()

            if name:
                self.get_logger().info(
                    f'Limelight name: {name}'
                )

            # Start WebSocket streaming
            self.ll.enable_websocket()

            self.get_logger().info(
                'Limelight WebSocket enabled.'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to connect to Limelight: {e}'
            )

            self.ll = None

    # =============================================================
    # Publish results
    # =============================================================

    def publish_results(self):

        if self.ll is None:

            # Keep trying to discover if disconnected
            self.discover_limelight()
            return

        try:

            # -----------------------------------------------------
            # Get latest WebSocket result
            # -----------------------------------------------------

            results = self.ll.get_latest_results()

            if results is None:
                self.get_logger().debug(
                    'Waiting for Limelight data...'
                )
                return

            # -----------------------------------------------------
            # Target validity
            # -----------------------------------------------------

            valid = results.get('Valid', False)

            target_msg = Bool()
            target_msg.data = bool(valid)

            self.target_found_pub.publish(target_msg)

            # -----------------------------------------------------
            # TX
            # -----------------------------------------------------

            tx = results.get('tx', 0.0)

            tx_msg = Float32()
            tx_msg.data = float(tx)

            self.tx_pub.publish(tx_msg)

            # -----------------------------------------------------
            # TY
            # -----------------------------------------------------

            ty = results.get('ty', 0.0)

            ty_msg = Float32()
            ty_msg.data = float(ty)

            self.ty_pub.publish(ty_msg)

            # -----------------------------------------------------
            # Debug output
            # -----------------------------------------------------

            if valid:

                self.get_logger().debug(
                    f'Target found | tx={tx:.2f} | ty={ty:.2f}'
                )

            else:

                self.get_logger().debug(
                    'No target visible'
                )

        except Exception as e:

            self.get_logger().error(
                f'Error reading Limelight: {e}'
            )

    # =============================================================
    # Shutdown
    # =============================================================

    def destroy_node(self):

        self.get_logger().info(
            'Shutting down Limelight node...'
        )

        if self.ll is not None:

            try:
                self.ll.disable_websocket()
            except Exception as e:
                self.get_logger().warn(
                    f'Error closing Limelight WebSocket: {e}'
                )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = LimelightNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
