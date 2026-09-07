#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import limelight


class LimelightNode(Node):
    def __init__(self):
        super().__init__('limelight_node')

        self.get_logger().info('Scanning the network for Limelights...')

        # Discover Limelights on the local network
        try:
            self.discovered_ips = limelight.discover_limelights(debug=True)
        except Exception as e:
            self.get_logger().error(f'Limelight discovery failed: {e}')
            self.discovered_ips = []

        if not self.discovered_ips:
            self.get_logger().error(
                'No Limelights found. Make sure the robot/camera is powered on '
                'and connected to the same Wi-Fi/Ethernet network!'
            )

            # Try discovery again every 5 seconds
            self.timer = self.create_timer(5.0, self.discover_limelight)
            return

        self.target_ip = self.discovered_ips[0]

        self.get_logger().info(
            f'Found Limelight(s) at: {self.discovered_ips}'
        )
        self.get_logger().info(
            f'Using Limelight at {self.target_ip}'
        )

        # Poll the Limelight every 100 ms (10 Hz)
        self.timer = self.create_timer(0.1, self.poll_limelight)

    def discover_limelight(self):
        """Try to discover a Limelight if one was not initially found."""
        self.get_logger().info('Searching for Limelights...')

        try:
            discovered_ips = limelight.discover_limelights(debug=True)

            if discovered_ips:
                self.discovered_ips = discovered_ips
                self.target_ip = discovered_ips[0]

                self.get_logger().info(
                    f'Found Limelight(s): {discovered_ips}'
                )

                # Stop the discovery timer and start polling
                self.timer.cancel()
                self.timer = self.create_timer(0.1, self.poll_limelight)

        except Exception as e:
            self.get_logger().warn(
                f'Limelight discovery failed: {e}'
            )

    def poll_limelight(self):
        """Get the latest data from the Limelight."""
        try:
            results = limelight.get_results(self.target_ip)

            if not results:
                self.get_logger().warn(
                    f'No data received from Limelight at {self.target_ip}'
                )
                return

            valid = results.get('Valid', False)

            if valid:
                self.get_logger().info('Target Found')
            else:
                self.get_logger().debug('No Target Visible')

        except Exception as e:
            self.get_logger().error(
                f'Error communicating with Limelight: {e}'
            )


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
