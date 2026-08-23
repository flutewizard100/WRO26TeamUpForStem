#!/usr/bin/env python3

import math
import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# ---------------------------------------------------------
# REPLACE THESE EXAMPLE NUMBERS WITH YOUR RVIZ CLICKED POINTS
# Put the points in driving order around the obstacle.
# ---------------------------------------------------------
WAYPOINTS = [
    (0.0, 0.0),
    (1.0, 1.0),
    (0.0, 2.0),
    (-1.0, 1.0),
]

# Number of complete circuits.
# Set this to None for continuous patrol until Ctrl+C.
NUMBER_OF_LOOPS = 6


def create_pose(navigator, x, y, yaw):
    pose = PoseStamped()

    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # Convert yaw angle into a quaternion.
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    return pose


def build_route(navigator):
    poses = []

    for index, (x, y) in enumerate(WAYPOINTS):
        # Make the robot face toward the following waypoint.
        next_index = (index + 1) % len(WAYPOINTS)
        next_x, next_y = WAYPOINTS[next_index]

        yaw = math.atan2(next_y - y, next_x - x)
        poses.append(create_pose(navigator, x, y, yaw))

    return poses


def main():
    rclpy.init()
    navigator = BasicNavigator()

    route = build_route(navigator)

    print("Waiting for the Nav2 waypoint action server...")
    print(f"Loaded {len(route)} waypoints.")

    completed_loops = 0

    try:
        while rclpy.ok():
            if NUMBER_OF_LOOPS is not None:
                if completed_loops >= NUMBER_OF_LOOPS:
                    break

            print(f"Starting circuit {completed_loops + 1}...")

            accepted = navigator.followWaypoints(route)

            if not accepted:
                print("Nav2 rejected the waypoint mission.")
                break

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()

                if feedback is not None:
                    current = feedback.current_waypoint + 1
                    print(
                        f"\rGoing to waypoint {current}/{len(route)}",
                        end="",
                        flush=True,
                    )

            print()
            result = navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                completed_loops += 1
                print(f"Completed circuit {completed_loops}.")

            elif result == TaskResult.CANCELED:
                print("Patrol was canceled.")
                break

            elif result == TaskResult.FAILED:
                print("Patrol failed. Check RViz and Nav2 logs.")
                break

            else:
                print("Nav2 returned an unknown result.")
                break

    except KeyboardInterrupt:
        print("\nCtrl+C received. Canceling patrol...")
        navigator.cancelTask()

    finally:
        navigator.destroyNode()
        rclpy.shutdown()

    print("Patrol finished.")


if __name__ == "__main__":
    main()
