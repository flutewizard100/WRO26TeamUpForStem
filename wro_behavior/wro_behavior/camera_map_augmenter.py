"""Standalone node that turns camera detections into obstacles for Nav2.

Runs its own subscription to /limelight/detections (via CameraPerception),
accumulates map-frame pillar landmarks, and republishes them as a
PointCloud2 on /camera_obstacles at a steady rate. Nav2's obstacle_layer
can consume this as an additional sensor source so the planner avoids
pillars just like it avoids lidar-detected walls.

Kept separate from wro_mission on purpose — mission logic doesn't need
to know that camera data is being fed to the map, and this node can be
started/stopped/restarted independently.

Wire-in on the Nav2 side (nav2_params_hw.yaml):

  obstacle_layer:
    observation_sources: scan camera_obstacles
    camera_obstacles:
      topic: /camera_obstacles
      data_type: "PointCloud2"
      sensor_frame: map
      marking: true
      clearing: false
      obstacle_max_range: 3.0

Run:
  ros2 run wro_behavior camera_map_augmenter
"""
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import tf2_ros

from wro_behavior.camera_perception import CameraPerception


# How often to republish the obstacle cloud. Higher = fresher costmap
# updates, more CPU. Nav2's obstacle layer will "clear" stale marks
# on its own so we don't have to worry about republishing every frame.
PUBLISH_HZ = 5.0

# Height above the ground plane to place obstacle points (metres). The
# WRO pillar is 100 mm tall; 5 cm places the point mid-pillar so the
# costmap's obstacle layer marks it whatever slice it samples.
OBSTACLE_Z_M = 0.05

# Only publish landmarks confirmed by at least this many sightings —
# reduces flicker from a single mis-detection.
MIN_SIGHTINGS = 2


class CameraMapAugmenter(Node):
    def __init__(self):
        super().__init__('camera_map_augmenter')

        # `publish_walls`: whether to include camera-detected wall points
        # in /camera_obstacles. Default False so the costmap only gets
        # camera-derived PILLARS by default — useful when isolating
        # costmap issues that might come from noisy wall projections.
        # Toggle with:
        #   ros2 run wro_behavior camera_map_augmenter \
        #     --ros-args -p publish_walls:=true
        self.declare_parameter('publish_walls', False)
        self.publish_walls = bool(
            self.get_parameter('publish_walls').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Owns its own perception instance — independent of the mission
        # node's copy, cleanly decoupled.
        self.camera = CameraPerception(self, self.tf_buffer)

        self.pub = self.create_publisher(
            PointCloud2, '/camera_obstacles', 10)

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_obstacles)

        self.get_logger().info(
            f'camera_map_augmenter up — publishing /camera_obstacles '
            f'at {PUBLISH_HZ:.0f} Hz (publish_walls={self.publish_walls})')

    def _publish_obstacles(self) -> None:
        pillars = self.camera.confirmed_pillar_landmarks(
            min_sightings=MIN_SIGHTINGS)
        walls = (self.camera.confirmed_wall_landmarks()
                 if self.publish_walls else [])

        if not pillars and not walls:
            return

        points: List[Tuple[float, float, float]] = []
        points.extend((x, y, OBSTACLE_Z_M) for _cls, x, y in pillars)
        points.extend((x, y, OBSTACLE_Z_M) for x, y in walls)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = CameraMapAugmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
