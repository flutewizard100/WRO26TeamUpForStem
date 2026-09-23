"""Shared hardware stack — everything except the mission node.

Brings up:
  - WRORobot/hardware.launch.py     — drivers + SLAM + Nav2 navigation
  - waypoint_nav_bridge/bridge      — /waypoint (PoseStamped) -> Nav2 action
  - wro_behavior/camera_map_augmenter — camera detections -> /camera_obstacles

The mission node is added by the challenge-specific launch on top of
this (open_challenge_hw.launch.py / obstacle_challenge_hw.launch.py).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('WRORobot'),
                'launch',
                'hardware.launch.py',
            ])
        ),
    )

    waypoint_bridge = Node(
        package='waypoint_nav_bridge',
        executable='bridge',
        name='waypoint_nav_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    camera_map_augmenter = Node(
        package='wro_behavior',
        executable='camera_map_augmenter',
        name='camera_map_augmenter',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        hardware,
        waypoint_bridge,
        camera_map_augmenter,
    ])
