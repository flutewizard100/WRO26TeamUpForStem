"""Real-hardware WRO mission launcher.

Composes:
  - WRORobot/hardware.launch.py  — drivers + SLAM + Nav2 navigation
  - waypoint_nav_bridge/bridge   — /waypoint (PoseStamped)  ->  Nav2 action
  - wro_behavior/wro_mission     — unified open/obstacle mission logic

Run:
  ros2 launch wro_behavior wro_mission_hw.launch.py
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

    mission = Node(
        package='wro_behavior',
        executable='wro_mission',
        name='wro_mission',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        hardware,
        waypoint_bridge,
        mission,
    ])
