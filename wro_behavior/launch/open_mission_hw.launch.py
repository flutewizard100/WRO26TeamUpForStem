"""Real-hardware open mission launcher.

Composes:
  - WRORobot/hardware.launch.py — lidar, IMU, motor, servo, otos, EKF, RSP,
    laser filter, and SLAM Toolbox + Nav2 navigation stack (via
    wro_nav2/slam.launch.py which hardware.launch.py already includes)
  - wro_behavior/waypoints — reusable waypoint runner (Nav2 goal publisher +
    arrival watcher)
  - wro_behavior/open_mission — the mission logic node

Run:
  ros2 launch wro_behavior open_mission_hw.launch.py

Kill:
  Ctrl+C, then
  pkill -9 -f 'ros_gz|nav2|amcl|robot_state|lifecycle|_server|scan_filter|waypoints|open_mission|ldlidar|otos|Motor|Servo|HighController|limelight'
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

    waypoints = Node(
        package='wro_behavior',
        executable='waypoints',
        name='waypoints',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    open_mission = Node(
        package='wro_behavior',
        executable='open_mission',
        name='open_mission',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        hardware,
        waypoints,
        open_mission,
    ])
