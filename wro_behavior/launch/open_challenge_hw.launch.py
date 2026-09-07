"""Real-hardware open challenge launcher.

Composes:
  - WRORobot/hardware.launch.py — lidar, IMU, motor, servo, otos, EKF, RSP,
    laser filter chain (raw /scan_raw -> cleaned /scan)
  - wro_behavior/open_challenge_template_lidar_only — the wall-follower node

Behavior node subscribes to /scan (the filtered output) and publishes /cmd_vel,
which the hardware layer's Motor + Servo nodes consume.

Run:
  ros2 launch wro_behavior open_challenge_hw.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('WRORobot'),
                'launch',
                'hardware.launch.py',
            ])
        ]),
    )

    open_challenge = Node(
        package='wro_behavior',
        executable='open_challenge_template_lidar_only',
        name='open_challenge_template',
        output='screen',
    )

    return LaunchDescription([
        hardware,
        open_challenge,
    ])
