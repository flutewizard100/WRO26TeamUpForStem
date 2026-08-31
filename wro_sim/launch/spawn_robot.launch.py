"""Standalone spawn of wro_bot into an already-running Gazebo instance.

Requires robot_description to be published on /robot_description (e.g. by
running robot_state_publisher separately, or by launching gz_sim.launch.py
which starts it for you).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw_pose = LaunchConfiguration('yaw')
    name = LaunchConfiguration('name')

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-1.3'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        DeclareLaunchArgument('name', default_value='wro_bot'),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_wro_bot',
            arguments=[
                '-name', name,
                '-topic', 'robot_description',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-Y', yaw_pose,
            ],
            output='screen',
        ),
    ])
