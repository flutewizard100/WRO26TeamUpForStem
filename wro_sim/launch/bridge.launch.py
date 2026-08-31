"""Standalone ros_gz_bridge + image_bridge.

Use with an already-running Gazebo + robot instance.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_wro_sim = get_package_share_directory('wro_sim')
    default_bridge = os.path.join(pkg_wro_sim, 'config', 'ros_gz_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=default_bridge),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{
                'config_file': config_file,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='camera_image_bridge',
            arguments=['/camera'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
