"""Bring up the full Nav2 stack for the WRO robot.

Thin wrapper around nav2_bringup/bringup_launch.py with pre-filled args:
  - params_file, map, use_sim_time, autostart, use_composition:=False.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_wro_nav2 = get_package_share_directory('wro_nav2')

    default_params = os.path.join(pkg_wro_nav2, 'params', 'nav2_params.yaml')
    default_map = os.path.join(pkg_wro_nav2, 'maps', 'wro_field.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_composition = LaunchConfiguration('use_composition')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Autostart the nav2 stack')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 params yaml')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map yaml')
    declare_use_composition = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use ComposableNodes for nav2 (False for easier debug)')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ]),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_map,
        declare_use_composition,
        nav2_bringup,
    ])
