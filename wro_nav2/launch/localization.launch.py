"""AMCL + map_server + lifecycle_manager only (no planner/controller).

Use this if you already have Nav2 planning bringup somewhere else, or if you
want to test localization separately.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_wro_nav2 = get_package_share_directory('wro_nav2')

    default_params = os.path.join(pkg_wro_nav2, 'params', 'nav2_params.yaml')
    default_map = os.path.join(pkg_wro_nav2, 'maps', 'wro_field.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map)

    lifecycle_nodes = ['map_server', 'amcl']

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time,
             'yaml_filename': map_yaml},
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params,
        declare_map,
        map_server,
        amcl,
        lifecycle_manager,
    ])
