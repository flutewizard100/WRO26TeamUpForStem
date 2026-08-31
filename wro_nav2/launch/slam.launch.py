"""SLAM Toolbox (online async) + Nav2 planning stack (no AMCL, no map_server).

Use this to build a map online. slam_params.yaml lives in the existing
WRORobot package (WRORobot/config/slam_params.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_wro_nav2 = get_package_share_directory('wro_nav2')
    pkg_wro_robot = get_package_share_directory('WRORobot')

    default_params = os.path.join(pkg_wro_nav2, 'params', 'nav2_params.yaml')
    default_slam_params = os.path.join(
        pkg_wro_robot, 'config', 'slam_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params)

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Bring up Nav2's navigation-only side (planner/controller/behavior/bt).
    # Nav2's navigation_launch.py starts everything except map_server + amcl.
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params,
        declare_slam_params,
        slam_toolbox,
        nav2_navigation,
    ])
