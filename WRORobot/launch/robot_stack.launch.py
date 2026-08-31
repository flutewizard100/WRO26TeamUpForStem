"""Top-level real-robot stack: hardware + Nav2 + behavior.

This is the hardware engineer's default entry point. Composes:
  1. WRORobot/hardware.launch.py       — drivers + robot_state_publisher
  2. wro_nav2/nav2.launch.py           — Nav2 stack (or slam.launch.py if slam:=true)
  3. wro_behavior/behavior.launch.py   — mission nodes (off by default)

Counterpart to wro_sim/sim_stack.launch.py — same nav + behavior layers,
hardware in place of sim.

Args:
  slam:=true      -> use slam_toolbox instead of AMCL/map_server
  run_goto:=true  -> fire the wro_behavior goto_pose hello-world on launch
  map:=<path>     -> map yaml to localize against (defaults to wro_nav2's placeholder)

See INTERFACE.md at repo root for the topic/frame/action contract.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_wro_nav2 = get_package_share_directory('wro_nav2')

    default_params = os.path.join(pkg_wro_nav2, 'params', 'nav2_params.yaml')
    default_map = os.path.join(pkg_wro_nav2, 'maps', 'wro_field.yaml')

    slam = LaunchConfiguration('slam')
    run_goto = LaunchConfiguration('run_goto')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_slam = DeclareLaunchArgument('slam', default_value='false')
    declare_run_goto = DeclareLaunchArgument(
        'run_goto', default_value='false')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    declare_map = DeclareLaunchArgument('map', default_value=default_map)

    # 1. Real hardware — drivers + robot_state_publisher.
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('WRORobot'),
                'launch', 'hardware.launch.py',
            ])
        ]),
    )

    # 2a. Full Nav2 with AMCL (default).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_nav2'),
                'launch', 'nav2.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'map': map_yaml,
            'use_composition': 'False',
        }.items(),
        condition=UnlessCondition(slam),
    )

    # 2b. SLAM Toolbox + Nav2 planning stack.
    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_nav2'),
                'launch', 'slam.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
        }.items(),
        condition=IfCondition(slam),
    )

    # 3. Behavior layer.
    behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_behavior'),
                'launch', 'behavior.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'run_goto': run_goto,
        }.items(),
    )

    return LaunchDescription([
        declare_slam, declare_run_goto, declare_params, declare_map,
        hardware,
        nav2,
        slam_bringup,
        behavior,
    ])
