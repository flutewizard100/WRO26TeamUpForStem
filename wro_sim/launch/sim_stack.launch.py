"""Top-level sim stack: Gazebo + Nav2 + behavior layer.

This is the sim engineer's default entry point. It composes three layers:
  1. wro_sim/sim.launch.py       — Gazebo + URDF + bridge
  2. wro_nav2/nav2.launch.py     — Nav2 stack (or slam.launch.py if slam:=true)
  3. wro_behavior/behavior.launch.py — mission nodes (off by default)

The real-robot counterpart is WRORobot/launch/robot_stack.launch.py — same
navigation + behavior layers, hardware.launch.py in place of sim.launch.py.

Args:
  slam:=true      -> use slam_toolbox instead of AMCL/map_server
  rviz:=true      -> also open RViz
  run_goto:=true  -> fire the wro_behavior goto_pose hello-world on launch
  x, y, yaw       -> spawn pose (defaults: 0, -1.3, 1.5708)

Note: wro_nav2/nav2.launch.py wraps nav2_bringup/bringup_launch.py, which
already starts AMCL + map_server. wro_nav2/localization.launch.py is a
subset of that for standalone use — do not include it here or AMCL and
map_server will double-launch.
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

    sim_params = os.path.join(pkg_wro_nav2, 'params', 'nav2_params_sim.yaml')
    sim_map = os.path.join(pkg_wro_nav2, 'maps', 'wro_field.yaml')

    slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_goto = LaunchConfiguration('run_goto')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    yaw_pose = LaunchConfiguration('yaw')

    declare_slam = DeclareLaunchArgument('slam', default_value='false')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_run_goto = DeclareLaunchArgument(
        'run_goto', default_value='false')
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='-1.3')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='1.5708')

    # 1. Gazebo + robot_state_publisher + spawn + bridges (+ optional RViz).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_sim'), 'launch', 'sim.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'x': x_pose,
            'y': y_pose,
            'yaw': yaw_pose,
        }.items(),
    )

    # 2a. Full Nav2 bringup with AMCL (default).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_nav2'),
                'launch', 'nav2.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': sim_params,
            'map': sim_map,
            'use_composition': 'False',
        }.items(),
        condition=UnlessCondition(slam),
    )

    # 2b. SLAM Toolbox + Nav2 planning stack (with slam:=true).
    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_nav2'),
                'launch', 'slam.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': sim_params,
        }.items(),
        condition=IfCondition(slam),
    )

    # 3. Behavior layer (mission-level nodes).
    behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_behavior'),
                'launch', 'behavior.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'run_goto': run_goto,
        }.items(),
    )

    return LaunchDescription([
        declare_slam, declare_rviz, declare_use_sim_time, declare_run_goto,
        declare_x, declare_y, declare_yaw,
        sim,
        nav2,
        slam_bringup,
        behavior,
    ])
