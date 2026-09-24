"""Shared hardware stack — everything except the mission node.

Brings up:
  - WRORobot/hardware.launch.py     — drivers + SLAM + Nav2 navigation
  - waypoint_nav_bridge/bridge      — /waypoint (PoseStamped) -> Nav2 action
  - wro_behavior/camera_map_augmenter — camera detections -> /camera_obstacles

The mission node is added by the challenge-specific launch on top of
this (open_challenge_hw.launch.py / obstacle_challenge_hw.launch.py).

Args:
  params_file — Nav2 params yaml (default: wro_nav2/params/nav2_params.yaml).
                open_challenge_hw passes a RewrittenYaml overlay built
                from wro_behavior/config/open_tuning.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('wro_nav2'),
            'params',
            'nav2_params.yaml',
        ]),
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('WRORobot'),
                'launch',
                'hardware.launch.py',
            ])
        ),
        launch_arguments={'params_file': params_file}.items(),
    )

    waypoint_bridge = Node(
        package='waypoint_nav_bridge',
        executable='bridge',
        name='waypoint_nav_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    camera_map_augmenter = Node(
        package='wro_behavior',
        executable='camera_map_augmenter',
        name='camera_map_augmenter',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        declare_params_file,
        hardware,
        waypoint_bridge,
        camera_map_augmenter,
    ])
