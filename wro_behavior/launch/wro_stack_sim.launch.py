"""Shared sim stack — everything except the mission node.

Brings up:
  - wro_sim/sim.launch.py           — Gazebo + spawn + ros_gz bridges
  - wro_nav2/slam.launch.py         — SLAM Toolbox + Nav2 (planner + MPPI)
  - waypoint_nav_bridge/bridge      — /waypoint (PoseStamped) -> Nav2 action
  - wro_behavior/sim_limelight_bridge — fake Limelight (HSV detections)
  - wro_behavior/camera_map_augmenter — camera detections -> /camera_obstacles

The mission node is added by the challenge-specific launch on top of
this (open_challenge_sim.launch.py / obstacle_challenge_sim.launch.py).

Args:
  obstacles      — sim world flag: true loads the pillars, false skips them.
  params_file    — Nav2 params yaml (default: wro_nav2/params/nav2_params_sim.yaml).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    obstacles = LaunchConfiguration('obstacles')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('wro_nav2'),
            'params',
            'nav2_params_sim.yaml',
        ]),
    )
    declare_obstacles = DeclareLaunchArgument(
        'obstacles',
        default_value='true',
        description='true = load pillars in sim world. false = plain field.',
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wro_sim'),
                'launch',
                'sim.launch.py',
            ])
        ),
        launch_arguments={
            'rviz': 'true',
            'obstacles': obstacles,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wro_nav2'),
                'launch',
                'slam.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': params_file,
        }.items(),
    )

    waypoint_bridge = Node(
        package='waypoint_nav_bridge',
        executable='bridge',
        name='waypoint_nav_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    sim_limelight = Node(
        package='wro_behavior',
        executable='sim_limelight_bridge',
        name='sim_limelight_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    camera_map_augmenter = Node(
        package='wro_behavior',
        executable='camera_map_augmenter',
        name='camera_map_augmenter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_params_file,
        declare_obstacles,
        simulator,
        navigation,
        waypoint_bridge,
        sim_limelight,
        camera_map_augmenter,
    ])
