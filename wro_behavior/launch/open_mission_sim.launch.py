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
            'nav2_params_sim.yaml',
        ])
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wro_sim'),
                'launch',
                'sim.launch.py',
            ])
        )
    )

    waypoint_bridge = Node(
        package='waypoint_nav_bridge',
        executable='bridge',
        name='waypoint_nav_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Starts both SLAM Toolbox and Nav2.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'slam': 'True',
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
            'params_file': params_file,
        }.items(),
    )

    waypoints = Node(
        package='wro_behavior',
        executable='waypoints',
        name='waypoints',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    open_challenge = Node(
        package='wro_behavior',
        executable='open_mission',
        name='open_mission',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_params_file,
        simulator,
        waypoint_bridge,
        navigation,
        waypoints,
        open_challenge,
    ])