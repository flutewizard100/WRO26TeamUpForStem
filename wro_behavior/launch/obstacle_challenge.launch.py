from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('WRORobot'),
                'launch',
                'hardware.launch.py',
            ])
        ]),
    )

    open_challenge = Node(
        package='wro_behavior',
        executable='obstacle_challenge_template',
        name='obstacle_challenge_template',
        output='screen',
    )

    return LaunchDescription([
        hardware,
        open_challenge,
    ])

