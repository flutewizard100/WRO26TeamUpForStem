"""Obstacle Challenge sim launcher.

Loads the shared sim stack with obstacles enabled (pillars in the
field) and adds the ObstacleMission node on top.

Run:
  ros2 launch wro_behavior obstacle_challenge_sim.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wro_behavior'),
                'launch',
                'wro_stack_sim.launch.py',
            ])
        ),
        launch_arguments={'obstacles': 'true'}.items(),
    )

    mission = Node(
        package='wro_behavior',
        executable='obstacle_mission',
        name='obstacle_mission',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            # Sim has no physical start button — kick off automatically.
            {'wait_for_start': False},
        ],
    )

    return LaunchDescription([stack, mission])
