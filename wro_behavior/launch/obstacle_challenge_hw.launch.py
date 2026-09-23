"""Obstacle Challenge hw launcher.

Loads the shared hardware stack and adds the ObstacleMission node on top.

Run:
  ros2 launch wro_behavior obstacle_challenge_hw.launch.py
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
                'wro_stack_hw.launch.py',
            ])
        ),
    )

    mission = Node(
        package='wro_behavior',
        executable='obstacle_mission',
        name='obstacle_mission',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            # Physical start button on the Teensy has been unreliable —
            # kick off automatically instead. Flip to True once the
            # /start_button signal is trustworthy.
            {'wait_for_start': False},
        ],
    )

    return LaunchDescription([stack, mission])
