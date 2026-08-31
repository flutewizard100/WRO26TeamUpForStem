"""Behavior-layer launch.

Starts the mission-level nodes that consume Nav2's action/topic contract.
This launch never depends on hardware drivers or sim internals — pair it
with either wro_sim/sim.launch.py or WRORobot/hardware.launch.py plus
wro_nav2/nav2.launch.py.

Right now the only node is the goto_pose hello-world. Add real mission
state machines / BT nodes here as they are written.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_goto = LaunchConfiguration('run_goto')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_run_goto = DeclareLaunchArgument(
        'run_goto', default_value='false',
        description='Fire the goto_pose hello-world node on launch.')
    declare_goal_x = DeclareLaunchArgument('goal_x', default_value='1.0')
    declare_goal_y = DeclareLaunchArgument('goal_y', default_value='0.0')
    declare_goal_yaw = DeclareLaunchArgument('goal_yaw', default_value='0.0')

    goto = Node(
        package='wro_behavior',
        executable='goto_pose',
        name='wro_goto_pose',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'x': goal_x,
            'y': goal_y,
            'yaw': goal_yaw,
        }],
        condition=IfCondition(run_goto),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_run_goto,
        declare_goal_x, declare_goal_y, declare_goal_yaw,
        goto,
    ])
