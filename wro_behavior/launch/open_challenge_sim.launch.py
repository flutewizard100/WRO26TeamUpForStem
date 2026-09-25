"""Open Challenge sim launcher.

Loads the shared sim stack (with obstacles disabled) and adds the
OpenMission node on top. Both the mission node's tunables AND the
Nav2 tunables come from wro_behavior/config/open_tuning.yaml — edit
that file, relaunch, no rebuild needed.

Nav2 tunables from the yaml's `nav2_overrides` section are merged onto
wro_nav2/params/nav2_params_sim.yaml at launch-generation time (once,
in this Python file) and written to /tmp. Nav2 then reads that
pre-materialized file like any other params yaml — no per-node
substitution overhead.

Args:
  rviz         — true to launch RViz2 alongside the sim (default false).
  gazebo_gui   — true to run Gazebo with GUI, false for headless
                 (default false; saves ~2-4s of startup).

Run:
  ros2 launch wro_behavior open_challenge_sim.launch.py
  ros2 launch wro_behavior open_challenge_sim.launch.py rviz:=true
  ros2 launch wro_behavior open_challenge_sim.launch.py gazebo_gui:=true rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wro_behavior.launch_helpers import build_merged_nav2_params, load_mission_params


def generate_launch_description():
    behavior_share = get_package_share_directory('wro_behavior')
    nav2_share = get_package_share_directory('wro_nav2')

    tuning_yaml = os.path.join(behavior_share, 'config', 'open_tuning.yaml')
    nav2_base = os.path.join(nav2_share, 'params', 'nav2_params_sim.yaml')

    mission_params = load_mission_params(tuning_yaml, 'open_mission')
    merged_nav2 = build_merged_nav2_params(
        tuning_yaml, nav2_base, out_name='open_nav2_sim.yaml')

    rviz = LaunchConfiguration('rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 alongside the sim.')
    declare_gazebo_gui = DeclareLaunchArgument(
        'gazebo_gui', default_value='false',
        description='Run Gazebo with GUI (default headless).')

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(behavior_share, 'launch', 'wro_stack_sim.launch.py'),
        ),
        launch_arguments={
            'obstacles': 'false',
            'params_file': merged_nav2,
            'rviz': rviz,
            'gazebo_gui': gazebo_gui,
        }.items(),
    )

    mission = Node(
        package='wro_behavior',
        executable='open_mission',
        name='open_mission',
        output='screen',
        parameters=[
            mission_params,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        declare_rviz,
        declare_gazebo_gui,
        stack,
        mission,
    ])
