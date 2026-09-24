"""Open Challenge hw launcher.

Loads the shared hardware stack and adds the OpenMission node on top.
Both the mission node's tunables AND the Nav2 tunables come from
wro_behavior/config/open_tuning.yaml — edit that file, relaunch, no
rebuild needed.

Nav2 tunables from the yaml's `nav2_overrides` section are merged onto
wro_nav2/params/nav2_params.yaml at launch-generation time and
written to /tmp. Nav2 reads that pre-materialized file directly.

Run:
  ros2 launch wro_behavior open_challenge_hw.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from wro_behavior.launch_helpers import build_merged_nav2_params, load_mission_params


def generate_launch_description():
    behavior_share = get_package_share_directory('wro_behavior')
    nav2_share = get_package_share_directory('wro_nav2')

    tuning_yaml = os.path.join(behavior_share, 'config', 'open_tuning.yaml')
    nav2_base = os.path.join(nav2_share, 'params', 'nav2_params.yaml')

    mission_params = load_mission_params(tuning_yaml, 'open_mission')
    merged_nav2 = build_merged_nav2_params(
        tuning_yaml, nav2_base, out_name='open_nav2_hw.yaml')

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(behavior_share, 'launch', 'wro_stack_hw.launch.py'),
        ),
        launch_arguments={
            'params_file': merged_nav2,
        }.items(),
    )

    mission = Node(
        package='wro_behavior',
        executable='open_mission',
        name='open_mission',
        output='screen',
        parameters=[
            mission_params,
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([stack, mission])
