"""Real-robot hardware layer — drivers only, no Nav2, no behavior, no SLAM.

Starts:
  - lidar (ldlidar_ros2)
  - IMU driver (lsm9ds1_handler) — /imu/data_raw for logging; not fused
  - robot_state_publisher with the shared URDF (WRORobot/urdf/wro_bot.urdf.xacro)
  - Motor (ESC PWM)
  - Servo (steering PWM)
  - HighController
  - otos_node — sole publisher of odom -> base_link

Pair this with wro_nav2/nav2.launch.py and wro_behavior/behavior.launch.py to
run the full real-robot stack, or use robot_stack.launch.py which composes
all three. See INTERFACE.md at repo root for the topic/frame contract.

odom -> base_link ownership: OTOS is the sole publisher. imu_filter_madgwick
and ekf_filter_node were removed 2026-08-30 (see git log). To re-enable EKF
fusion later, add otos as an odom0 input to ekf.yaml and re-add the
ekf_node here; disable otos_node's tf broadcast in otos_node.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def _robot_description():
    """Process the shared URDF xacro at launch time."""
    urdf_xacro = os.path.join(
        get_package_share_directory('WRORobot'),
        'urdf', 'wro_bot.urdf.xacro',
    )
    return xacro.process_file(urdf_xacro).toxml()


def generate_launch_description():
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lsm9ds1_handler'),
                'launch',
                'lsm9ds1_handler.launch.py'
            )
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ldlidar_ros2'),
                'launch',
                'ld19.launch.py'
            )
        )
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': _robot_description(),
        }],
    )

    return LaunchDescription([
        lidar_launch,
        imu_launch,
        rsp,

        Node(
            package='WRORobot',
            executable='HighController',
            name='HighController',
            output='screen'
        ),

        Node(
            package='WRORobot',
            executable='Motor',
            name='Motor',
            output='screen'
        ),

        Node(
            package='WRORobot',
            executable='Servo',
            name='Servo',
            output='screen'
        ),

        Node(
            package='WRORobot',
            executable='otos_node',
            name='otos_odometry_node',
            output='screen'
        ),
    ])
