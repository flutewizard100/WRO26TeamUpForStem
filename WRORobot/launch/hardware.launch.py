"""Real-robot hardware layer — drivers + local-pose fusion. No Nav2, no
behavior, no SLAM.

Starts:
  - lidar (ldlidar_ros2)
  - IMU driver (lsm9ds1_handler) — /imu/data_raw
  - imu_filter_madgwick — /imu/data_raw + accel/gyro -> /imu/data (no mag,
    LSM9DS1 handler doesn't publish MagneticField; task #22 is the future
    fix if absolute yaw fallback becomes critical)
  - laser_scan_matcher — /scan -> /odometry/laser (scan-to-scan odom)
  - otos_node — publishes /odom (no TF; EKF owns odom->base_link now)
  - ekf_filter_node — fuses /odom (OTOS), /odometry/laser (LiDAR),
    /imu/data (IMU) into filtered odom->base_link. Graceful degradation:
    if OTOS goes silent for 0.3s EKF drops it and continues on the
    remaining sources.
  - robot_state_publisher with the shared URDF (WRORobot/urdf/wro_bot.urdf.xacro)
  - Motor (ESC PWM)
  - Servo (steering PWM)
  - HighController
  - limelight_bridge — Limelight 3 neural detector, publishes
    vision_msgs/Detection2DArray on /limelight/detections

Pair this with wro_nav2/nav2.launch.py and wro_behavior/behavior.launch.py to
run the full real-robot stack, or use robot_stack.launch.py which composes
all three. See INTERFACE.md at repo root for the topic/frame contract.

odom -> base_link ownership: ekf_filter_node is the sole publisher when
fusion is enabled (see config/ekf.yaml). To run OTOS-standalone (no
fusion), remove the ekf/madgwick/scan_matcher nodes below and set
otos_node's `publish_tf` param to True.
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


def _config_path(basename):
    return os.path.join(
        get_package_share_directory('WRORobot'),
        'config',
        basename,
    )


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
            output='screen',
            parameters=[{'publish_tf': False}],  # EKF owns odom->base_link now
        ),

        # imu_filter_madgwick — /imu/data_raw -> /imu/data (orientation
        # quaternion). No magnetometer input; yaw is gyro-integrated and will
        # drift. See task #22 to add /imu/mag if absolute yaw fallback is
        # needed.
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
                'fixed_frame': 'odom',
            }],
        ),

        # robot_localization EKF — fuses OTOS + IMU + scan matcher into a
        # single filtered odometry stream and publishes odom->base_link TF.
        # Sensor timeout is set in the yaml; if OTOS drops out for >0.3s,
        # EKF continues integrating from the remaining sources.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[_config_path('ekf.yaml')],
        ),

        # Limelight 3 vision bridge. Publishes vision_msgs/Detection2DArray
        # on /limelight/detections. Intrinsic parameters are nominal LL3
        # 640x480 defaults — run task #15 (intrinsic calibration) and pass
        # measured fx/fy/cx/cy via parameters= for accurate 3D pose.
        Node(
            package='WRORobot',
            executable='limelight_bridge',
            name='limelight_bridge',
            output='screen',
        ),
    ])
