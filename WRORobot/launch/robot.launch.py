from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('WRORobot')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config_path = os.path.join(pkg_share, 'config', 'slam_params.yaml')
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

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_config_path
        }.items()
    )
    return LaunchDescription([
        slam_launch,
        lidar_launch,
        imu_launch,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_link',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'base_link',
                'imu_link'
            ],
            output='screen'
        ),


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
            package='imu_filter_madgwick',
       	    executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu'
            }]
        ),



	Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
	    parameters=[ekf_config_path]
	),


    ])
