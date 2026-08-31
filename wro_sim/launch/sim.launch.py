"""Launch Gazebo Harmonic with the wro_field world and spawn the wro_bot.

Starts:
  - gz sim (via ros_gz_sim gz_sim.launch.py)
  - robot_state_publisher with the xacro-processed URDF
  - ros_gz_sim create to spawn wro_bot at the requested pose
  - parameter_bridge with our YAML bridge config
  - image_bridge for /camera (raw image)
  - (optional, rviz:=true) rviz2 with sim.rviz
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def _robot_description(context):
    """Process the URDF xacro at launch time.

    The URDF lives in the WRORobot package so both the sim and the real
    robot consume the same file — see INTERFACE.md.
    """
    urdf_xacro = os.path.join(
        get_package_share_directory('WRORobot'),
        'urdf', 'wro_bot.urdf.xacro',
    )
    doc = xacro.process_file(urdf_xacro)
    return doc.toxml()


def _make_rsp(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time.lower() in ('true', '1'),
            'robot_description': _robot_description(context),
        }],
    )]


def generate_launch_description():
    pkg_wro_sim = get_package_share_directory('wro_sim')

    default_world = os.path.join(pkg_wro_sim, 'worlds', 'wro_field.sdf')
    default_bridge = os.path.join(pkg_wro_sim, 'config', 'ros_gz_bridge.yaml')
    default_rviz = os.path.join(pkg_wro_sim, 'rviz', 'sim.rviz')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw_pose = LaunchConfiguration('yaw')

    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false')
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='-1.3')
    declare_z = DeclareLaunchArgument('z', default_value='0.103')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='1.5708')

    # Start gz sim with our world.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'gz_args': [world, ' -s -r -v 3'],
        }.items(),
    )

    # robot_state_publisher (needs xacro-processed URDF at launch time).
    rsp = OpaqueFunction(function=_make_rsp)

    # Spawn the robot at the requested pose. ros_gz_sim's create executable
    # pulls the model definition from robot_description.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_wro_bot',
        arguments=[
            '-name', 'wro_bot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose,
        ],
        output='screen',
    )

    # ros_gz_bridge parameter_bridge reads bridge pairs from a YAML config
    # passed via the `config_file` ROS parameter (canonical Jazzy form —
    # see ros_gz_bridge README).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': default_bridge,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # Raw image bridge (ros_gz_image handles image_transport-aware bridging).
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_image_bridge',
        arguments=['/camera'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_world,
        declare_use_sim_time,
        declare_rviz,
        declare_x, declare_y, declare_z, declare_yaw,
        gz_sim,
        rsp,
        spawn,
        bridge,
        image_bridge,
        rviz2,
    ])
