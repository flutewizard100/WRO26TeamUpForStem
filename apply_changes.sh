#!/usr/bin/env bash
#
# apply_changes.sh
#
# Reproduces every change made during the session on 2026-08-30 that split the
# WRO26 codebase into three cleanly-separated ROS 2 layers:
#
#   * WRORobot/      -- real-hardware drivers (existed already; small edits)
#   * wro_nav2/      -- Nav2 params/BT/maps/launch (new package)
#   * wro_sim/       -- Gazebo Harmonic sim + bridge (new package)
#   * wro_behavior/  -- mission logic (new package)
#   * INTERFACE.md   -- the topic/frame/action contract between the layers
#
# Run this on a fresh Ubuntu 24.04 machine with ROS 2 Jazzy installed, with
# the repo cloned to ~/WRO26TeamUpForStem at commit 28aa9b4 (a clean tree),
# from the repo root:
#
#     cd ~/WRO26TeamUpForStem && bash apply_changes.sh
#
# This script does NOT run apt/rosdep, does NOT build, and does NOT commit.
# It only writes files. Sanity checks are at the bottom; commit guidance is
# in a commented-out block at the very end for you to run manually.

set -euo pipefail

# =============================================================================
# 1. Preflight
# =============================================================================
# Why: fail fast if we're in the wrong directory or the tree isn't in the
# expected clean state at commit 28aa9b4. Every subsequent step assumes we're
# at the repo root, so a wrong-directory mistake here would scatter garbage
# files into whatever cwd the user happens to be in.

EXPECTED_COMMIT="28aa9b4"

if [[ ! -d WRORobot || ! -d ldlidar_ros2 || ! -f README.md ]]; then
  echo "ERROR: This script must be run from the repo root (~/WRO26TeamUpForStem)." >&2
  echo "       Missing one of: WRORobot/, ldlidar_ros2/, README.md" >&2
  exit 1
fi

if ! command -v git >/dev/null 2>&1; then
  echo "ERROR: git is required." >&2
  exit 1
fi

CUR_COMMIT="$(git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)"
if [[ "${CUR_COMMIT}" != "${EXPECTED_COMMIT}" ]]; then
  echo "WARNING: HEAD is at ${CUR_COMMIT}, expected ${EXPECTED_COMMIT}." >&2
  echo "         This script was authored against ${EXPECTED_COMMIT}. Continuing," >&2
  echo "         but review the diff carefully before committing." >&2
fi

# The initial checkout on Linux should be clean. On Windows the same clone
# looked like a mass "staged deletion" because of core.autocrlf/permission
# quirks -- that was a Windows-side artifact and does not need reproducing
# here. We warn (rather than abort) if the tree isn't clean, because the user
# may deliberately be re-running this script over a partial run.
if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "NOTE: working tree is not clean. This script will overwrite files;" >&2
  echo "      that is intentional. Continuing." >&2
fi

echo "==> Preflight OK. Applying changes at ${CUR_COMMIT}."

# =============================================================================
# 2. Restore staged deletions (Windows -> Linux only)
# =============================================================================
# Why: on the Windows machine where the session ran, `git status` showed the
# entire tracked tree as "staged for deletion" (D) because of a line-ending /
# permissions mismatch. On a fresh Linux checkout at 28aa9b4 the tree is
# clean, so this section is normally a no-op. Left in for the pathological
# case where the same corruption reappears -- `git reset` clears the index
# but keeps the working tree; `git checkout .` restores anything that was
# actually removed from disk. Both are safe on a truly clean tree.

if git diff --cached --name-only | grep -q .; then
  echo "==> Staged deletions detected; running 'git reset' to clear the index."
  git reset
fi

# =============================================================================
# 3. Fix WRORobot/setup.py
# =============================================================================
# Why: two changes in one file.
#   (a) Case typo: `wroRobot.otos_node:main` -> `WRORobot.otos_node:main`.
#       Python is case-sensitive; the lowercase entrypoint would fail at
#       runtime with `ModuleNotFoundError: No module named 'wroRobot'`.
#   (b) Add urdf/*.urdf.xacro to data_files so the shared URDF gets installed
#       into WRORobot's share dir. Both hardware.launch.py and the sim's
#       sim.launch.py look up the URDF via get_package_share_directory
#       ('WRORobot')/urdf/wro_bot.urdf.xacro -- if it isn't installed there,
#       both stacks fail at launch.
#
# The original file has an unusual single-line entry in data_files (the
# launch tuple sits on the same line as the package.xml tuple) which is
# fragile to sed against. Replace the whole file with a heredoc instead.

echo "==> Writing WRORobot/setup.py"
cat > WRORobot/setup.py <<'EOF'
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'WRORobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),( os.path.join('share', package_name, 'launch') , glob(os.path.join('launch', '*launch.[pxy][yma]*')) ),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf.xacro'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wroteam',
    maintainer_email='sagnikbiswas712@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'HighController = WRORobot.RobotController:main',
            'Motor = WRORobot.Motor:main',
            'Servo = WRORobot.Servo:main',
            'otos_node = WRORobot.otos_node:main',
        ],
    },

)
EOF

# =============================================================================
# 4. Update WRORobot/package.xml
# =============================================================================
# Why: WRORobot's launches now depend on packages it didn't declare before:
#   - robot_state_publisher / xacro: hardware.launch.py processes the URDF
#     and starts robot_state_publisher.
#   - ldlidar_ros2: hardware.launch.py includes ld19.launch.py for the lidar.
#   - wro_nav2 / wro_behavior: robot_stack.launch.py includes the Nav2 and
#     behavior launch files. These are workspace-local, so rosdep can't
#     resolve them -- callers must pass `rosdep --skip-keys "wro_nav2
#     wro_behavior"`. That's called out in the comment inside the file.
# Whole-file replacement (safer than a sed insert against XML).

echo "==> Writing WRORobot/package.xml"
cat > WRORobot/package.xml <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>WRORobot</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="sagnikbiswas712@gmail.com">wroteam</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <depend>lsm9ds1_handler</depend>
  <depend>rclpy</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>ldlidar_ros2</exec_depend>
  <!-- robot_stack.launch.py composes wro_nav2 and wro_behavior; both are
       workspace-local (pass rosdep --skip-keys "wro_nav2 wro_behavior"). -->
  <exec_depend>wro_nav2</exec_depend>
  <exec_depend>wro_behavior</exec_depend>


  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# =============================================================================
# 5. Move URDF into WRORobot
# =============================================================================
# Why: the URDF is the single source of truth for physical dimensions AND
# it's the file both stacks (sim + real) need. Placing it in WRORobot rather
# than wro_sim/models/ means the real-robot hardware.launch.py can consume
# it without depending on wro_sim (which pulls in Gazebo -- a very heavy
# dependency the real robot doesn't need). setup.py already installs
# urdf/*.urdf.xacro to share/WRORobot/urdf/ (see section 3).

echo "==> Writing WRORobot/urdf/wro_bot.urdf.xacro"
mkdir -p WRORobot/urdf
cat > WRORobot/urdf/wro_bot.urdf.xacro <<'EOF'
<?xml version="1.0"?>
<!--
  WRO Ackermann robot model for Gazebo Harmonic.

  All dimensions here are placeholders. Every one of them is marked
  TODO(measure): with a pointer to where the real value lives (physical
  robot, or the WRO Chassis CAD in the repo root).
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="wro_bot">

  <!-- ============================================================
       Geometry parameters. Replace these with measured values.
       ============================================================ -->
  <!-- TODO(measure): axle-to-axle distance from the real chassis / CAD -->
  <xacro:property name="wheelbase" value="0.16"/>
  <!-- TODO(measure): left-to-right wheel centerline distance -->
  <xacro:property name="track_width" value="0.14"/>
  <!-- TODO(measure): wheel rolling radius (including tire) -->
  <xacro:property name="wheel_radius" value="0.033"/>
  <!-- TODO(measure): tire width -->
  <xacro:property name="wheel_thickness" value="0.025"/>
  <!-- TODO(measure): mechanical max steer of the front wheels (30 deg) -->
  <xacro:property name="max_steer_angle" value="0.5236"/>

  <!-- TODO(measure): chassis outer box from CAD -->
  <xacro:property name="chassis_length" value="0.28"/>
  <xacro:property name="chassis_width" value="0.16"/>
  <xacro:property name="chassis_height" value="0.08"/>

  <!-- Sensor mount offsets (all in base_link frame). -->
  <!-- TODO(measure): lidar mount position on real bot -->
  <xacro:property name="lidar_xyz" value="0.0 0.0 0.10"/>
  <!-- TODO(measure): IMU mount position on real bot -->
  <xacro:property name="imu_xyz" value="0.0 0.0 0.03"/>
  <!-- TODO(measure): camera mount position on real bot (front-facing).
       Component numbers to keep this trivially editable -- camera lives at
       (chassis_length/2 - 0.01, 0.0, 0.06). -->
  <xacro:property name="camera_x" value="${chassis_length/2 - 0.01}"/>
  <xacro:property name="camera_y" value="0.0"/>
  <xacro:property name="camera_z" value="0.06"/>

  <!-- Derived. -->
  <xacro:property name="half_l" value="${wheelbase/2}"/>
  <xacro:property name="half_t" value="${track_width/2}"/>
  <xacro:property name="chassis_z" value="${wheel_radius + chassis_height/2}"/>

  <!-- ============================================================
       Materials (visual only).
       ============================================================ -->
  <material name="wro_body_grey">
    <color rgba="0.35 0.35 0.4 1.0"/>
  </material>
  <material name="wro_wheel_black">
    <color rgba="0.05 0.05 0.05 1.0"/>
  </material>
  <material name="wro_sensor_blue">
    <color rgba="0.2 0.4 0.8 1.0"/>
  </material>

  <!-- ============================================================
       Reusable macros.
       ============================================================ -->
  <xacro:macro name="inertial_box" params="mass x y z">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia
        ixx="${(1.0/12.0)*mass*(y*y + z*z)}" ixy="0" ixz="0"
        iyy="${(1.0/12.0)*mass*(x*x + z*z)}" iyz="0"
        izz="${(1.0/12.0)*mass*(x*x + y*y)}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="inertial_cyl" params="mass r l">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia
        ixx="${(1.0/12.0)*mass*(3*r*r + l*l)}" ixy="0" ixz="0"
        iyy="${(1.0/12.0)*mass*(3*r*r + l*l)}" iyz="0"
        izz="${0.5*mass*r*r}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="wheel_link" params="name">
    <link name="${name}">
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_thickness}"/>
        </geometry>
        <material name="wro_wheel_black"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_thickness}"/>
        </geometry>
      </collision>
      <xacro:inertial_cyl mass="0.05" r="${wheel_radius}" l="${wheel_thickness}"/>
    </link>
  </xacro:macro>

  <!-- ============================================================
       Frames.

       base_link is the tree root. The AckermannSteering plugin publishes
       odom -> base_link on /model/wro_bot/tf; robot_state_publisher owns
       every other transform. We do NOT define base_footprint (it would
       give base_link two parents once the plugin's tf is bridged).

       Spawn the model with z = wheel_radius + chassis_height/2 so wheels
       touch the ground (see wro_sim/launch/gz_sim.launch.py 'z' default).
       ============================================================ -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <material name="wro_body_grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
    </collision>
    <xacro:inertial_box mass="1.5" x="${chassis_length}" y="${chassis_width}" z="${chassis_height}"/>
  </link>

  <!-- Steering knuckles (front left, front right). -->
  <link name="front_left_wheel_steer">
    <xacro:inertial_box mass="0.02" x="0.02" y="0.02" z="0.02"/>
  </link>
  <link name="front_right_wheel_steer">
    <xacro:inertial_box mass="0.02" x="0.02" y="0.02" z="0.02"/>
  </link>

  <joint name="front_left_wheel_steer_joint" type="revolute">
    <parent link="base_link"/>
    <child link="front_left_wheel_steer"/>
    <origin xyz="${half_l} ${half_t} ${-chassis_height/2}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-max_steer_angle}" upper="${max_steer_angle}"
           effort="10.0" velocity="5.0"/>
  </joint>

  <joint name="front_right_wheel_steer_joint" type="revolute">
    <parent link="base_link"/>
    <child link="front_right_wheel_steer"/>
    <origin xyz="${half_l} ${-half_t} ${-chassis_height/2}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-max_steer_angle}" upper="${max_steer_angle}"
           effort="10.0" velocity="5.0"/>
  </joint>

  <!-- Wheels. -->
  <xacro:wheel_link name="front_left_wheel"/>
  <xacro:wheel_link name="front_right_wheel"/>
  <xacro:wheel_link name="rear_left_wheel"/>
  <xacro:wheel_link name="rear_right_wheel"/>

  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="front_left_wheel_steer"/>
    <child link="front_left_wheel"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="front_right_wheel_steer"/>
    <child link="front_right_wheel"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_left_wheel"/>
    <origin xyz="${-half_l} ${half_t} ${-chassis_height/2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_right_wheel"/>
    <origin xyz="${-half_l} ${-half_t} ${-chassis_height/2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Sensor frames. -->
  <link name="base_laser">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.03"/>
      </geometry>
      <material name="wro_sensor_blue"/>
    </visual>
    <xacro:inertial_cyl mass="0.05" r="0.025" l="0.03"/>
  </link>

  <joint name="base_link_to_base_laser" type="fixed">
    <parent link="base_link"/>
    <child link="base_laser"/>
    <origin xyz="${lidar_xyz}" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <xacro:inertial_box mass="0.01" x="0.02" y="0.02" z="0.005"/>
  </link>

  <joint name="base_link_to_imu_link" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="${imu_xyz}" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.04 0.02"/>
      </geometry>
      <material name="wro_sensor_blue"/>
    </visual>
    <xacro:inertial_box mass="0.02" x="0.02" y="0.04" z="0.02"/>
  </link>

  <joint name="base_link_to_camera_link" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${camera_x} ${camera_y} ${camera_z}" rpy="0 0 0"/>
  </joint>

  <!-- ROS image convention frame (Z forward, X right, Y down). Attach the
       Gazebo camera sensor here so the ROS bridge image already lives in the
       optical frame. -->
  <link name="camera_optical_frame"/>
  <joint name="camera_link_to_optical" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  </joint>

  <!-- ============================================================
       Gazebo Harmonic plugin fragments.
       ============================================================ -->

  <!-- Ackermann drive. Publishes odom on /model/wro_bot/odometry and tf on
       /model/wro_bot/tf (bridged to /tf on the ROS side). Consumes Twist on
       /model/wro_bot/cmd_vel (bridged from /cmd_vel). -->
  <gazebo>
    <plugin filename="gz-sim-ackermann-steering-system"
            name="gz::sim::systems::AckermannSteering">
      <left_joint>front_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <left_steering_joint>front_left_wheel_steer_joint</left_steering_joint>
      <right_steering_joint>front_right_wheel_steer_joint</right_steering_joint>
      <kingpin_width>${track_width}</kingpin_width>
      <steering_limit>${max_steer_angle}</steering_limit>
      <wheel_base>${wheelbase}</wheel_base>
      <wheel_separation>${track_width}</wheel_separation>
      <wheel_radius>${wheel_radius}</wheel_radius>
      <min_velocity>-1.0</min_velocity>
      <max_velocity>1.0</max_velocity>
      <min_acceleration>-2.0</min_acceleration>
      <max_acceleration>2.0</max_acceleration>
      <topic>/model/wro_bot/cmd_vel</topic>
      <odom_topic>/model/wro_bot/odometry</odom_topic>
      <tf_topic>/model/wro_bot/tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>
  </gazebo>

  <!-- Joint states -->
  <gazebo>
    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
      <topic>/model/wro_bot/joint_state</topic>
    </plugin>
  </gazebo>

  <!-- GPU Lidar on base_laser. -->
  <gazebo reference="base_laser">
    <sensor name="wro_lidar" type="gpu_lidar">
      <topic>/lidar</topic>
      <update_rate>10</update_rate>
      <always_on>1</always_on>
      <visualize>false</visualize>
      <gz_frame_id>base_laser</gz_frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.141592653589793</min_angle>
            <max_angle>3.141592653589793</max_angle>
          </horizontal>
          <vertical>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>0.0</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.02</min>
          <max>8.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </lidar>
    </sensor>
  </gazebo>

  <!-- IMU on imu_link. -->
  <gazebo reference="imu_link">
    <sensor name="wro_imu" type="imu">
      <topic>/imu</topic>
      <update_rate>100</update_rate>
      <always_on>1</always_on>
      <gz_frame_id>imu_link</gz_frame_id>
    </sensor>
  </gazebo>

  <!-- RGB camera on camera_link (attached to the optical frame so the ROS
       image already lives in optical convention). -->
  <gazebo reference="camera_link">
    <sensor name="wro_camera" type="camera">
      <topic>/camera</topic>
      <update_rate>30</update_rate>
      <always_on>1</always_on>
      <gz_frame_id>camera_optical_frame</gz_frame_id>
      <camera>
        <horizontal_fov>1.0472</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>50.0</far>
        </clip>
      </camera>
    </sensor>
  </gazebo>

</robot>
EOF

# =============================================================================
# 6. Split WRORobot launch files
# =============================================================================
# Why: the old robot.launch.py mixed drivers, EKF fusion, and mission-level
# concerns in one file, and duplicated concerns with Nav2 launches. The new
# split follows the "layers only ever compose downward" rule:
#   - hardware.launch.py       -- drivers + robot_state_publisher only
#   - robot_stack.launch.py    -- hardware + wro_nav2 + wro_behavior
# The wro_nav2 and wro_behavior packages don't know hardware exists; the
# hardware package doesn't know Nav2 exists. Only robot_stack composes them.

echo "==> Removing WRORobot/launch/robot.launch.py (replaced by hardware.launch.py + robot_stack.launch.py)"
rm -f WRORobot/launch/robot.launch.py

echo "==> Writing WRORobot/launch/hardware.launch.py"
mkdir -p WRORobot/launch
cat > WRORobot/launch/hardware.launch.py <<'EOF'
"""Real-robot hardware layer -- drivers only, no Nav2, no behavior, no SLAM.

Starts:
  - lidar (ldlidar_ros2)
  - IMU driver (lsm9ds1_handler) -- /imu/data_raw for logging; not fused
  - robot_state_publisher with the shared URDF (WRORobot/urdf/wro_bot.urdf.xacro)
  - Motor (ESC PWM)
  - Servo (steering PWM)
  - HighController
  - otos_node -- sole publisher of odom -> base_link

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
EOF

echo "==> Writing WRORobot/launch/robot_stack.launch.py"
cat > WRORobot/launch/robot_stack.launch.py <<'EOF'
"""Top-level real-robot stack: hardware + Nav2 + behavior.

This is the hardware engineer's default entry point. Composes:
  1. WRORobot/hardware.launch.py       -- drivers + robot_state_publisher
  2. wro_nav2/nav2.launch.py           -- Nav2 stack (or slam.launch.py if slam:=true)
  3. wro_behavior/behavior.launch.py   -- mission nodes (off by default)

Counterpart to wro_sim/sim_stack.launch.py -- same nav + behavior layers,
hardware in place of sim.

Args:
  slam:=true      -> use slam_toolbox instead of AMCL/map_server
  run_goto:=true  -> fire the wro_behavior goto_pose hello-world on launch
  map:=<path>     -> map yaml to localize against (defaults to wro_nav2's placeholder)

See INTERFACE.md at repo root for the topic/frame/action contract.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_wro_nav2 = get_package_share_directory('wro_nav2')

    default_params = os.path.join(pkg_wro_nav2, 'params', 'nav2_params.yaml')
    default_map = os.path.join(pkg_wro_nav2, 'maps', 'wro_field.yaml')

    slam = LaunchConfiguration('slam')
    run_goto = LaunchConfiguration('run_goto')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_slam = DeclareLaunchArgument('slam', default_value='false')
    declare_run_goto = DeclareLaunchArgument(
        'run_goto', default_value='false')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    declare_map = DeclareLaunchArgument('map', default_value=default_map)

    # 1. Real hardware -- drivers + robot_state_publisher.
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('WRORobot'),
                'launch', 'hardware.launch.py',
            ])
        ]),
    )

    # 2a. Full Nav2 with AMCL (default).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_nav2'),
                'launch', 'nav2.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'map': map_yaml,
            'use_composition': 'False',
        }.items(),
        condition=UnlessCondition(slam),
    )

    # 2b. SLAM Toolbox + Nav2 planning stack.
    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_nav2'),
                'launch', 'slam.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
        }.items(),
        condition=IfCondition(slam),
    )

    # 3. Behavior layer.
    behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_behavior'),
                'launch', 'behavior.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'run_goto': run_goto,
        }.items(),
    )

    return LaunchDescription([
        declare_slam, declare_run_goto, declare_params, declare_map,
        hardware,
        nav2,
        slam_bringup,
        behavior,
    ])
EOF

# =============================================================================
# 7. Create wro_nav2/ package
# =============================================================================
# Why: Nav2 config -- params/BT/maps/RViz/launches -- lives in its own
# package so wro_sim and WRORobot can both include the same launches
# unchanged. Config-only; no C++/Python nodes. Key design choices:
#
#   * Planner: SmacPlannerHybrid with DUBIN motion model and
#     minimum_turning_radius = 0.30 m (robot is Ackermann; can't turn
#     tighter than wheelbase/tan(max_steer) ~= 0.277 m).
#   * Controller: MPPI with motion_model: "Ackermann". Same
#     min_turning_r constraint.
#   * AMCL: OmniMotionModel (per plan section 2.2; small Ackermann robots
#     with clean odom don't need the DifferentialMotionModel's extra
#     alpha params).
#   * BT: Spin recovery REMOVED -- the Ackermann robot cannot spin in
#     place. wait + backup(0.15) + assisted_teleop remain.
#   * Map: 200x200 blank PGM at 0.02 m/px = 4x4 m of free space.
#     Placeholder; replace with a real SLAM'd map before competition.

echo "==> Creating wro_nav2/"
mkdir -p wro_nav2/launch wro_nav2/params wro_nav2/bt wro_nav2/maps wro_nav2/rviz wro_nav2/resource

echo "==> Writing wro_nav2/package.xml"
cat > wro_nav2/package.xml <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>wro_nav2</name>
  <version>0.0.1</version>
  <description>Nav2 configuration, BT, maps, and launch for the WRO Ackermann robot.</description>
  <maintainer email="sagnikbiswas712@gmail.com">wroteam</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_lifecycle_manager</exec_depend>
  <exec_depend>nav2_common</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>nav2_amcl</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <!-- slam.launch.py resolves slam_params.yaml from WRORobot's share dir.
       This is a workspace-local package; pass rosdep --skip-keys WRORobot
       (see wro_sim/README.md Build section). -->
  <exec_depend>WRORobot</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

echo "==> Writing wro_nav2/setup.py"
cat > wro_nav2/setup.py <<'EOF'
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wro_nav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'params'),
            glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'bt'),
            glob(os.path.join('bt', '*.xml'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*.yaml')) +
            glob(os.path.join('maps', '*.pgm'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wroteam',
    maintainer_email='sagnikbiswas712@gmail.com',
    description='Nav2 config, BTs, and launch for the WRO Ackermann robot.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
EOF

echo "==> Writing wro_nav2/setup.cfg"
cat > wro_nav2/setup.cfg <<'EOF'
[develop]
script_dir=$base/lib/wro_nav2
[install]
install_scripts=$base/lib/wro_nav2
EOF

# ament_python needs an empty marker file at resource/<pkg_name> so the
# ament_index_python resource_index can find the package after install.
echo "==> Writing wro_nav2/resource/wro_nav2 (empty marker)"
: > wro_nav2/resource/wro_nav2

echo "==> Writing wro_nav2/README.md"
cat > wro_nav2/README.md <<'EOF'
# wro_nav2

Nav2 configuration, behavior tree, maps, RViz, and launch files for the WRO
Ackermann robot. Config-only -- no C++ / Python nodes, no vendored Nav2.

## What's in here

```
wro_nav2/
  params/
    nav2_params.yaml       # real-hardware params (use_sim_time: false)
    nav2_params_sim.yaml   # sim-time params (use_sim_time: true, tighter inflation)
  bt/
    wro_navigate_to_pose.xml   # Nav2's default BT with Spin removed
  maps/
    wro_field.yaml         # placeholder blank 4x4 m map
    wro_field.pgm
    README.md              # how to regenerate / SLAM a real one
  rviz/
    nav2.rviz              # map + costmaps + scan + paths + AMCL pose
  launch/
    nav2.launch.py         # full Nav2 stack (wraps nav2_bringup)
    localization.launch.py # AMCL + map_server + lifecycle_manager subset
    slam.launch.py         # slam_toolbox + Nav2 planning (no AMCL/map_server)
```

## Build

```bash
# from your ROS 2 Jazzy workspace (this repo is a flat workspace -- no src/)
colcon build --packages-select wro_nav2
source install/setup.bash
```

## Run

Real robot, pre-built map + AMCL:

```bash
ros2 launch wro_nav2 nav2.launch.py
```

With SLAM instead of AMCL (building a map online):

```bash
ros2 launch wro_nav2 slam.launch.py
```

Localization-only (no planner/controller):

```bash
ros2 launch wro_nav2 localization.launch.py
```

RViz:

```bash
rviz2 -d $(ros2 pkg prefix wro_nav2)/share/wro_nav2/rviz/nav2.rviz
```

## Design notes

- **Planner:** `nav2_smac_planner::SmacPlannerHybrid` with `motion_model: DUBIN`
  and `minimum_turning_radius: 0.30` (approx `wheelbase / tan(max_steer)` =
  `0.16 / tan(30 deg)` = 0.277). Bumped to 0.30 for margin.
- **Controller:** `nav2_mppi_controller::MPPIController` with
  `motion_model: Ackermann` and matching `AckermannConstraints.min_turning_r`.
- **Behaviors:** `wait`, `backup` (dist 0.15 m), `assisted_teleop`. `Spin` is
  **not** included -- the Ackermann robot cannot spin in place.
- **AMCL:** `DifferentialMotionModel`. Acceptable for a small Ackermann on a
  smooth WRO field per plan section 2.2; revisit if pose drift is visible.
- **Footprint:** rectangle from chassis dims (0.28 x 0.16 m), not a radius.

## TODO(measure): placeholders to replace

These need real values from the physical robot / `WRO Chassis` CAD:

- `wheelbase = 0.16 m` -> `params/nav2_params*.yaml` (used indirectly via
  `minimum_turning_radius`) and `wro_sim/models/wro_bot/wro_bot.urdf.xacro`.
- `max_steer_angle = 0.5236 rad` -> same URDF, same `minimum_turning_radius`
  calculation.
- Chassis footprint corners `[[0.14, 0.08], ...]` -> both param files,
  local and global costmap.
- Max linear velocity `vx_max: 0.5`, min `vx_min: -0.35` in MPPI + velocity
  smoother -> tune to your motor limits.
- Max angular velocity `wz_max: 1.9`, rotational accel `3.2` in
  behavior_server + velocity_smoother.
- Map (`maps/wro_field.pgm` is a 4x4 m blank). See `maps/README.md`.

## Decisions the user still needs to confirm

- **TF owner on hardware.** This task assumes `ekf_node` publishes
  `odom -> base_link` (i.e. `WRORobot/config/ekf.yaml` keeps
  `publish_tf: true`). `WRORobot/WRORobot/otos_node.py` currently also
  publishes that transform -- that's a **duplicate tf publisher** and will
  cause TF_REPEATED_DATA on the real robot. Choose one owner. See
  `wro_sim/README.md` "Known bugs from the plan not fixed here".
- **Camera in sim.** The sim exposes a plain 640x480 image on `/camera` (see
  `wro_sim`). Real hardware uses a Limelight over network tables -- the sim
  camera is **not** a Limelight equivalent, only a placeholder for future
  ROS-side vision experiments.
EOF

echo "==> Writing wro_nav2/maps/README.md"
cat > wro_nav2/maps/README.md <<'EOF'
# wro_nav2 maps

`wro_field.pgm` / `wro_field.yaml` is a placeholder: a 200x200 pixel image at
0.02 m/px (so 4x4 m) filled with the "free" value (0xFE = 254). Origin is
`(-2.0, -2.0, 0.0)` so the map is centered on `(0, 0)`.

## Regenerating a blank placeholder

If you ever need to recreate it (e.g. resize):

```bash
python3 - <<'PY'
w, h = 200, 200
with open('wro_field.pgm', 'wb') as f:
    f.write(f'P5\n{w} {h}\n255\n'.encode())
    f.write(bytes([254]) * (w * h))
PY
```

Then edit `wro_field.yaml` if you change resolution, size, or origin.

## Making a real map

Either:

1. Run the sim (`ros2 launch wro_sim sim_nav2.launch.py slam:=true`), drive
   the robot around, then save with:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f wro_field
   ```

   and drop the resulting `wro_field.pgm` + `wro_field.yaml` here.

2. Or draw a map by hand from the WRO field spec, keeping the same
   resolution/origin scheme.
EOF

echo "==> Writing wro_nav2/maps/wro_field.yaml"
cat > wro_nav2/maps/wro_field.yaml <<'EOF'
image: wro_field.pgm
mode: trinary
resolution: 0.02
origin: [-2.0, -2.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
# Placeholder map: 200x200 px at 0.02 m/px = 4x4 m of free space, all white.
# TODO(measure): replace with a real map either by loading a real WRO field
# occupancy grid, or by SLAMing the sim world and saving with:
#   ros2 run nav2_map_server map_saver_cli -f wro_field
EOF

# The PGM is binary; a bash heredoc can't safely carry the payload. Generate
# it with a Python one-liner: PGM header + 40000 bytes of 0xFE (free space).
echo "==> Writing wro_nav2/maps/wro_field.pgm (binary; generated by python3)"
python3 -c "open('wro_nav2/maps/wro_field.pgm','wb').write(b'P5\n200 200\n255\n' + b'\xfe'*(200*200))"

echo "==> Writing wro_nav2/bt/wro_navigate_to_pose.xml"
cat > wro_nav2/bt/wro_navigate_to_pose.xml <<'EOF'
<!--
  WRO NavigateToPose BT.
  Based on Nav2's default navigate_to_pose_w_replanning_and_recovery.xml,
  but Spin recovery is REMOVED (Ackermann robot cannot spin in place).
  Kept: Wait and BackUp (backup_dist 0.15) as recovery behaviors.
-->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
        <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Wait wait_duration="5.0" error_code_id="{wait_error_code}"/>
          <BackUp backup_dist="0.15" backup_speed="0.05" error_code_id="{backup_error_code}"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
EOF

echo "==> Writing wro_nav2/rviz/nav2.rviz"
cat > wro_nav2/rviz/nav2.rviz <<'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
  - Class: nav2_rviz_plugins/Navigation 2
    Name: Navigation 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Update Interval: 0
      Value: true
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Value: true
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: GlobalCostmap
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /global_costmap/costmap
      Value: true
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: LocalCostmap
      Topic:
        Depth: 1
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /local_costmap/costmap
      Value: true
    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: LaserScan
      Size (m): 0.03
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Name: GlobalPath
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /plan
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 255; 170; 0
      Enabled: true
      Name: LocalPath
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /local_plan
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/PoseWithCovariance
      Color: 255; 25; 0
      Enabled: true
      Name: AmclPose
      Shape: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /amcl_pose
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: nav2_rviz_plugins/GoalTool
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 6
      Name: Current View
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
Window Geometry:
  Height: 900
  Hide Left Dock: false
  Hide Right Dock: false
  Width: 1400
EOF

echo "==> Writing wro_nav2/launch/nav2.launch.py"
cat > wro_nav2/launch/nav2.launch.py <<'EOF'
"""Bring up the full Nav2 stack for the WRO robot.

Thin wrapper around nav2_bringup/bringup_launch.py with pre-filled args:
  - params_file, map, use_sim_time, autostart, use_composition:=False.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_wro_nav2 = get_package_share_directory('wro_nav2')

    default_params = os.path.join(pkg_wro_nav2, 'params', 'nav2_params.yaml')
    default_map = os.path.join(pkg_wro_nav2, 'maps', 'wro_field.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_composition = LaunchConfiguration('use_composition')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Autostart the nav2 stack')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 params yaml')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map yaml')
    declare_use_composition = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use ComposableNodes for nav2 (False for easier debug)')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ]),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_map,
        declare_use_composition,
        nav2_bringup,
    ])
EOF

echo "==> Writing wro_nav2/launch/localization.launch.py"
cat > wro_nav2/launch/localization.launch.py <<'EOF'
"""AMCL + map_server + lifecycle_manager only (no planner/controller).

Use this if you already have Nav2 planning bringup somewhere else, or if you
want to test localization separately.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_wro_nav2 = get_package_share_directory('wro_nav2')

    default_params = os.path.join(pkg_wro_nav2, 'params', 'nav2_params.yaml')
    default_map = os.path.join(pkg_wro_nav2, 'maps', 'wro_field.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map)

    lifecycle_nodes = ['map_server', 'amcl']

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time,
             'yaml_filename': map_yaml},
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params,
        declare_map,
        map_server,
        amcl,
        lifecycle_manager,
    ])
EOF

echo "==> Writing wro_nav2/launch/slam.launch.py"
cat > wro_nav2/launch/slam.launch.py <<'EOF'
"""SLAM Toolbox (online async) + Nav2 planning stack (no AMCL, no map_server).

Use this to build a map online. slam_params.yaml lives in the existing
WRORobot package (WRORobot/config/slam_params.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_wro_nav2 = get_package_share_directory('wro_nav2')
    pkg_wro_robot = get_package_share_directory('WRORobot')

    default_params = os.path.join(pkg_wro_nav2, 'params', 'nav2_params.yaml')
    default_slam_params = os.path.join(
        pkg_wro_robot, 'config', 'slam_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params)

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Bring up Nav2's navigation-only side (planner/controller/behavior/bt).
    # Nav2's navigation_launch.py starts everything except map_server + amcl.
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params,
        declare_slam_params,
        slam_toolbox,
        nav2_navigation,
    ])
EOF

echo "==> Writing wro_nav2/params/nav2_params.yaml"
cat > wro_nav2/params/nav2_params.yaml <<'EOF'
# Nav2 parameters for the WRO Ackermann robot (real hardware).
#
# Ackermann geometry (matches wro_sim/models/wro_bot/wro_bot.urdf.xacro):
#   wheelbase        = 0.16 m
#   max_steer_angle  = 0.5236 rad (30 deg)
#   min_turning_r    = wheelbase / tan(max_steer) approx 0.277 m -> use 0.30
#
# Footprint (rectangle from chassis dims 0.28 x 0.16):
#   [[ 0.14,  0.08], [ 0.14, -0.08], [-0.14, -0.08], [-0.14,  0.08]]
#
# TODO(measure): refresh wheelbase / max_steer_angle / footprint from real chassis.
# The values in wro_sim/models/wro_bot/wro_bot.urdf.xacro are the source of truth.

amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 8.0
    laser_min_range: 0.05
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::OmniMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.05
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: /scan

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    default_nav_to_pose_bt_xml: $(find-pkg-share wro_nav2)/bt/wro_navigate_to_pose.xml
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_assisted_teleop_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node
      - nav2_is_battery_charging_condition_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.10
      yaw_goal_tolerance: 0.20

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.0
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.0
      wz_max: 1.9
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "Ackermann"
      visualize: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      AckermannConstraints:
        min_turning_r: 0.30
      critics:
        - "ConstraintCritic"
        - "CostCritic"
        - "GoalCritic"
        - "GoalAngleCritic"
        - "PathAlignCritic"
        - "PathFollowCritic"
        - "PathAngleCritic"
        - "PreferForwardCritic"
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        critical_cost: 300.0
        consider_footprint: true
        collision_cost: 1000000.0
        near_goal_distance: 1.0
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.2
        mode: 0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.02
      footprint: "[[0.14, 0.08], [0.14, -0.08], [-0.14, -0.08], [-0.14, 0.08]]"
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 8.0
          raytrace_min_range: 0.0
          obstacle_max_range: 6.0
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.10
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.02
      footprint: "[[0.14, 0.08], [0.14, -0.08], [-0.14, -0.08], [-0.14, 0.08]]"
      track_unknown_space: true
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.10
      always_send_full_costmap: true

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: "wro_field.yaml"

map_saver:
  ros__parameters:
    use_sim_time: false
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: true

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      tolerance: 0.25
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      motion_model_for_search: "DUBIN"
      angle_quantization_bins: 72
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      minimum_turning_radius: 0.30
      reverse_penalty: 2.0
      change_penalty: 0.0
      non_straight_penalty: 1.2
      cost_penalty: 2.0
      retrospective_penalty: 0.015
      lookup_table_size: 20.0
      cache_obstacle_heuristic: false
      debug_visualizations: false
      use_quadratic_cost_penalty: false
      downsample_obstacle_heuristic: true
      allow_primitive_interpolation: false
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    use_sim_time: false
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    # NOTE: Spin recovery intentionally dropped for Ackermann.
    behavior_plugins: ["wait", "backup", "assisted_teleop"]
    wait:
      plugin: "nav2_behaviors::Wait"
    backup:
      plugin: "nav2_behaviors::BackUp"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 1.9]
    min_velocity: [-0.35, 0.0, -1.9]
    max_accel: [1.0, 0.0, 3.2]
    max_decel: [-1.5, 0.0, -3.2]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
EOF

echo "==> Writing wro_nav2/params/nav2_params_sim.yaml"
cat > wro_nav2/params/nav2_params_sim.yaml <<'EOF'
# Sim-time Nav2 parameters -- identical to nav2_params.yaml except:
#   - use_sim_time: true everywhere
#   - inflation_radius reduced to 0.07 (per plan; sim has cleaner obstacle data)
#
# Keep this in sync with nav2_params.yaml when you change controller/planner
# tuning. TODO(measure): once the real chassis dims are locked in, refresh
# footprint and min_turning_r in BOTH files.

amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 8.0
    laser_min_range: 0.05
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::OmniMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.05
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: /scan

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    default_nav_to_pose_bt_xml: $(find-pkg-share wro_nav2)/bt/wro_navigate_to_pose.xml
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_assisted_teleop_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node
      - nav2_is_battery_charging_condition_bt_node

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.10
      yaw_goal_tolerance: 0.20

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.0
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.0
      wz_max: 1.9
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "Ackermann"
      visualize: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      AckermannConstraints:
        min_turning_r: 0.30
      critics:
        - "ConstraintCritic"
        - "CostCritic"
        - "GoalCritic"
        - "GoalAngleCritic"
        - "PathAlignCritic"
        - "PathFollowCritic"
        - "PathAngleCritic"
        - "PreferForwardCritic"
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        critical_cost: 300.0
        consider_footprint: true
        collision_cost: 1000000.0
        near_goal_distance: 1.0
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.2
        mode: 0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.02
      footprint: "[[0.14, 0.08], [0.14, -0.08], [-0.14, -0.08], [-0.14, 0.08]]"
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 8.0
          raytrace_min_range: 0.0
          obstacle_max_range: 6.0
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.07
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.02
      footprint: "[[0.14, 0.08], [0.14, -0.08], [-0.14, -0.08], [-0.14, 0.08]]"
      track_unknown_space: true
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.07
      always_send_full_costmap: true

map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: "wro_field.yaml"

map_saver:
  ros__parameters:
    use_sim_time: true
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: true

planner_server:
  ros__parameters:
    use_sim_time: true
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      tolerance: 0.25
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      motion_model_for_search: "DUBIN"
      angle_quantization_bins: 72
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      minimum_turning_radius: 0.30
      reverse_penalty: 2.0
      change_penalty: 0.0
      non_straight_penalty: 1.2
      cost_penalty: 2.0
      retrospective_penalty: 0.015
      lookup_table_size: 20.0
      cache_obstacle_heuristic: false
      debug_visualizations: false
      use_quadratic_cost_penalty: false
      downsample_obstacle_heuristic: true
      allow_primitive_interpolation: false
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10

smoother_server:
  ros__parameters:
    use_sim_time: true
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    use_sim_time: true
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["wait", "backup", "assisted_teleop"]
    wait:
      plugin: "nav2_behaviors::Wait"
    backup:
      plugin: "nav2_behaviors::BackUp"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

waypoint_follower:
  ros__parameters:
    use_sim_time: true
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200

velocity_smoother:
  ros__parameters:
    use_sim_time: true
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 1.9]
    min_velocity: [-0.35, 0.0, -1.9]
    max_accel: [1.0, 0.0, 3.2]
    max_decel: [-1.5, 0.0, -3.2]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
EOF

# =============================================================================
# 8. Create wro_sim/ package
# =============================================================================
# Why: Gazebo Harmonic simulation + ros_gz bridge, packaged as ament_cmake
# (so we can use ament_environment_hooks to prepend GZ_SIM_RESOURCE_PATH).
# The sim publishes the exact same ROS topics the real robot publishes --
# see wro_sim/config/ros_gz_bridge.yaml -- so wro_nav2 and wro_behavior
# run identically against both.
#
# Note: worlds/wro_field.sdf and worlds/wro_field.sdf.xacro are copies of
# ../wro_map_nav2_sim.sdf and ../wro_map_nav2_sim.sdf.xacro at the repo
# root. Copied (not moved) so the root files stay in place as reference.

echo "==> Creating wro_sim/"
mkdir -p wro_sim/worlds wro_sim/models/wro_bot wro_sim/config wro_sim/hook wro_sim/launch wro_sim/rviz

echo "==> Copying world SDFs into wro_sim/worlds/"
cp wro_map_nav2_sim.sdf wro_sim/worlds/wro_field.sdf
cp wro_map_nav2_sim.sdf.xacro wro_sim/worlds/wro_field.sdf.xacro

echo "==> Writing wro_sim/package.xml"
cat > wro_sim/package.xml <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>wro_sim</name>
  <version>0.0.1</version>
  <description>Gazebo Harmonic simulation of the WRO Ackermann robot + Nav2 bringup.</description>
  <maintainer email="sagnikbiswas712@gmail.com">wroteam</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ros_gz_image</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>sdformat_urdf</exec_depend>
  <exec_depend>WRORobot</exec_depend>
  <exec_depend>wro_nav2</exec_depend>
  <exec_depend>wro_behavior</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

echo "==> Writing wro_sim/CMakeLists.txt"
cat > wro_sim/CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.8)
project(wro_sim)

find_package(ament_cmake REQUIRED)

# Install runtime data.
install(DIRECTORY
    worlds
    models
    launch
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# Environment hook: prepend our worlds/ and models/ to GZ_SIM_RESOURCE_PATH so
# gz-sim can find the robot model and world files at runtime.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hook/wro_sim.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
EOF

echo "==> Writing wro_sim/README.md"
cat > wro_sim/README.md <<'EOF'
# wro_sim

Gazebo Harmonic simulation of the WRO Ackermann robot. Bridges the sim to
the same ROS 2 topic/frame contract that the real robot uses, so anything
built against `wro_nav2` and `wro_behavior` runs unchanged on both.

**Do NOT confuse this with Gazebo Classic or Ignition Fortress.** This
package uses `gz-sim-*` plugins and the Jazzy `ros_gz_*` bridges.

## Contents

```
wro_sim/
  worlds/
    wro_field.sdf            # copy of ../wro_map_nav2_sim.sdf
    wro_field.sdf.xacro
  models/wro_bot/
    model.config             # (URDF lives in WRORobot/urdf/, shared with real robot)
  config/
    ros_gz_bridge.yaml       # gz <-> ROS 2 topic map
  hook/
    wro_sim.dsv.in           # prepends GZ_SIM_RESOURCE_PATH
  launch/
    sim.launch.py            # Gazebo + rsp + spawn + bridges (+ optional rviz)
    sim_stack.launch.py      # top-level: sim + Nav2 + behavior
    spawn_robot.launch.py    # standalone spawn
    bridge.launch.py         # standalone bridge
    rviz.launch.py           # standalone rviz
  rviz/sim.rviz
```

## Build

```bash
# From the flat workspace root (this repo).
# rosdep can't resolve workspace-local packages; pass --skip-keys so it
# doesn't error on wro_behavior/wro_nav2/WRORobot. Colcon still respects
# these deps for build ordering.
rosdep install --from-paths . --ignore-src -r -y \
  --skip-keys "WRORobot wro_nav2 wro_behavior"

colcon build --packages-select WRORobot wro_nav2 wro_behavior wro_sim
source install/setup.bash
```

## Run

Sim stack (Gazebo + Nav2 + behavior):

```bash
ros2 launch wro_sim sim_stack.launch.py rviz:=true
# Drop a 2D Nav Goal in RViz, or:
ros2 launch wro_sim sim_stack.launch.py rviz:=true run_goto:=true goal_x:=1.0
```

Just the sim (drive with teleop):

```bash
ros2 launch wro_sim sim.launch.py rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Sim + SLAM (build a map online):

```bash
ros2 launch wro_sim sim_stack.launch.py slam:=true rviz:=true
# When happy:
ros2 run nav2_map_server map_saver_cli -f wro_field
mv wro_field.pgm wro_field.yaml $(ros2 pkg prefix wro_nav2)/share/wro_nav2/maps/
```

## Topic bridge

Every ROS topic below matches the real hardware's topic name -- see
`INTERFACE.md` at repo root for the full contract.

| ROS topic         | direction | Gazebo topic                 | type                       |
|-------------------|-----------|------------------------------|----------------------------|
| `/clock`          | gz -> ros | `/clock`                     | rosgraph_msgs/Clock        |
| `/scan`           | gz -> ros | `/lidar`                     | sensor_msgs/LaserScan      |
| `/imu/data_raw`   | gz -> ros | `/imu`                       | sensor_msgs/Imu            |
| `/odom`           | gz -> ros | `/model/wro_bot/odometry`    | nav_msgs/Odometry          |
| `/tf`             | gz -> ros | `/model/wro_bot/tf`          | tf2_msgs/TFMessage         |
| `/joint_states`   | gz -> ros | `/model/wro_bot/joint_state` | sensor_msgs/JointState     |
| `/camera_info`    | gz -> ros | `/camera_info`               | sensor_msgs/CameraInfo     |
| `/camera`         | gz -> ros | `/camera`                    | sensor_msgs/Image (via image_bridge) |
| `/cmd_vel`        | ros -> gz | `/model/wro_bot/cmd_vel`     | geometry_msgs/Twist        |

## TODO(measure): placeholders

The URDF (`WRORobot/urdf/wro_bot.urdf.xacro`) is the single source of truth
for physical dimensions on both sim and real robot. Every property in
that file is a placeholder; replace against the physical car or CAD:

- `wheelbase = 0.16`
- `track_width = 0.14`
- `wheel_radius = 0.033`
- `wheel_thickness = 0.025`
- `max_steer_angle = 0.5236` (30 deg)
- Chassis and sensor-mount offsets
- Link masses / inertias

When you update any of these, also update `wro_nav2/params/nav2_params*.yaml`:
- `minimum_turning_radius` on both `SmacPlannerHybrid` and MPPI's
  `AckermannConstraints.min_turning_r`
- Footprint polygon in both costmaps

Sim spawn pose defaults live in `launch/sim.launch.py` (`x=0`, `y=-1.3`,
`yaw=1.5708`). Adjust to your WRO field start box.

## Real-hardware decisions already made

- **`odom -> base_link` on hardware is published by `otos_node` only.**
  EKF and Madgwick were removed from `WRORobot/launch/hardware.launch.py`
  on 2026-08-30. See that file's header comment for how to re-enable.
- **Sim `/camera`** is a plain 640x480 image, NOT a Limelight replacement.
  Real hardware uses a Limelight over NetworkTables; sim camera is for
  future ROS-side vision development.
EOF

echo "==> Writing wro_sim/hook/wro_sim.dsv.in"
cat > wro_sim/hook/wro_sim.dsv.in <<'EOF'
prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share/wro_sim/models
prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share/wro_sim/worlds
EOF

echo "==> Writing wro_sim/models/wro_bot/model.config"
cat > wro_sim/models/wro_bot/model.config <<'EOF'
<?xml version="1.0"?>
<model>
  <name>wro_bot</name>
  <version>1.0</version>
  <sdf version="1.9">wro_bot.urdf.xacro</sdf>
  <author>
    <name>WRO Team</name>
    <email>sagnikbiswas712@gmail.com</email>
  </author>
  <description>
    Placeholder 4-wheel Ackermann model of the WRO robot. All dimensions are
    TODO(measure) placeholders -- see wro_bot.urdf.xacro for the real values
    to swap in.
  </description>
</model>
EOF

echo "==> Writing wro_sim/config/ros_gz_bridge.yaml"
cat > wro_sim/config/ros_gz_bridge.yaml <<'EOF'
# ros_gz_bridge configuration for the wro_bot in sim.
#
# ROS-side topic names match what the real hardware stack uses:
#   /scan, /imu/data_raw, /odom, /cmd_vel, /tf, /clock, /joint_states,
#   /camera_info.
#
# The raw /camera image is bridged separately with ros_gz_image (see
# launch/gz_sim.launch.py) -- the ros_gz_bridge only handles /camera_info here.

- ros_topic_name: "clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "scan"
  gz_topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "imu/data_raw"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "odom"
  gz_topic_name: "/model/wro_bot/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "tf"
  gz_topic_name: "/model/wro_bot/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "joint_states"
  gz_topic_name: "/model/wro_bot/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "camera_info"
  gz_topic_name: "/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS

- ros_topic_name: "cmd_vel"
  gz_topic_name: "/model/wro_bot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
EOF

echo "==> Writing wro_sim/launch/sim.launch.py"
cat > wro_sim/launch/sim.launch.py <<'EOF'
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
    robot consume the same file -- see INTERFACE.md.
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
    declare_z = DeclareLaunchArgument('z', default_value='0.073')
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
            'gz_args': [world, ' -r -v 3'],
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
    # passed via the `config_file` ROS parameter (canonical Jazzy form --
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
EOF

echo "==> Writing wro_sim/launch/spawn_robot.launch.py"
cat > wro_sim/launch/spawn_robot.launch.py <<'EOF'
"""Standalone spawn of wro_bot into an already-running Gazebo instance.

Requires robot_description to be published on /robot_description (e.g. by
running robot_state_publisher separately, or by launching gz_sim.launch.py
which starts it for you).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw_pose = LaunchConfiguration('yaw')
    name = LaunchConfiguration('name')

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-1.3'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        DeclareLaunchArgument('name', default_value='wro_bot'),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_wro_bot',
            arguments=[
                '-name', name,
                '-topic', 'robot_description',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-Y', yaw_pose,
            ],
            output='screen',
        ),
    ])
EOF

echo "==> Writing wro_sim/launch/bridge.launch.py"
cat > wro_sim/launch/bridge.launch.py <<'EOF'
"""Standalone ros_gz_bridge + image_bridge.

Use with an already-running Gazebo + robot instance.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_wro_sim = get_package_share_directory('wro_sim')
    default_bridge = os.path.join(pkg_wro_sim, 'config', 'ros_gz_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=default_bridge),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{
                'config_file': config_file,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='camera_image_bridge',
            arguments=['/camera'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
EOF

echo "==> Writing wro_sim/launch/rviz.launch.py"
cat > wro_sim/launch/rviz.launch.py <<'EOF'
"""Standalone RViz with the sim.rviz config."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_rviz = os.path.join(
        get_package_share_directory('wro_sim'), 'rviz', 'sim.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
EOF

echo "==> Writing wro_sim/launch/sim_stack.launch.py"
cat > wro_sim/launch/sim_stack.launch.py <<'EOF'
"""Top-level sim stack: Gazebo + Nav2 + behavior layer.

This is the sim engineer's default entry point. It composes three layers:
  1. wro_sim/sim.launch.py       -- Gazebo + URDF + bridge
  2. wro_nav2/nav2.launch.py     -- Nav2 stack (or slam.launch.py if slam:=true)
  3. wro_behavior/behavior.launch.py -- mission nodes (off by default)

The real-robot counterpart is WRORobot/launch/robot_stack.launch.py -- same
navigation + behavior layers, hardware.launch.py in place of sim.launch.py.

Args:
  slam:=true      -> use slam_toolbox instead of AMCL/map_server
  rviz:=true      -> also open RViz
  run_goto:=true  -> fire the wro_behavior goto_pose hello-world on launch
  x, y, yaw       -> spawn pose (defaults: 0, -1.3, 1.5708)

Note: wro_nav2/nav2.launch.py wraps nav2_bringup/bringup_launch.py, which
already starts AMCL + map_server. wro_nav2/localization.launch.py is a
subset of that for standalone use -- do not include it here or AMCL and
map_server will double-launch.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_wro_nav2 = get_package_share_directory('wro_nav2')

    sim_params = os.path.join(pkg_wro_nav2, 'params', 'nav2_params_sim.yaml')
    sim_map = os.path.join(pkg_wro_nav2, 'maps', 'wro_field.yaml')

    slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_goto = LaunchConfiguration('run_goto')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    yaw_pose = LaunchConfiguration('yaw')

    declare_slam = DeclareLaunchArgument('slam', default_value='false')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_run_goto = DeclareLaunchArgument(
        'run_goto', default_value='false')
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='-1.3')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='1.5708')

    # 1. Gazebo + robot_state_publisher + spawn + bridges (+ optional RViz).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_sim'), 'launch', 'sim.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'x': x_pose,
            'y': y_pose,
            'yaw': yaw_pose,
        }.items(),
    )

    # 2a. Full Nav2 bringup with AMCL (default).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_nav2'),
                'launch', 'nav2.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': sim_params,
            'map': sim_map,
            'use_composition': 'False',
        }.items(),
        condition=UnlessCondition(slam),
    )

    # 2b. SLAM Toolbox + Nav2 planning stack (with slam:=true).
    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_nav2'),
                'launch', 'slam.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': sim_params,
        }.items(),
        condition=IfCondition(slam),
    )

    # 3. Behavior layer (mission-level nodes).
    behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wro_behavior'),
                'launch', 'behavior.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'run_goto': run_goto,
        }.items(),
    )

    return LaunchDescription([
        declare_slam, declare_rviz, declare_use_sim_time, declare_run_goto,
        declare_x, declare_y, declare_yaw,
        sim,
        nav2,
        slam_bringup,
        behavior,
    ])
EOF

echo "==> Writing wro_sim/rviz/sim.rviz"
cat > wro_sim/rviz/sim.rviz <<'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Description Source: Topic
      Description Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Visual Enabled: true
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Update Interval: 0
      Value: true
    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: LaserScan
      Size (m): 0.03
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Camera
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /camera
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/Odometry
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 255; 127
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.3
          Color: 204; 51; 204
          Scale: 1
          Value: true
      Enabled: true
      Keep: 100
      Name: Odometry
      Position Tolerance: 0.1
      Angle Tolerance: 0.1
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /odom
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Interact
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 4
      Name: Current View
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
Window Geometry:
  Height: 900
  Hide Left Dock: false
  Hide Right Dock: false
  Width: 1400
EOF

# =============================================================================
# 9. Create wro_behavior/ package
# =============================================================================
# Why: mission logic (state machines, waypoint sequences, vision reactions)
# has to live in its own layer with strict "no direct imports from hardware
# or sim" discipline. If behavior imports from WRORobot the same code
# can't run in sim; if it imports from wro_sim the same code can't run on
# the real robot. Everything it needs from the lower layers is pinned in
# INTERFACE.md.
#
# The one node here right now is goto_pose -- a hello-world NavigateToPose
# client. It's a template, not real mission logic.

echo "==> Creating wro_behavior/"
mkdir -p wro_behavior/launch wro_behavior/resource wro_behavior/wro_behavior

echo "==> Writing wro_behavior/package.xml"
cat > wro_behavior/package.xml <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>wro_behavior</name>
  <version>0.0.1</version>
  <description>
    High-level behavior layer for the WRO robot: mission state machines,
    NavigateToPose action clients, custom BT nodes, and per-round waypoint
    sequences. Consumes the Nav2 action/topic contract only -- never touches
    hardware drivers or sim internals directly, so it runs unchanged against
    both wro_sim and the real robot.
  </description>
  <maintainer email="sagnikbiswas712@gmail.com">wroteam</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

echo "==> Writing wro_behavior/setup.py"
cat > wro_behavior/setup.py <<'EOF'
from glob import glob

from setuptools import find_packages, setup

package_name = 'wro_behavior'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wroteam',
    maintainer_email='sagnikbiswas712@gmail.com',
    description='High-level behavior for the WRO robot (sim + real).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto_pose = wro_behavior.goto_pose:main',
        ],
    },
)
EOF

echo "==> Writing wro_behavior/setup.cfg"
cat > wro_behavior/setup.cfg <<'EOF'
[develop]
script_dir=$base/lib/wro_behavior
[install]
install_scripts=$base/lib/wro_behavior
EOF

echo "==> Writing wro_behavior/README.md"
cat > wro_behavior/README.md <<'EOF'
# wro_behavior

High-level behavior for the WRO robot. This is where mission logic, state
machines, custom BT nodes, waypoint sequences, and vision reactions live.

## Contract

This package consumes the interface pinned in `../INTERFACE.md`. It never
imports from `WRORobot`, `wro_sim`, or any hardware/driver package. If you
find yourself wanting to depend on one, the abstraction is wrong and the
answer is a new topic/action/service in the interface, not a direct import.

## Nodes

- `goto_pose` -- hello-world NavigateToPose action client. Sends one goal in
  the `map` frame, prints feedback and result. Use it as a template for
  new mission nodes; not real mission logic.

## Run

Against the sim:
```bash
ros2 launch wro_sim sim_stack.launch.py rviz:=true
ros2 launch wro_behavior behavior.launch.py \
  run_goto:=true goal_x:=1.0 goal_y:=0.5 use_sim_time:=true
```

Against the real robot:
```bash
ros2 launch WRORobot robot_stack.launch.py
ros2 launch wro_behavior behavior.launch.py run_goto:=true goal_x:=1.0
```

Or fire a goal by hand without this package running:
```bash
ros2 run wro_behavior goto_pose --ros-args -p x:=1.0 -p y:=0.5
```
EOF

# ament_index marker (empty).
echo "==> Writing wro_behavior/resource/wro_behavior (empty marker)"
: > wro_behavior/resource/wro_behavior

# Python package marker (empty).
echo "==> Writing wro_behavior/wro_behavior/__init__.py (empty)"
: > wro_behavior/wro_behavior/__init__.py

echo "==> Writing wro_behavior/wro_behavior/goto_pose.py"
cat > wro_behavior/wro_behavior/goto_pose.py <<'EOF'
#!/usr/bin/env python3
"""Minimal NavigateToPose action-client node.

Sends a single goal to Nav2's /navigate_to_pose action and prints feedback +
result. This is the 'hello world' for the behavior layer -- it proves the
sim engineer's stack talks to Nav2 correctly. Real mission logic (state
machines, per-round sequences, vision reactions) will replace this node.

Run:
  ros2 run wro_behavior goto_pose --ros-args \
    -p x:=1.0 -p y:=0.5 -p yaw:=0.0
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GotoPose(Node):
    def __init__(self):
        super().__init__('wro_goto_pose')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send(self):
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value
        frame_id = self.get_parameter('frame_id').value

        self.get_logger().info(
            f'Waiting for /navigate_to_pose action server...')
        self._client.wait_for_server()

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f'Sending goal: frame={frame_id} x={x} y={y} yaw={yaw}')
        future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'distance remaining: {fb.distance_remaining:.2f} m, '
            f'ETA: {fb.estimated_time_remaining.sec} s')

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted; waiting for result...')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 reached the goal.')
        else:
            self.get_logger().warn(f'Nav2 terminated with status {status}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = GotoPose()
    node.send()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
EOF

echo "==> Writing wro_behavior/launch/behavior.launch.py"
cat > wro_behavior/launch/behavior.launch.py <<'EOF'
"""Behavior-layer launch.

Starts the mission-level nodes that consume Nav2's action/topic contract.
This launch never depends on hardware drivers or sim internals -- pair it
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
EOF

# =============================================================================
# 10. Create INTERFACE.md
# =============================================================================
# Why: this file pins the contract between the three layers. If a hardware
# engineer changes what topics get published, and the behavior engineer's
# code silently breaks two weeks later at a competition, the failure was
# the interface not being written down -- not the change itself.
# The rule: any change to what's in here needs a review from an engineer on
# each side, before the code change.

echo "==> Writing INTERFACE.md"
cat > INTERFACE.md <<'EOF'
# WRO26 Robot Interface Contract

This document pins the interface between the three layers of the stack:

- **Hardware** (`WRORobot/`, drivers) OR **Sim** (`wro_sim/`) -- one of the two
  publishes the sensor topics and consumes `/cmd_vel`.
- **Navigation** (`wro_nav2/`) -- plans, controls, localizes.
- **Behavior** (`wro_behavior/`) -- mission logic. Never touches hardware or
  sim internals.

Both sides of every arrow below MUST hold in both sim and on real hardware.
Any change to this file requires review from an engineer on each side.

---

## Frames

```
map -> odom -> base_link -> { base_laser, imu_link, camera_link, *_wheel_* }
                                                       |
                                                       v
                                              camera_optical_frame
```

- `map` -- global fixed frame. Published by AMCL (`wro_nav2/nav2.launch.py`)
  or by `slam_toolbox` (`wro_nav2/slam.launch.py`). Never published by
  anything else.
- `odom` -- smooth-drift odom parent. Publisher differs by side:
  - **Sim**: `gz-sim-ackermann-steering-system` plugin, bridged as
    `/model/wro_bot/tf` -> `/tf`.
  - **Real robot**: `otos_node` (see WRORobot/WRORobot/otos_node.py).
    EKF is intentionally disabled -- see WRORobot/launch/robot.launch.py
    header comment for how to re-enable if OTOS becomes unreliable.
- `base_link` -- robot body root. Owned by `robot_state_publisher`
  processing `WRORobot/urdf/wro_bot.urdf.xacro`.
- `base_laser`, `imu_link`, `camera_link`, `camera_optical_frame`, wheel
  frames -- all published by `robot_state_publisher` from the same URDF.

Only one publisher per TF edge. Any duplication is a bug.

---

## Topics behavior consumes

| Topic | Type | Publisher | Rate |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Sim: bridge from `/lidar`. Real: `ldlidar_ros2`. | ~10 Hz |
| `/odom` | `nav_msgs/Odometry` | Sim: bridge. Real: `otos_node`. | >=30 Hz |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | as above per edge | -- |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Nav2 AMCL | ~2 Hz |
| `/camera/image_raw`, `/camera_info` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | Sim: bridge. Real: TBD (not wired yet -- Limelight-shaped). | ~30 Hz |
| `/joint_states` | `sensor_msgs/JointState` | Sim: bridge. Real: TBD. | ~30 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` | Sim: bridge. Real: `lsm9ds1_imu_driver`. Not currently fused; available for logging. | ~100 Hz |

---

## Topics behavior publishes

| Topic | Type | Consumer |
|---|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Nav2 (RViz-style single-goal path; testing only) |

---

## Actions behavior calls

| Action | Type | Server |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | `bt_navigator` in Nav2 |
| `/follow_waypoints` | `nav2_msgs/FollowWaypoints` | `waypoint_follower` in Nav2 |

---

## Topics behavior/nav publishes -> hardware or sim consumes

| Topic | Type | Consumer | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Sim: bridge -> AckermannSteering. Real: `Motor` node (ESC) + `Servo` node (steering servo). | Nav2 publishes via `velocity_smoother`; behavior can also publish for teleop-style tests. |

---

## Ackermann kinematics assumptions

`/cmd_vel.linear.x` is the desired forward velocity in m/s.
`/cmd_vel.angular.z` is the desired yaw rate in rad/s.

The controller and drive plugin/driver must convert `(vx, wz)` to
`(wheel_speed, steering_angle)` using the wheelbase and steering limits
declared in `WRORobot/urdf/wro_bot.urdf.xacro`. If those values change,
both sides must re-read the URDF at launch.

There is no support for `/cmd_vel.linear.y` -- the robot cannot strafe.
Nav2 params set `vy_max: 0.0` and `motion_model: Ackermann`.

---

## Guarantees each side must uphold

**Hardware side (`WRORobot/hardware.launch.py`):**
- `/odom` at >= 30 Hz; drift < 5 cm over a 30 s straight-line run.
- `/scan` at >= 10 Hz; frame_id `base_laser`.
- `/imu/data_raw` at >= 100 Hz; frame_id `imu_link`.
- `/cmd_vel` -> motion within 50 ms.
- No writes to `/tf` except `odom -> base_link` (from `otos_node`) and
  robot_state_publisher's `base_link -> *`.

**Sim side (`wro_sim/sim.launch.py`):**
- Same rates and frame_ids as above, matched by `ros_gz_bridge`.
- Physics stable at 1x real time on a target dev laptop.

**Nav side (`wro_nav2/nav2.launch.py`):**
- Consumes only the topics/frames listed above.
- Publishes `/cmd_vel` (via smoother) and `map -> odom`.

**Behavior side (`wro_behavior/behavior.launch.py`):**
- Consumes only the topics/actions listed above.
- Never imports from hardware or sim packages.

---

## When something on this list changes

1. Open a PR that edits **only** `INTERFACE.md` first.
2. Get review from one engineer per side.
3. Only after that PR merges, implement the change in the affected packages.

This keeps the two engineers unblocked -- if one changes the interface
unilaterally, the other's code will silently break at competition, not at
merge time.
EOF

# =============================================================================
# 11. Sanity checks
# =============================================================================
# Why: verify the file writes actually produced parseable content, so the
# user finds out about a broken YAML or a broken xacro BEFORE they run
# colcon build (which surfaces the same errors much later and much noisier).

echo ""
echo "==> Sanity check: all expected files exist"
MISSING=0
for f in \
    INTERFACE.md \
    WRORobot/setup.py \
    WRORobot/package.xml \
    WRORobot/urdf/wro_bot.urdf.xacro \
    WRORobot/launch/hardware.launch.py \
    WRORobot/launch/robot_stack.launch.py \
    wro_nav2/package.xml \
    wro_nav2/setup.py \
    wro_nav2/setup.cfg \
    wro_nav2/resource/wro_nav2 \
    wro_nav2/README.md \
    wro_nav2/maps/README.md \
    wro_nav2/maps/wro_field.yaml \
    wro_nav2/maps/wro_field.pgm \
    wro_nav2/params/nav2_params.yaml \
    wro_nav2/params/nav2_params_sim.yaml \
    wro_nav2/bt/wro_navigate_to_pose.xml \
    wro_nav2/rviz/nav2.rviz \
    wro_nav2/launch/nav2.launch.py \
    wro_nav2/launch/localization.launch.py \
    wro_nav2/launch/slam.launch.py \
    wro_sim/package.xml \
    wro_sim/CMakeLists.txt \
    wro_sim/README.md \
    wro_sim/hook/wro_sim.dsv.in \
    wro_sim/models/wro_bot/model.config \
    wro_sim/config/ros_gz_bridge.yaml \
    wro_sim/worlds/wro_field.sdf \
    wro_sim/worlds/wro_field.sdf.xacro \
    wro_sim/launch/sim.launch.py \
    wro_sim/launch/spawn_robot.launch.py \
    wro_sim/launch/bridge.launch.py \
    wro_sim/launch/rviz.launch.py \
    wro_sim/launch/sim_stack.launch.py \
    wro_sim/rviz/sim.rviz \
    wro_behavior/package.xml \
    wro_behavior/setup.py \
    wro_behavior/setup.cfg \
    wro_behavior/resource/wro_behavior \
    wro_behavior/README.md \
    wro_behavior/wro_behavior/__init__.py \
    wro_behavior/wro_behavior/goto_pose.py \
    wro_behavior/launch/behavior.launch.py \
    ; do
  if [[ ! -f "$f" ]]; then
    echo "MISSING: $f" >&2
    MISSING=$((MISSING+1))
  fi
done
if [[ $MISSING -gt 0 ]]; then
  echo "ERROR: $MISSING files missing." >&2
  exit 1
fi
echo "    all $(ls | wc -l)-plus expected files present."

# The robot.launch.py file must NOT exist anymore.
if [[ -f WRORobot/launch/robot.launch.py ]]; then
  echo "ERROR: WRORobot/launch/robot.launch.py should have been removed." >&2
  exit 1
fi
echo "    WRORobot/launch/robot.launch.py correctly removed."

echo ""
echo "==> Sanity check: xacro parses"
# xacro isn't guaranteed to be on PATH before rosdep runs. If it is, use it;
# if not, just verify the file is well-formed XML.
if python3 -c "import xacro" 2>/dev/null; then
  python3 -c "import xacro; xacro.process_file('WRORobot/urdf/wro_bot.urdf.xacro')" >/dev/null \
    && echo "    xacro parse OK." \
    || { echo "ERROR: xacro parse failed." >&2; exit 1; }
else
  python3 -c "import xml.etree.ElementTree as ET; ET.parse('WRORobot/urdf/wro_bot.urdf.xacro')" >/dev/null \
    && echo "    xml parse OK (xacro module not importable; source ROS setup.bash to enable full xacro check)." \
    || { echo "ERROR: URDF xacro XML parse failed." >&2; exit 1; }
fi

echo ""
echo "==> Sanity check: YAML syntax on all yamls"
for y in \
    wro_nav2/params/nav2_params.yaml \
    wro_nav2/params/nav2_params_sim.yaml \
    wro_nav2/maps/wro_field.yaml \
    wro_sim/config/ros_gz_bridge.yaml \
    ; do
  python3 -c "import yaml,sys; yaml.safe_load(open('$y'))" \
    && echo "    OK: $y" \
    || { echo "ERROR: YAML parse failed on $y" >&2; exit 1; }
done

echo ""
echo "==> git status (short) -- review before committing:"
git status --short | head -60

echo ""
echo "==> apply_changes.sh finished successfully."
echo "    Nothing has been committed. See the commented block at the bottom"
echo "    of this script for suggested commit commands."

# =============================================================================
# 12. Commit guidance (commented -- run manually if you want these commits)
# =============================================================================
# The user asked to commit themselves. These are suggested three-commit splits
# that keep each layer isolated in git history.
#
# --- suggested commit 1: WRORobot changes (drivers + URDF) ---
# git add WRORobot/setup.py WRORobot/package.xml \
#         WRORobot/urdf/wro_bot.urdf.xacro \
#         WRORobot/launch/hardware.launch.py \
#         WRORobot/launch/robot_stack.launch.py
# git rm WRORobot/launch/robot.launch.py
# git commit -m "WRORobot: split hardware.launch.py + robot_stack.launch.py, add shared URDF"
#
# --- suggested commit 2: new packages (wro_nav2 + wro_sim + wro_behavior) ---
# git add wro_nav2/ wro_sim/ wro_behavior/
# git commit -m "Add wro_nav2, wro_sim, wro_behavior packages (Ackermann-aware Nav2 + Gazebo Harmonic sim)"
#
# --- suggested commit 3: interface contract ---
# git add INTERFACE.md
# git commit -m "Add INTERFACE.md contract between hardware/sim, nav2, and behavior layers"
