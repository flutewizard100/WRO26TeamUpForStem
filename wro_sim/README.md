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

# slam_toolbox is a LIFECYCLE node and slam.launch.py does NOT wire in a
# lifecycle_manager yet. Activate it by hand from another terminal:
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

# Drive with teleop until the RViz Map display looks acceptable.
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save. The transient_local param is REQUIRED — slam_toolbox publishes
# /map with TRANSIENT_LOCAL durability and map_saver_cli defaults to
# VOLATILE, which silently misses every message and prints
# "Failed to spin map subscription" after 2 s.
ros2 run nav2_map_server map_saver_cli -f wro_field \
    --ros-args -p map_subscribe_transient_local:=true
mv wro_field.pgm wro_field.yaml $(ros2 pkg prefix wro_nav2)/share/wro_nav2/maps/
```

The saved map's origin is anchored at the robot's pose when slam_toolbox
was activated, NOT Gazebo's world origin. When you later reload the map
with Nav2 (`slam:=false`), use RViz's 2D Pose Estimate to set the
initial pose to `(0, 0)` yaw `0` in the map frame — that places the
robot at Gazebo world `(0, -1.3)` yaw `1.5708`.

## Topic bridge

Every ROS topic below matches the real hardware's topic name — see
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
`z=0.103`, `yaw=1.5708`). Adjust to your WRO field start box.

**Do not lower the default `z`.** `z = wheel_radius + chassis_height/2`
(0.033 + 0.070 = 0.103) is the resting height with wheels on the ground.
Spawning below that (e.g. the old `z=0.073`) buries the wheels ~3 cm
underground; physics still runs and joint commands still spin the wheels,
but the chassis stays jammed and never translates. Symptom: `/odom`
integrates a fake forward velocity while `gz model -m wro_bot -p` shows
the pose unchanged.

## Real-hardware decisions already made

- **`odom -> base_link` on hardware is published by `otos_node` only.**
  EKF and Madgwick were removed from `WRORobot/launch/hardware.launch.py`
  on 2026-08-30. See that file's header comment for how to re-enable.
- **Sim `/camera`** is a plain 640x480 image, NOT a Limelight replacement.
  Real hardware uses a Limelight over NetworkTables; sim camera is for
  future ROS-side vision development.

## Gotchas encountered during first bring-up

Kept here so the next engineer doesn't re-diagnose the same problems:

- **Launch it from a plain terminal, not the VSCode Snap terminal.**
  VSCode-as-Snap leaks Snap env into subprocesses, and `gz sim` picks up
  `/snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0` — an
  ABI-incompatible libpthread — and dies with
  `undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE`. The
  GUI crashes first, then the server gets SIGKILL'd. Use GNOME Terminal
  / kitty / anything non-Snap.
- **Ackermann plugin driven joints must be the rear wheels.**
  `<left_joint>` / `<right_joint>` in
  `gz-sim-ackermann-steering-system` are the joints velocity is
  applied to. Setting them to the front wheels (which are also the
  steering knuckles) results in front wheels spinning kinematically
  with no traction and the car sits still. Fixed in
  `WRORobot/urdf/wro_bot.urdf.xacro`.
- **slam_toolbox does not auto-activate.** See the SLAM run block
  above. Long-term fix: add a `nav2_lifecycle_manager` in
  `wro_nav2/launch/slam.launch.py` with `autostart: true` and
  `node_names: ['slam_toolbox']`.
- **`map_saver_cli` needs `-p map_subscribe_transient_local:=true`.**
  Without it, saves fail with "Failed to spin map subscription".
