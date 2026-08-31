# WRO26 Robot Interface Contract

This document pins the interface between the three layers of the stack:

- **Hardware** (`WRORobot/`, drivers) OR **Sim** (`wro_sim/`) — one of the two
  publishes the sensor topics and consumes `/cmd_vel`.
- **Navigation** (`wro_nav2/`) — plans, controls, localizes.
- **Behavior** (`wro_behavior/`) — mission logic. Never touches hardware or
  sim internals.

Both sides of every arrow below MUST hold in both sim and on real hardware.
Any change to this file requires review from an engineer on each side.

---

## Frames

```
map → odom → base_link → { base_laser, imu_link, camera_link, *_wheel_* }
                                                       ↓
                                              camera_optical_frame
```

- `map` — global fixed frame. Published by AMCL (`wro_nav2/nav2.launch.py`)
  or by `slam_toolbox` (`wro_nav2/slam.launch.py`). Never published by
  anything else.
- `odom` — smooth-drift odom parent. Publisher differs by side:
  - **Sim**: `gz-sim-ackermann-steering-system` plugin, bridged as
    `/model/wro_bot/tf` → `/tf`.
  - **Real robot**: `otos_node` (see WRORobot/WRORobot/otos_node.py).
    EKF is intentionally disabled — see WRORobot/launch/robot.launch.py
    header comment for how to re-enable if OTOS becomes unreliable.
- `base_link` — robot body root. Owned by `robot_state_publisher`
  processing `WRORobot/urdf/wro_bot.urdf.xacro`.
- `base_laser`, `imu_link`, `camera_link`, `camera_optical_frame`, wheel
  frames — all published by `robot_state_publisher` from the same URDF.

Only one publisher per TF edge. Any duplication is a bug.

---

## Topics behavior consumes

| Topic | Type | Publisher | Rate |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Sim: bridge from `/lidar`. Real: `ldlidar_ros2`. | ~10 Hz |
| `/odom` | `nav_msgs/Odometry` | Sim: bridge. Real: `otos_node`. | ≥30 Hz |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | as above per edge | — |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Nav2 AMCL | ~2 Hz |
| `/camera/image_raw`, `/camera_info` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | Sim: bridge. Real: TBD (not wired yet — Limelight-shaped). | ~30 Hz |
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

## Topics behavior/nav publishes → hardware or sim consumes

| Topic | Type | Consumer | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Sim: bridge → AckermannSteering. Real: `Motor` node (ESC) + `Servo` node (steering servo). | Nav2 publishes via `velocity_smoother`; behavior can also publish for teleop-style tests. |

---

## Ackermann kinematics assumptions

`/cmd_vel.linear.x` is the desired forward velocity in m/s.
`/cmd_vel.angular.z` is the desired yaw rate in rad/s.

The controller and drive plugin/driver must convert `(vx, ωz)` to
`(wheel_speed, steering_angle)` using the wheelbase and steering limits
declared in `WRORobot/urdf/wro_bot.urdf.xacro`. If those values change,
both sides must re-read the URDF at launch.

There is no support for `/cmd_vel.linear.y` — the robot cannot strafe.
Nav2 params set `vy_max: 0.0` and `motion_model: Ackermann`.

---

## Guarantees each side must uphold

**Hardware side (`WRORobot/hardware.launch.py`):**
- `/odom` at ≥ 30 Hz; drift < 5 cm over a 30 s straight-line run.
- `/scan` at ≥ 10 Hz; frame_id `base_laser`.
- `/imu/data_raw` at ≥ 100 Hz; frame_id `imu_link`.
- `/cmd_vel` → motion within 50 ms.
- No writes to `/tf` except `odom → base_link` (from `otos_node`) and
  robot_state_publisher's `base_link → *`.

**Sim side (`wro_sim/sim.launch.py`):**
- Same rates and frame_ids as above, matched by `ros_gz_bridge`.
- Physics stable at 1× real time on a target dev laptop.

**Nav side (`wro_nav2/nav2.launch.py`):**
- Consumes only the topics/frames listed above.
- Publishes `/cmd_vel` (via smoother) and `map → odom`.

**Behavior side (`wro_behavior/behavior.launch.py`):**
- Consumes only the topics/actions listed above.
- Never imports from hardware or sim packages.

---

## When something on this list changes

1. Open a PR that edits **only** `INTERFACE.md` first.
2. Get review from one engineer per side.
3. Only after that PR merges, implement the change in the affected packages.

This keeps the two engineers unblocked — if one changes the interface
unilaterally, the other's code will silently break at competition, not at
merge time.
