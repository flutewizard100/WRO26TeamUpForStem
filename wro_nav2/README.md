# wro_nav2

Nav2 configuration, behavior tree, maps, RViz, and launch files for the WRO
Ackermann robot. Config-only — no C++ / Python nodes, no vendored Nav2.

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
# from your ROS 2 Jazzy workspace (this repo is a flat workspace — no src/)
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
  **not** included — the Ackermann robot cannot spin in place.
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
  publishes that transform — that's a **duplicate tf publisher** and will
  cause TF_REPEATED_DATA on the real robot. Choose one owner. See
  `wro_sim/README.md` "Known bugs from the plan not fixed here".
- **Camera in sim.** The sim exposes a plain 640x480 image on `/camera` (see
  `wro_sim`). Real hardware uses a Limelight over network tables — the sim
  camera is **not** a Limelight equivalent, only a placeholder for future
  ROS-side vision experiments.
