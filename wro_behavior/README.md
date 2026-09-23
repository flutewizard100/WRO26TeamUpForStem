# wro_behavior

High-level behavior for the WRO robot. Mission logic, state machines, and
perception helpers live here.

## Contract

This package consumes the interface pinned in `../INTERFACE.md`. It never
imports from `WRORobot`, `wro_sim`, or any hardware/driver package. If you
find yourself wanting to depend on one, the abstraction is wrong and the
answer is a new topic/action/service in the interface, not a direct import.

## Nodes

- `wro_mission` — unified Open + Obstacle Challenge mission node. State
  machine, direction detection, waypoint dispatch, first-corner guard,
  end-of-race handling.
- `camera_map_augmenter` — subscribes to `/limelight/detections`, projects
  confirmed pillar landmarks into the map frame, publishes them as
  `PointCloud2` on `/camera_obstacles` for Nav2's obstacle layer.
- `sim_limelight_bridge` — sim-only substitute for the real Limelight
  bridge. HSV-detects pillars and lines from `/camera` and publishes
  `vision_msgs/Detection2DArray` on `/limelight/detections`.

## Perception helpers (imported by the nodes above)

- `lidar_perception.LidarPerception` — `/scan` subscription, arc queries,
  forward-edge waypoint generation.
- `camera_perception.CameraPerception` — `/camera_info` + `/limelight/detections`
  subscriptions, pillar/line semantics, pixel → map projection, landmark
  accumulator.

## Run

Sim, full stack (Gazebo + Nav2 + SLAM + mission):
```bash
ros2 launch wro_behavior wro_mission_sim.launch.py
```

Real hardware:
```bash
ros2 launch wro_behavior wro_mission_hw.launch.py
```

Trigger the start button from a terminal (equivalent to pressing the
force sensor):
```bash
ros2 topic pub --once /start_button std_msgs/msg/Bool "{data: true}"
```
