# wro_behavior

High-level behavior for the WRO robot. This is where mission logic, state
machines, custom BT nodes, waypoint sequences, and vision reactions live.

## Contract

This package consumes the interface pinned in `../INTERFACE.md`. It never
imports from `WRORobot`, `wro_sim`, or any hardware/driver package. If you
find yourself wanting to depend on one, the abstraction is wrong and the
answer is a new topic/action/service in the interface, not a direct import.

## Nodes

- `goto_pose` — hello-world NavigateToPose action client. Sends one goal in
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
