# 6. Observability: bags, metrics, RViz

## Recording a run

```bash
ros2 launch wdr_bringup warehouse.launch.py record:=true                      # ~/wdr_bags/run_<timestamp>
ros2 launch wdr_bringup warehouse.launch.py record:=true bag_dir:=~/wdr_bags/demo
```

`rosbag2` records to **MCAP** (the default storage in Jazzy: one indexed file, readable by
Foxglove and the ROS 2 tools). Recorded: `/wdr/order_events`, `/battery_state`, `/odom`, `/scan`,
`/tf`, `/tf_static`, `/cmd_vel`, `/speed_limit`, `/collision_monitor_state`, `/plan`,
`/amcl_pose`, `/ground_truth/odom` (Gazebo's true robot pose, for evaluation only), `/wdr/obstacles`,
the action status topics, `/robot_description` and `/map`:
enough to replay the run in RViz or Foxglove and to compute the KPIs offline. Costmaps and
camera data are left out (large, and reproducible from the rest).

## Metrics report (`wdr_tools metrics`)

```bash
ros2 run wdr_tools metrics ~/wdr_bags/demo          # table
ros2 run wdr_tools metrics ~/wdr_bags/demo --json   # for scripts / dashboards
```

| KPI | Definition |
|---|---|
| deliveries per hour | delivered orders / recorded time |
| success rate | delivered / finished (delivered + failed + canceled) |
| mean time per order | `started` → `succeeded` events (execution only) |
| mean time in queue | `queued` → `started` |
| distance driven | odometry path length, sub-centimetre jitter ignored |
| near misses | episodes with the closest lidar return under 0.35 m (hysteresis: ends above 0.5 m), docking excluded |
| closest approach | minimum lidar range outside docking |
| dock visits | times the robot went to charge (low battery or idle) |
| battery | start, end, minimum |
| localization error | Gazebo runs: each `/amcl_pose` estimate against the `/ground_truth/odom` pose nearest in time (within 0.1 s); mean, 95th percentile, maximum, mean heading error |
| failures by code | `DeliverOrder` result codes of failed orders |

All times are the bag's receive timestamps (one clock for every topic), except the localization error,
which pairs message stamps because AMCL's estimate refers to its scan time; see the
[engineering log](engineering-log.md), entries 9–10. The calculations live in
`metrics_core.py` as plain functions over lists, unit tested, plus one test that writes a real
MCAP bag and reads it back.

Why the order events topic: the task manager knows the mission states; publishing every change
as a typed message makes a run auditable after the fact, without parsing logs.

## RViz

```bash
ros2 launch wdr_bringup warehouse.launch.py rviz:=true
```

`rviz/warehouse.rviz` shows the map, keep-out and speed masks, both costmaps, the robot model,
lidar, AMCL particles, the global plan, the collision monitor's stop zone and the true obstacle
positions, with Nav2's goal tool and the initial-pose tool. To replay a bag:
`ros2 bag play ~/wdr_bags/demo --clock` and start RViz with `use_sim_time:=true`.

## Foxglove and Groot2 (optional)

- **Foxglove**: open the MCAP file directly, or run `foxglove_bridge`
  (`apt install ros-jazzy-foxglove-bridge`) for live data. No layout is committed.
- **Groot2**: set `groot_port: 1667` in `wdr_mission/config/task_manager.yaml` to watch the
  mission tree tick live (Nav2's BT engine publishes it).

## Logs

Each node logs mission transitions (`[ORD-0003] GOING_TO_CHARGE`) and rejections with their
reason. `tools/wsl/stop.sh` cleans up leftover processes after a run.
