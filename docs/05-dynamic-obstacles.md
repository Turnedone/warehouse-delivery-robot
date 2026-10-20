# 5. Dynamic obstacles

## What moves

Three obstacles from `warehouse_layout.yaml`, moved back and forth along their paths by
`wdr_sim/obstacle_mover`:

| Obstacle | Size | Speed | Path |
|---|---|---|---|
| forklift | 1.2 × 0.8 × 1.8 m | 0.45 m/s | up and down the cross aisle (also a 40 % slow zone) |
| picking cart | 0.7 × 0.5 × 1.0 m | 0.3 m/s | along the aisle between rack rows A and B, past `shelf_a1`/`shelf_a2` |
| worker | 0.45 × 0.45 × 1.7 m | 0.35 m/s | through the packing area, past every drop-off |

The mover calls Gazebo's `/world/warehouse/set_pose` service (bridged by `ros_gz_bridge`) at
10 Hz, so the obstacles are static models that teleport along their path: the rendered lidar sees
them, physics does not push them around. It also publishes their true poses as markers on
`/wdr/obstacles` (RViz and the bag). Path timing lives in `PathPlayer`, unit tested.

Why not Gazebo actors: animated actors need skin meshes from Gazebo Fuel (network access at
startup), and a box with a known path is easier to reason about when tuning.

## How the stack copes

The obstacles are **not in the map**. Nav2 learns about them only through `/scan`:

- the **obstacle layers** of both costmaps mark them, and ray-tracing clears the cells again
  once they have moved on;
- **MPPI** re-optimizes every 50 ms against the local costmap, so it swerves around a cart or
  slows behind it;
- the global plan is recomputed at 1 Hz (Nav2's default navigation tree), so a blocked aisle is
  routed around;
- the **collision monitor** stops the robot if anything gets within its stop polygon;
- if Nav2 still gives up (for example a forklift parked in the only path), the mission clears
  the costmaps, waits and retries ([03-mission](03-mission.md)); after three attempts the order
  fails with a clear error code.

`progress_checker.movement_time_allowance` was raised from 10 s to 15 s: waiting for a forklift
to pass is not being stuck.

## Measuring it

The metrics report counts **near misses**: episodes where the closest lidar return is under
0.35 m (about 15 cm of free space around the TB4's body), ending once the clearance is back above
0.5 m, with docking manoeuvres excluded. It also reports the closest approach of the run.
See [06-observability](06-observability.md).

## Limits

- Teleported obstacles have no momentum; a real forklift does, and a person can stop or turn.
- No prediction: the costmap only knows where an obstacle *is*. MPPI's cost critic reacts to the
  current costmap; tracking obstacles and predicting their motion is the next step for crowded
  floors.
