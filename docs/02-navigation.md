# 2. Navigation with Nav2

Nav2 is the ROS 2 navigation stack most mobile-robot companies build on, or at least benchmark
against. This project uses it as a product team would: configure and compose the upstream
servers, add plugins and filters, and own the bringup.

`wdr_bringup/config/nav2_params.yaml` starts from Nav2 Jazzy's stock file; every change is marked
`WDR:` with its reason.

## Servers and composition

`navigation.launch.py` loads all servers as components into **one container process**
(`component_container_isolated`, one executor thread per node), brought up by three lifecycle
managers:

| Lifecycle manager | Nodes |
|---|---|
| localization | `map_server`, `amcl` (Gazebo) / `map_server` only (loopback) |
| filters | keep-out and speed mask servers + their `costmap_filter_info_server`s |
| navigation | `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `bt_navigator`, `velocity_smoother`, `collision_monitor`, `docking_server` |

**Lifecycle nodes** (configure → activate) make startup ordered and observable: a server only
accepts goals once active, and a lifecycle manager keeps a heartbeat **bond** with each node and
brings the group down if one dies. **Composition** saves processes and allows intra-process
messaging. Unlike `nav2_bringup`'s launch files, the route server and waypoint follower are
left out (unused), and the costmap filters are added.

Velocity chain: controller / behaviors → `cmd_vel_nav` → `velocity_smoother` → `cmd_vel_smoothed`
→ `collision_monitor` → `cmd_vel`. The docking server drives the final dock approach directly.

## Localization: AMCL

Monte Carlo localization (particle filter) against the static map: 500–2000 particles, a
likelihood-field laser model, differential-drive motion model. Changed from stock: updates every
10 cm / 0.1 rad (so the pose stays tight for the dock approach), 12 m laser range, and the initial
pose comes from `initial_pose_publisher` (the same `/initialpose` message RViz's "2D Pose
Estimate" sends) instead of AMCL's own parameter.

## Global planner: Smac 2D

`nav2_smac_planner::SmacPlanner2D` replaces NavFn: A* on the costmap with cost-aware search
(`cost_travel_multiplier: 2.0` pushes paths to the middle of the aisles) and a built-in smoother.
For a **circular differential-drive robot that turns in place**, Smac's 2D variant is the
recommended one: Hybrid-A* and the State Lattice planner model a minimum turning radius and a
non-circular footprint, which only pays off for car-like or rectangular robots (a pallet AMR
would use them). `allow_unknown: false`: never plan through space the map does not know.

## Controller: MPPI

Model Predictive Path Integral control samples 2000 noisy velocity sequences over a 2.8 s horizon
(56 × 0.05 s) every cycle, scores each with **critics** (follow the path, keep away from cost,
reach the goal, prefer driving forwards, respect constraints) and averages them weighted by
score. It handles dynamic obstacles well, because the cost critic sees the local costmap every
cycle. Limits set to the TB4: 0.45 m/s, short reversing only, 1.5 rad/s.

## Costmaps

- **Global** (map frame, whole warehouse): static layer from the map, obstacle layer from the
  lidar, inflation (0.55 m, cost scaling 4.0), plus the keep-out and speed filters.
- **Local** (odom frame, rolling 4 × 4 m): obstacle layer (a 2D obstacle layer, not voxels: a
  planar lidar has nothing to put in a 3D grid), inflation, plus the keep-out filter, so MPPI
  refuses to cut through a zone too.

Robot radius 0.18 m (TB4 0.17 m + 1 cm).

## Costmap filters

A filter is a mask map plus an info message saying how to read it:

- **Keep-out filter** (`type: 0`): mask cells marked occupied become lethal. Used for the
  forklift-only pallet staging area and a maintenance bay, which are open floor without walls.
  Filters are applied after inflation, so the generator grows each zone by the robot radius plus a
  margin in the mask (see the [engineering log](engineering-log.md), entries 3–4).
- **Speed filter** (`type: 1`, percent): mask value = percentage of the controller's maximum
  speed, 0 = no limit. The forklift cross aisle is limited to 40 %, the packing area where people
  work to 50 %. The filter publishes `nav2_msgs/SpeedLimit` on `/speed_limit`; the controller
  server applies it.

The mask PGMs use `mode: scale`, where `map_server` loads a pixel as `rint((255 − p)/255 × 100)`;
the generator picks the pixel that loads back as exactly the configured percent, and a test
checks it.

## Collision monitor

The last safety layer between the controller and the wheels, independent of the costmaps: a
`stop` polygon 12 cm in front of the robot's bumper (hard stop if anything is inside), and the stock
`approach` model (slow down so that a collision is at least 1.2 s away).

## Verified behaviour (Gazebo, AMCL)

Four goals across the warehouse: all reached with 2–18 cm final error, speed limits of 40 % and
50 % applied in the two slow zones, no sample inside a keep-out zone.

## Things a production setup would add

- A route graph (`nav2_route`) with one-way lanes in the aisles, so several robots never meet
  head-on; this single robot plans freely.
- A footprint polygon instead of a radius once a cargo tote overhangs the base.
- Tuning MPPI on recorded bags of real traffic rather than on three scripted obstacles.
