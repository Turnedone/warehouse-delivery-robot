# 1. Simulation: world, robot, map, stations

## What is simulated

A 24 × 16 m warehouse in **Gazebo Harmonic** (`gz sim`, the successor of Gazebo Classic): three
rows of pallet racks split by a cross aisle, a packing area with three tables, an office block,
two roof columns, a charging dock, and three moving obstacles (forklift, picking cart, worker).
Floor markings show the keep-out zones (red), slow zones (yellow), station pads (blue) and the
dock (green).

The robot is a **TurtleBot 4** (iRobot Create 3 base, RPLIDAR A1) from Nav2's
`nav2_minimal_tb4_description`, spawned on the charger. `wdr_bringup/urdf/wdr_tb4.urdf.xacro` is
the TB4 description with the OAK-D RGB-D camera made optional and off: nothing in the stack uses
it, and rendering it costs simulation time.

## One layout file, everything generated

`wdr_bringup/config/warehouse_layout.yaml` is the single source of truth.
`scripts/generate_warehouse.py` turns it into:

| Output | Used by |
|---|---|
| `worlds/warehouse.sdf` | Gazebo |
| `maps/warehouse.{pgm,yaml}` | `map_server` → AMCL and the global costmap's static layer |
| `maps/keepout_mask.{pgm,yaml}`, `maps/speed_mask.{pgm,yaml}` | Nav2 costmap filters |
| `config/stations.yaml` | task manager (station name → pose) |
| `config/docks.yaml` | `opennav_docking` dock database |
| `config/obstacles.yaml` | `obstacle_mover` (paths of the moving obstacles) |

Why generate instead of drawing a map by hand or mapping with SLAM: the world, the map AMCL
localizes against and the filter masks are rasterized from the same rectangles, so they cannot
drift apart, and moving a rack is a one-line change. A test (`test_generated_files.py`) fails if a
generated file is stale, a station sits in an obstacle or a keep-out zone, or a speed-mask pixel
does not load back as the configured percentage.

On a real robot the map would come from SLAM (`slam_toolbox` is installed with Nav2) and the masks
would be drawn over it; the rest of the stack would not change.

**Why not the stock TB4 warehouse.** `nav2_minimal_tb4_sim` ships an AWS-style warehouse world, but
it downloads its models from Gazebo Fuel at startup (no network in CI, slow first start), and its
map does not line up with masks of our own. A generated world is offline, deterministic and sized
for minute-long deliveries.

## Gazebo ↔ ROS 2

`ros_gz_bridge` (`config/gz_bridge.yaml`) carries `/clock`, `/odom`, `/tf` (odom → base_link from
Gazebo's DiffDrive system), `/scan`, `/imu`, `/joint_states` and `/cmd_vel`, plus the `set_pose`
service the obstacle mover uses. `robot_state_publisher` publishes the robot's static transforms
from the URDF. Gazebo runs as a server only (`gz sim -s`); `gui:=true` opens the client window.

## Two simulators

| | Gazebo (`sim:=gazebo`, default) | Nav2 loopback (`sim:=loopback`) |
|---|---|---|
| Physics, wheel slip | yes | no (integrates `cmd_vel`) |
| Lidar | rendered (`gpu_lidar`) | ray-cast from the map |
| Localization | AMCL | perfect (publishes map → odom) |
| Moving obstacles | yes | no |
| Clock | `/clock` from Gazebo | wall time |
| Use | demos, tuning, the full scenario | fast end-to-end tests in CI |

The loopback simulator (`nav2_loopback_sim`) is Nav2's own tool for exactly this: it tests
everything above the controller (behavior trees, mission logic, docking sequence) in seconds
without a GPU.

## Frames

`map → odom` (AMCL, or the loopback simulator) → `base_link` (Gazebo DiffDrive odometry) →
`shell_link → rplidar_link` (URDF). `base_footprint` coincides with `base_link` on the TB4, so the
whole stack uses `base_link`.
