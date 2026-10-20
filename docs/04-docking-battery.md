# 4. Battery and docking

## Battery simulator (`wdr_sim/battery_sim`)

Publishes `sensor_msgs/BatteryState` on `/battery_state` at 5 Hz from an energy model
(`wdr_sim/battery_model.*`, unit tested without ROS):

- 26 Wh pack (the Create 3's), drained by a base load (6 W: computer, lidar) plus 25 W per m/s of
  speed, read from `/odom`;
- charged at 30 W while the robot is **on the charger**: TF `map → base_link` within 12 cm of the
  dock pose, standing in for the dock's charging contacts;
- current follows the `BatteryState` convention: negative discharging, positive charging, and a
  small trickle current on a full battery while on the charger;
- `drain_scale` and `charge_scale` compress hours into minutes for demos and tests;
  `~/set_level` sets the charge directly.

Simulated time: the model integrates over the ROS clock, so a Gazebo run that is slower than real
time drains by simulated seconds, not wall seconds.

## Docking server (`opennav_docking`)

The docking server in Nav2 Jazzy runs a docking action as:

1. **Navigate to the staging pose** (0.7 m in front of the dock, using Nav2's `NavigateToPose`);
2. **Detect the dock** pose (from a camera / AprilTag on a real robot);
3. **Approach** with its own graceful controller at 0.15 m/s, with collision checking on the
   local costmap, until within 5 cm of the docking pose;
4. **Wait for charging** to start, retrying the approach up to 3 times otherwise.

Undocking reverses to the staging pose and waits until charging has stopped.

`SimpleChargingDock` is configured with `use_external_detection_pose: false` (no camera here:
the dock pose comes from the generated dock database `config/docks.yaml`) and
`use_battery_status: true`: "docked" and "charging" are read from `/battery_state` (current above
0.5 A), as on a real base that reports charge current on the contacts.

The trickle current matters: without it, a robot on the dock with a full battery shows zero
current, the docking server concludes "not on the dock", undocking aborts and docking a full
robot fails at step 4. See the [engineering log](engineering-log.md), entry 6.

## When the robot charges

- **Before every order** (`EnsureCharged`): below 30 % it docks and charges to 90 % first. The
  threshold is checked per order, not continuously: a robot carrying goods finishes the delivery
  (30 % is far more than one trip needs).
- **When idle** for 10 s: back to the charger (`ReturnToDock`), interrupted by the next order.

Both use the same `Dock` subtree, which skips docking when the `docked` flag is set and retries
the docking action once.

## Verified

Loopback run starting at 40 % with 15× drain and 20× charging: orders 1 and 2 delivered, order 3
started at 10 % → `GOING_TO_CHARGE` → docked → `CHARGING` to 90 % → `UNDOCKING` → delivered. The
integration test `test_4_low_battery_docks_and_charges_first` checks the same sequence in CI.

## What a real deployment would change

- Detect the dock with a camera or AprilTag (`use_external_detection_pose: true`) instead of
  trusting the map pose: AMCL is accurate to a few centimetres, contacts need better.
- Plan charging from predicted energy per order instead of one threshold, and stagger charging
  across a fleet.
- Read docking state from the base (the Create 3 publishes `dock_status`) instead of charge current.
