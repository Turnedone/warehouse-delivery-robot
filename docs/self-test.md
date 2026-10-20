# Self-test

Questions an interviewer could ask about this project. Answer each out loud before opening it;
every answer points to where it is explained.

## System and architecture

<details><summary>1. Walk through what happens between <code>submit_orders shelf_a1:pack_1</code> and the robot moving.</summary>

The CLI sends a `DeliverOrder` goal. The task manager validates the stations, accepts the goal
deferred and queues it. The mission thread pops it, calls `execute()`, sets `pickup`/`dropoff` on
the blackboard and ticks the tree: battery check, undock (`UndockRobot` → docking server), then
`GetStationPose` → `NavigateToPose` → `bt_navigator` runs Nav2's navigation tree → planner
(Smac) computes a path, controller (MPPI) computes `cmd_vel_nav` → velocity smoother → collision
monitor → `cmd_vel` → Gazebo's DiffDrive. [03-mission](03-mission.md), [02-navigation](02-navigation.md)
</details>

<details><summary>2. Why is an order an action and not a service or a topic?</summary>

It runs for minutes, has progress, a result, and must be cancelable. A service blocks with no
feedback and no cancel; a topic has no result or acceptance. [03-mission](03-mission.md)
</details>

<details><summary>3. What does ACCEPT_AND_DEFER buy you?</summary>

A queued goal stays ACCEPTED until the task manager calls `execute()`, so clients see "queued"
vs "executing" in the standard goal status, and a queued goal can be canceled cleanly.
</details>

<details><summary>4. What are lifecycle nodes and bonds, and how did they bite this project?</summary>

Nodes with explicit configure/activate/deactivate states, brought up in order by a lifecycle
manager, which keeps a heartbeat bond with each and tears the group down if one dies. The mission
started sending goals while servers were still activating; the fix was to wait for each
manager's `is_active` service. [engineering log](engineering-log.md) #11
</details>

<details><summary>5. Why one component container for all Nav2 servers? What pitfall did it have?</summary>

Fewer processes, intra-process messaging, one thing to monitor. Pitfall: child nodes (the
costmaps) read their parameters from the process-wide params file, so the file has to be given
to the container too. [engineering log](engineering-log.md) #1
</details>

## Navigation

<details><summary>6. How does AMCL work, in two sentences? Which parameters did you change and why?</summary>

A particle filter: particles move with the odometry motion model plus noise, are weighted by how
well the scan matches the map (likelihood field), and are resampled. Changed: update every 10 cm /
0.1 rad for a tight pose near the dock, 12 m laser range, initial pose from a publisher.
[02-navigation](02-navigation.md)
</details>

<details><summary>7. Why Smac 2D and not Hybrid-A* or the State Lattice planner?</summary>

The TB4 is circular and turns in place; the 2D planner is the recommended one for that. Hybrid-A*
and Lattice model a turning radius and non-circular footprint, worth it for car-like or
rectangular robots. [02-navigation](02-navigation.md)
</details>

<details><summary>8. Explain MPPI. Why is it good with moving obstacles?</summary>

It samples thousands of noisy control sequences over a horizon, rolls each out with the motion
model, scores them with critics (path following, obstacle cost, goal, forward preference,
constraints) and takes a cost-weighted average, every cycle. Because it re-optimizes against the
fresh local costmap at 20 Hz, a moving obstacle simply changes the costs. [02-navigation](02-navigation.md)
</details>

<details><summary>9. What is the difference between the inflation layer and the collision monitor?</summary>

Inflation makes cells near obstacles expensive so planner and controller keep a margin; it only
affects choices. The collision monitor sits after the controller and overrides commands (stop,
slow down) from raw lidar data, independent of the costmaps: the last safety layer.
</details>

<details><summary>10. How do costmap filters work? Why did the keep-out mask need a margin?</summary>

A mask map plus a filter-info message (type, base, multiplier). The keep-out filter makes mask
cells lethal; the speed filter turns mask values into a speed limit published to the controller.
Filters are applied after inflation, so keep-out cells are not inflated and only the robot's
centre is kept out; the generator grows the zones by robot radius + clearance.
[engineering log](engineering-log.md) #3–4
</details>

<details><summary>11. How does a mask pixel become "40 % speed"?</summary>

`map_server` in `scale` mode loads `rint((255 − p) / 255 × 100)`; the speed filter (type 1,
percent) reads that as percent of the maximum speed, 0 = no limit. The generator picks
p = 153 for 40 %, and a test checks the round trip.
</details>

## Mission and behavior trees

<details><summary>12. Why a behavior tree rather than a state machine?</summary>

Reusable subtrees (Dock, NavigateWithRetry), failure handling as structure (Fallback,
RetryUntilSuccessful) rather than extra transitions, local changes, and the same engine and plugins
as Nav2. [03-mission](03-mission.md)
</details>

<details><summary>13. What do Sequence, Fallback, RetryUntilSuccessful and ForceFailure do here?</summary>

Sequence: run children in order, fail on the first failure. Fallback: try children until one
succeeds (deliver, else park the load in the exception area). Retry: re-run a failing child up to
N times (navigation legs). ForceFailure: run a recovery but still report failure so the Retry
tries again.
</details>

<details><summary>14. Why did the battery check never fire at first?</summary>

The tree was rebuilt per order; Nav2's `IsBatteryLow` subscribes when constructed and only reads
messages when ticked, so a new instance had seen none and said "not low". Trees are now built once,
and the condition reads a long-lived monitor; no data counts as low. [engineering log](engineering-log.md) #8
</details>

<details><summary>15. How are failures reported, and what happens to goods on board?</summary>

The last phase state maps to a result code (`TO_PICKUP` → `PICKUP_UNREACHABLE`, ...); retries
don't change the phase. If `loaded`, the robot drives to `qa_hold` before the order fails.
</details>

<details><summary>16. What are the two levels of navigation recovery?</summary>

Inside Nav2's own tree (replan at 1 Hz, clear costmaps, spin, wait, back up), and in the mission
(clear both costmaps, wait 3 s, retry `NavigateToPose`, three attempts per leg).
</details>

## Docking and battery

<details><summary>17. Describe opennav_docking's docking sequence.</summary>

Navigate to the staging pose, detect the dock, approach with its own controller until within the
docking threshold, wait for charging (retry otherwise). Undock: back to the staging pose until
charging stops. [04-docking-battery](04-docking-battery.md)
</details>

<details><summary>18. How does the docking server know the robot is docked here, and what broke?</summary>

`use_battery_status`: charge current above 0.5 A. A full battery drew 0 A and looked undocked; the
model now reports a trickle current on the charger. [engineering log](engineering-log.md) #6
</details>

<details><summary>19a. How does the robot decide to charge before an order?</summary>

Below 30 %, or when an estimate of the trip (robot → pick-up → drop-off → dock, straight line × 1.4,
power model, operator waits) would leave less than 15 %. The threshold alone let the battery hit 0 %.
[04-docking-battery](04-docking-battery.md)
</details>

<details><summary>19b. What happens if Nav2 dies in the middle of a delivery?</summary>

The task manager's 1 Hz supervisor sees a lifecycle manager inactive (or sim time frozen), halts the
tree, puts the order back at the front of the queue as WAITING_FOR_NAV2, asks the managers to RESUME
then STARTUP, and resumes the order; a loaded robot skips the pick-up. A failing order double-checks
Nav2 first, because with Nav2 down all goals fail before the supervisor's next tick.
[engineering log](engineering-log.md) #16
</details>

<details><summary>19. What would you change for a real dock?</summary>

Detect the dock (AprilTag / camera, `use_external_detection_pose`), read the base's dock status
instead of current, plan charging from predicted energy, stagger charging across a fleet.
</details>

## Testing, observability, tooling

<details><summary>20. What does the loopback simulator do, and why test on it?</summary>

It integrates `cmd_vel` into odometry, publishes perfect localization and a scan ray-cast from the
map. Everything above the controller (BTs, mission, docking sequence) runs for real, in seconds,
without a GPU: suitable for CI. [01-simulation](01-simulation.md)
</details>

<details><summary>21. What can't the loopback tests catch?</summary>

Localization errors, physics and wheel slip, real sensor noise, moving obstacles, controller tuning,
timing under a slow simulator: those need the Gazebo scenario.
</details>

<details><summary>22. How is a near miss defined, and why with hysteresis?</summary>

Closest lidar return under 0.35 m starts an episode; it ends when clearance is back above 0.5 m.
Without hysteresis a robot standing next to a worker would count once per scan. Docking is
excluded. [06-observability](06-observability.md)
</details>

<details><summary>23. Why did the distance metric come out doubled once?</summary>

Sorting samples by (time, x, y): sim-time receive stamps repeat, and ties were reordered by x.
Bag order is chronological; keep it. [engineering log](engineering-log.md) #10
</details>

<details><summary>24. Why use_sim_time in Gazebo but not in loopback mode?</summary>

Gazebo drives a simulation clock; every node must follow it. The loopback simulator runs in real
time; giving it a clock only adds a jump from t = 0 when its clock starts late.
</details>

<details><summary>25. Why generate the world, map and masks from one file?</summary>

They must agree to the centimetre (AMCL matches scans against the map, masks overlay it). Generated
from the same rectangles, they cannot drift; a test catches stale files.
</details>
