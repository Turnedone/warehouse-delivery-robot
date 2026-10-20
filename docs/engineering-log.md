# Engineering log

Problems found while building the project, how each was tracked down, and what was changed.
Almost all of them were integration problems between correct-looking pieces: the kind a
robotics team spends most of its time on. The project was built with an AI coding assistant;
this log records what the runs and tests actually showed.

---

### 1. The costmaps silently ignored their configuration (Nav2 composition)

**Symptom.** Navigation worked, but the keep-out and speed filters had no effect and the local
costmap listed a `static_layer` that the configuration did not ask for.
**Investigation.** The log showed the local costmap loading Nav2's *default* plugin list. The
controller and planner servers had their parameters (MPPI and Smac were loaded), only the costmaps
did not.
**Cause.** The costmaps are child nodes created inside `controller_server` and `planner_server`.
In a component container they read parameters from the **process-wide** `--params-file`, not from
the parameters given to their parent component.
**Fix.** Pass the parameter file to the container process itself as well (`navigation.launch.py`),
as Nav2's own bringup does.
**Lesson.** "It navigates" is not evidence that the configuration was applied: check the log
for what each plugin actually loaded.

### 2. Filter topics resolved into the costmap's namespace

**Symptom.** With the parameters loaded, both filters logged `Filter mask was not received`
every second, and `/speed_limit` never published.
**Investigation.** `ros2 topic list -v`: the mask and info servers published
`/keepout_costmap_filter_info`, the costmaps subscribed to
`/global_costmap/keepout_costmap_filter_info`.
**Cause.** Relative topic names resolve inside the node's namespace, and the costmaps live in
`/global_costmap` and `/local_costmap`.
**Fix.** Absolute names for every filter topic (`filter_info_topic`, `mask_topic`,
`speed_limit_topic`).

### 3. A keep-out zone that only kept the robot's centre out

**Symptom.** The robot cut through the corner of the pallet-staging keep-out zone.
**Cause.** Costmap filters are applied *after* the inflation layer, so keep-out cells are lethal
but not inflated: the planner only keeps the robot's centre out of them.
**Fix.** The generator grows every keep-out rectangle by `keepout_margin` (robot radius + 7 cm)
in the mask, while the floor marking in Gazebo shows the zone itself. A test checks the margin.

### 4. ...which then closed an aisle

**Symptom.** After growing the mask, the robot got stuck 4.6 m short of the shipping station,
spinning and backing up until the controller gave up.
**Cause.** The grown zone left a 0.3 m gap next to the end of rack C2, narrower than the robot.
The planner's point-robot search squeezed a path through, the controller could not follow it.
**Fix.** Moved the zone off the rack end. **Lesson:** a safety margin changes the free space; the
layout has to be re-checked with it, not without it.

### 5. The loopback simulator waited on its own clock

**Symptom.** In loopback mode nothing happened: no `odom` frame, lifecycle bonds timing out.
**Cause.** The loopback simulator *publishes* `/clock`, and it had been started with
`use_sim_time: true`, so its clock waited for itself.
**Fix (first).** Run the clock source on wall time. See entry 11 for the final fix.

### 6. A full battery looked like "not on the dock"

**Symptom.** The first order failed at `UNDOCKING`: the docking server logged
`Robot is not in the dock, no need to undock` and aborted.
**Cause.** With `use_battery_status: true`, `SimpleChargingDock` decides "docked" from the charge
current. The battery model drew zero current at 100 %, which is also what an undocked robot shows.
**Fix.** The battery model reports a small trickle current on a full battery while on the
charger, as a real charger's float phase does. Without it, docking a full robot would also have
failed (the server waits for "charging" after the approach).

### 7. Nav2 BT nodes need their servers when the tree is built

**Symptom.** The task manager crashed at startup: `Action server dock_robot not available`.
**Cause.** Nav2's BT action nodes look for their action server in the constructor; the tree had
been built before Nav2 was up.
**Fix.** Build the trees lazily from the mission thread, retrying until the servers answer.
Orders wait in the queue meanwhile.

### 8. The low-battery check never fired (fresh subscriptions)

**Symptom.** A run starting at 40 % battery delivered three orders and ended at 0 %; the robot
never went to charge, although every order started with `CHECKING_BATTERY`.
**Cause.** The tree was rebuilt for every order. Nav2's `IsBatteryLow` condition subscribes to
`/battery_state` when constructed and only processes messages when ticked; a brand-new
subscription ticked microseconds later has seen no message, so it answered "not low" every time.
**Fix.** Build each tree once and reuse it (what `bt_navigator` does), and replace `IsBatteryLow`
with a `BatteryBelow` condition reading the task manager's long-lived battery subscription. No
reading at all counts as "low": the robot does not start a delivery without knowing its charge.
**Lesson.** A condition with hidden state (a subscription) is not a pure function of the
blackboard; where it is created matters.

### 9. The metrics report claimed 57 years of recording

**Symptom.** `recorded time 29875163 min`, `0.0 deliveries per hour`.
**Cause.** The report used header stamps; messages published before `/clock` started carry a zero
stamp, and different topics use different clocks.
**Fix.** Use the bag's receive timestamps: one clock for every topic.

### 10. The distance driven came out doubled

**Symptom.** The bag report said 214 m; the task manager's per-order odometry summed to 106 m.
**Investigation.** Reading the bag in its own order gave 106.2 m. Only the report's code
disagreed.
**Cause.** The report sorted odometry samples by `(time, x, y)`. With sim time, receive stamps
repeat (the loopback `/clock` ticked at 10 Hz) and the tuple sort reordered ties by `x`,
zig-zagging the path.
**Fix.** Keep bag order. A test now feeds samples with identical stamps.

### 11. Every lifecycle bond timed out at once

**Symptom.** In the integration test, a few seconds after startup all three lifecycle managers
reported `SERVER ... IS DOWN after not receiving a heartbeat for 10000 ms` and shut Nav2 down,
although the servers re-activated within a second (they were alive).
**Investigation.** The log showed the test's first order starting while `bt_navigator`,
`behavior_server` and `docking_server` were still inactive: every goal was rejected, and the
retry branch (clear both costmaps, wait, try again) ran three times in about 50 ms, hammering
half-activated costmaps with service calls. Running the same order by hand a few seconds later
never reproduced it.
**Fix.** The task manager waits until Nav2 reports active (each lifecycle manager's `is_active`
service, the same check as `nav2_simple_commander`'s `waitUntilNav2Active`), until the robot is
localized and until the battery reports. Separately, loopback mode now runs entirely on wall time
(`use_sim_time: false`, no `/clock`), which removes the clock jump from entry 5 for good.
**Lesson.** "Ready" is a system state; a mission layer has to check it, not assume it.

### 12. The fake lidar was blank for the whole run (startup race)

**Symptom.** Every recorded loopback scan was all `inf`, so the near-miss metric had no data.
**Cause.** Nav2's loopback simulator looks up the base → lidar transform only in a setup timer,
which it destroys on the first `/initialpose`. If the initial pose arrives before
`robot_state_publisher`'s static transforms, the lookup never happens again.
**Fix.** `initial_pose_publisher` waits until base → lidar has been available for a second before
sending the pose.

### 13. One node name for two nodes

**Symptom.** `Publisher already registered for node name: 'task_manager'`.
**Cause.** The launch file's `name=` becomes a process-wide `__node:=task_manager` remap, which
also renamed the second node the process creates (the client node the Nav2 BT nodes use).
**Fix.** Give that node its own explicit `__node` remap, as `bt_navigator` does.

### 14. Canceling a queued order waited for the running one

**Symptom.** In the test, canceling the second queued order produced no result for 30 s.
**Cause.** Cancel requests of queued goals were processed between orders only.
**Fix.** Process them from the BT loop's per-tick hook too.

### 15. Nav2's container segfaults on shutdown (accepted, not fixed)

**Symptom.** At the end of a run `Magick: abort due to signal 11 (SIGSEGV)` and exit code -6 from
the component container.
**Status.** Happens after all work is done, during teardown, inside Nav2/GraphicsMagick
(map_server's image library installs the signal handler that prints it); the same crash was
seen with Nav2's own demo launch in the robot-sim project. The shutdown test allows exit -6/-11
for the Nav2 container only and still fails if any other process crashes.

### 16. WSL clears /tmp between sessions

**Symptom.** A bag recorded a minute earlier did not exist any more.
**Cause.** In this WSL distribution `/tmp` does not survive the distribution being idle.
**Fix.** Bags go to `~/wdr_bags` and logs to `~/wdr_logs`.
