# 3. Mission layer: orders, queue, behavior tree

## Interfaces (`wdr_interfaces`)

| Interface | Kind | Purpose |
|---|---|---|
| `DeliverOrder` | action | goal: pick-up and drop-off station names, priority; feedback: state, battery, retries; result: success, error code, duration, queue time, distance, retries |
| `OrderEvent` | message on `/wdr/order_events` | every state change of every order; recorded in bags, read back by the metrics report |
| `OperatorConfirm` | service `/wdr/operator_confirm` | an operator confirms "loaded" / "unloaded" |
| `SetBatteryLevel` | service `/battery_sim/set_level` | demos and tests |

An **action** is the right shape for an order: it is long-running, has progress (feedback),
a result, and can be canceled. Station names instead of coordinates keep clients independent of
the map.

## Task manager (`wdr_mission/src/task_manager.cpp`)

- Validates a goal (stations exist and have the right type, queue not full, order id unique) and
  rejects it otherwise.
- Accepts goals with `ACCEPT_AND_DEFER`: a queued order stays in the ACCEPTED state until it
  starts, so a client can tell queued from executing by the goal status alone.
- Keeps an `OrderQueue`: highest priority first, first come first served within a priority
  (unit tested). Canceling a queued order removes it at once.
- Runs one **mission thread** that pops an order and ticks the tree; the ROS executor (action
  server, subscriptions, confirm service) runs on other threads. Shared objects (battery
  monitor, operator panel) are thread-safe.
- Waits before the first order until Nav2 reports active (each lifecycle manager's `is_active`
  service), the robot is localized (TF map → base_link) and the battery reports. See the
  [engineering log](engineering-log.md), entry 11, for what happens without that.
- When idle for 10 s, runs a second tree that drives back to the charger; it is halted as soon
  as an order arrives.

## The behavior tree (`wdr_mission/behavior_trees/deliver_order.xml`)

```
DeliverOrder (Fallback)
├─ Deliver (Sequence)
│   ├─ EnsureCharged: battery ≥ 30 %?  else Dock → wait until 90 %
│   ├─ LeaveDock: if docked, UndockRobot
│   ├─ GetStationPose(pickup) → NavigateWithRetry
│   ├─ WaitForOperator(load)            → loaded := true
│   ├─ GetStationPose(dropoff) → NavigateWithRetry
│   └─ WaitForOperator(unload)          → loaded := false
└─ ParkLoadInExceptionArea (only if loaded): navigate to qa_hold, then fail the order

NavigateWithRetry: up to 3 × [ NavigateToPose  |  on failure: clear both costmaps, wait 3 s ]
```

**Why a behavior tree** and not a state machine: the same subtrees (`Dock`,
`NavigateWithRetry`) are reused in several places, failure handling is explicit structure
(Fallback / Retry) instead of transitions sprinkled over states, and adding a step is local.
It is also what Nav2 itself uses, so the mission layer runs on **Nav2's BT engine**
(`nav2_behavior_tree::BehaviorTreeEngine`, BehaviorTree.CPP v4) and reuses Nav2's BT plugins:
`NavigateToPose`, `ClearEntireCostmap`, `Wait`, and `opennav_docking_bt`'s `DockRobot` /
`UndockRobot`. Custom nodes (`wdr_mission_bt_nodes`, a plugin library):

| Node | Type | Does |
|---|---|---|
| `GetStationPose` | sync action | station name → `PoseStamped` goal |
| `WaitForOperator` | stateful action | waits for a confirmation; a simulated operator confirms after `operator_auto_confirm_s`; fails after `timeout_s` |
| `BatteryBelow` | condition | battery below threshold, or no reading yet |
| `WaitForCharge` | stateful action | waits on the charger until the target charge |
| `ReportState` | sync action | tells the task manager the mission state (feedback, events, error codes) |

`docked` and `loaded` are blackboard flags flipped by `Script` nodes and tested by
`ScriptCondition`; they persist across orders because the tree is built once and reused.

## Two levels of recovery

1. **Inside Nav2**: each `NavigateToPose` runs Nav2's own navigation tree with replanning at 1 Hz
   and controller/planner recoveries (clear costmaps, spin, wait, back up).
2. **In the mission**: if Nav2 still gives up, the mission clears both costmaps (stale marks from a
   forklift that has left), waits and retries, up to three attempts per leg.

If the order still fails, the result's error code says where: the last *phase* state
(`TO_PICKUP` → `PICKUP_UNREACHABLE`, `WAITING_FOR_LOAD` → `LOAD_TIMEOUT`, …), unaffected by retry
and exception-area states. A robot carrying goods parks them in the exception area (`qa_hold`)
instead of driving around with them.

## Trying it

```bash
ros2 run wdr_tools submit_orders shelf_a1:pack_1 shelf_b2:shipping   # queue two, follow them
ros2 action send_goal --feedback /deliver_order wdr_interfaces/action/DeliverOrder \
  "{pickup_station: shelf_c1, dropoff_station: pack_2, priority: 5}"
# With operator_auto_confirm_s:=-1 the robot waits for a person:
ros2 service call /wdr/operator_confirm wdr_interfaces/srv/OperatorConfirm "{step: load}"
```
