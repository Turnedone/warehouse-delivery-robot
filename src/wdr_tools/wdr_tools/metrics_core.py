"""Fleet-style KPIs computed from a recorded run, independent of how the data was read.

Inputs are plain Python values (times in seconds), so the calculations are unit tested
without a bag file:
    events     [(t, order_id, event, state, error_code), ...]  from /wdr/order_events
    odom       [(t, x, y), ...]                                   from /odom
    scan_min   [(t, min_range), ...]                              closest lidar return per scan
    battery    [(t, percentage), ...]                             from /battery_state
    estimate   [(t, x, y, yaw), ...]                              from /amcl_pose (Gazebo runs)
    truth      [(t, x, y, yaw), ...]                              from /ground_truth/odom
"""

import bisect

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class OrderRecord:
    order_id: str
    queued: Optional[float] = None
    started: Optional[float] = None
    finished: Optional[float] = None
    outcome: Optional[str] = None      # succeeded | failed | canceled
    error_code: int = 0
    states: List[str] = field(default_factory=list)


@dataclass
class Report:
    duration_s: float
    orders_total: int
    delivered: int
    failed: int
    canceled: int
    success_rate: float                # delivered / finished orders
    deliveries_per_hour: float
    mean_order_time_s: Optional[float]  # start of execution -> delivered
    mean_queue_time_s: Optional[float]
    distance_m: float
    near_misses: int
    min_clearance_m: Optional[float]
    dock_visits: int
    battery_start: Optional[float]
    battery_end: Optional[float]
    battery_min: Optional[float]
    failures_by_code: Dict[int, int]
    loc_samples: int = 0                    # AMCL estimates matched to a ground-truth pose
    loc_mean_m: Optional[float] = None      # localization error, position
    loc_p95_m: Optional[float] = None
    loc_max_m: Optional[float] = None
    loc_yaw_mean_deg: Optional[float] = None


def collect_orders(events) -> Dict[str, OrderRecord]:
    orders: Dict[str, OrderRecord] = {}
    for t, order_id, event, state, code in sorted(events, key=lambda e: e[0]):
        if not order_id:
            continue  # idle-time events (return to dock) belong to no order
        rec = orders.setdefault(order_id, OrderRecord(order_id))
        if event == 'queued':
            rec.queued = t
        elif event == 'started':
            rec.started = t
        elif event == 'state' and state:
            rec.states.append(state)
        elif event in ('succeeded', 'failed', 'canceled'):
            rec.finished = t
            rec.outcome = event
            rec.error_code = code
    return orders


def path_length(odom) -> float:
    """Distance driven, ignoring sub-centimetre jitter so a parked robot adds nothing."""
    total = 0.0
    last = None
    for _, x, y in odom:
        if last is not None:
            step = math.hypot(x - last[0], y - last[1])
            if step >= 0.005:
                total += step
                last = (x, y)
        else:
            last = (x, y)
    return total


def count_near_misses(scan_min, threshold_m: float, rearm_m: float) -> Tuple[int, Optional[float]]:
    """Counts episodes where the closest lidar return came within threshold_m.

    Hysteresis: one episode lasts until the clearance is back above rearm_m, so a robot
    standing next to a worker for ten scans counts once, not ten times.
    """
    count = 0
    inside = False
    min_seen = None
    for _, r in scan_min:
        if r is None or not math.isfinite(r):
            continue
        min_seen = r if min_seen is None else min(min_seen, r)
        if not inside and r < threshold_m:
            count += 1
            inside = True
        elif inside and r > rearm_m:
            inside = False
    return count, min_seen


def localization_errors(estimate, truth, max_dt=0.1):
    """Position and heading error of each estimate against the ground truth nearest in time.

    Estimates with no ground-truth sample within max_dt seconds are skipped. Returns
    [(position_error_m, yaw_error_rad), ...].
    """
    if not truth:
        return []
    truth = sorted(truth, key=lambda s: s[0])
    times = [s[0] for s in truth]
    out = []
    for t, x, y, yaw in estimate:
        i = bisect.bisect_left(times, t)
        best = min((j for j in (i - 1, i) if 0 <= j < len(truth)), key=lambda j: abs(times[j] - t))
        if abs(times[best] - t) > max_dt:
            continue
        _, tx, ty, tyaw = truth[best]
        dyaw = math.atan2(math.sin(yaw - tyaw), math.cos(yaw - tyaw))
        out.append((math.hypot(x - tx, y - ty), abs(dyaw)))
    return out


def percentile(values, q):
    """q-th percentile (0..100) by linear interpolation."""
    v = sorted(values)
    if not v:
        return None
    pos = (len(v) - 1) * q / 100.0
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (pos - lo)


def compute_report(events, odom, scan_min, battery, near_miss_m=0.35, rearm_m=0.5,
                   ignore_windows=(), estimate=(), truth=()) -> Report:
    """ignore_windows: [(t0, t1), ...] during which near misses are not counted (docking)."""
    orders = collect_orders(events)
    times = [e[0] for e in events] + [o[0] for o in odom] + [b[0] for b in battery]
    duration = (max(times) - min(times)) if times else 0.0

    finished = [o for o in orders.values() if o.outcome]
    delivered = [o for o in finished if o.outcome == 'succeeded']
    failed = [o for o in finished if o.outcome == 'failed']
    canceled = [o for o in finished if o.outcome == 'canceled']
    order_times = [o.finished - o.started for o in delivered if o.started is not None]
    queue_times = [o.started - o.queued for o in orders.values()
                   if o.started is not None and o.queued is not None]

    def outside_windows(sample):
        return not any(t0 <= sample[0] <= t1 for t0, t1 in ignore_windows)

    near, min_clear = count_near_misses([s for s in scan_min if outside_windows(s)],
                                        near_miss_m, rearm_m)

    dock_visits = sum(1 for e in events if e[2] == 'state' and e[3] in ('GOING_TO_CHARGE',
                                                                       'RETURNING_TO_DOCK'))
    failures_by_code: Dict[int, int] = {}
    for o in failed:
        failures_by_code[o.error_code] = failures_by_code.get(o.error_code, 0) + 1

    levels = [b[1] for b in sorted(battery, key=lambda b: b[0])]
    loc = localization_errors(estimate, truth)
    pos_err = [e[0] for e in loc]
    return Report(
        duration_s=duration,
        orders_total=len(orders),
        delivered=len(delivered),
        failed=len(failed),
        canceled=len(canceled),
        success_rate=(len(delivered) / len(finished)) if finished else 0.0,
        deliveries_per_hour=(len(delivered) / duration * 3600.0) if duration > 0 else 0.0,
        mean_order_time_s=(sum(order_times) / len(order_times)) if order_times else None,
        mean_queue_time_s=(sum(queue_times) / len(queue_times)) if queue_times else None,
        # Bag order is chronological. Do not sort: with sim time, many samples share one
        # receive stamp (/clock ticks at 10 Hz in loopback) and a tuple sort would reorder them
        # by x, zig-zagging the path (this once doubled the reported distance).
        distance_m=path_length(odom),
        near_misses=near,
        min_clearance_m=min_clear,
        dock_visits=dock_visits,
        battery_start=levels[0] if levels else None,
        battery_end=levels[-1] if levels else None,
        battery_min=min(levels) if levels else None,
        failures_by_code=failures_by_code,
        loc_samples=len(loc),
        loc_mean_m=(sum(pos_err) / len(pos_err)) if pos_err else None,
        loc_p95_m=percentile(pos_err, 95),
        loc_max_m=max(pos_err) if pos_err else None,
        loc_yaw_mean_deg=(math.degrees(sum(e[1] for e in loc) / len(loc))) if loc else None,
    )


def docking_windows(events, margin_s=2.0):
    """Time windows in which the robot drives onto or off the charger on purpose.

    Docking brings the lidar close to the charger and wall by design; those are not near misses.
    """
    windows = []
    start = None
    for t, _, event, state, _ in sorted(events, key=lambda e: e[0]):
        if event != 'state':
            continue
        if state in ('GOING_TO_CHARGE', 'CHARGING', 'RETURNING_TO_DOCK', 'UNDOCKING'):
            start = t if start is None else start
        elif start is not None:
            windows.append((start - margin_s, t + margin_s))
            start = None
    if start is not None:
        windows.append((start - margin_s, float('inf')))
    return windows


def format_report(r: Report) -> str:
    def opt(v, fmt):
        return fmt.format(v) if v is not None else 'n/a'

    lines = [
        'Warehouse delivery run report',
        '-----------------------------',
        f'recorded time        {r.duration_s / 60:.1f} min',
        f'orders               {r.orders_total} ({r.delivered} delivered, {r.failed} failed, '
        f'{r.canceled} canceled)',
        f'success rate         {r.success_rate * 100:.1f} %',
        f'deliveries per hour  {r.deliveries_per_hour:.1f}',
        f'mean time per order  {opt(r.mean_order_time_s, "{:.1f} s")}  (execution only)',
        f'mean time in queue   {opt(r.mean_queue_time_s, "{:.1f} s")}',
        f'distance driven      {r.distance_m:.1f} m',
        f'near misses          {r.near_misses}  (lidar return closer than threshold, '
        f'docking excluded)',
        f'closest approach     {opt(r.min_clearance_m, "{:.2f} m")}  (from the lidar, centre '
        f'offset included)',
        f'dock visits          {r.dock_visits}',
        f'battery              start {opt(r.battery_start and r.battery_start * 100, "{:.0f}%")}, '
        f'end {opt(r.battery_end and r.battery_end * 100, "{:.0f}%")}, '
        f'min {opt(r.battery_min and r.battery_min * 100, "{:.0f}%")}',
    ]
    if r.loc_samples:
        lines.append(f'localization error   mean {r.loc_mean_m * 100:.1f} cm, 95th percentile '
                     f'{r.loc_p95_m * 100:.1f} cm, max {r.loc_max_m * 100:.1f} cm, heading '
                     f'{r.loc_yaw_mean_deg:.1f} deg  (AMCL vs ground truth, {r.loc_samples} samples)')
    if r.failures_by_code:
        lines.append('failures by code     ' + ', '.join(
            f'{code}: {n}' for code, n in sorted(r.failures_by_code.items())))
    return '\n'.join(lines)
