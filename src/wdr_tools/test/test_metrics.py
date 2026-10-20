"""Tests for the run metrics: the pure KPI maths, and a round trip through a real MCAP bag."""

import math

import pytest

from wdr_tools.metrics_core import (collect_orders, compute_report, count_near_misses,
                                    docking_windows, format_report, path_length)


def ev(t, order_id, event, state='', code=0):
    return (t, order_id, event, state, code)


EVENTS = [
    ev(0.0, 'ORD-1', 'queued'), ev(0.0, 'ORD-2', 'queued'), ev(1.0, 'ORD-3', 'queued'),
    ev(1.0, 'ORD-1', 'started'), ev(2.0, 'ORD-1', 'state', 'TO_PICKUP'),
    ev(61.0, 'ORD-1', 'succeeded'),
    ev(61.0, 'ORD-2', 'started'), ev(62.0, 'ORD-2', 'state', 'TO_PICKUP'),
    ev(80.0, 'ORD-2', 'failed', 'TO_PICKUP', 1),
    ev(80.0, 'ORD-3', 'started'), ev(81.0, 'ORD-3', 'state', 'GOING_TO_CHARGE'),
    ev(100.0, 'ORD-3', 'state', 'CHARGING'), ev(150.0, 'ORD-3', 'state', 'UNDOCKING'),
    ev(155.0, 'ORD-3', 'state', 'TO_PICKUP'), ev(200.0, 'ORD-3', 'succeeded'),
    ev(210.0, '', 'state', 'RETURNING_TO_DOCK'),
]


def test_collect_orders_tracks_each_order():
    orders = collect_orders(EVENTS)
    assert set(orders) == {'ORD-1', 'ORD-2', 'ORD-3'}
    assert orders['ORD-1'].outcome == 'succeeded'
    assert orders['ORD-2'].outcome == 'failed' and orders['ORD-2'].error_code == 1
    assert orders['ORD-3'].states[0] == 'GOING_TO_CHARGE'


def test_report_kpis():
    odom = [(0.0, 0.0, 0.0), (100.0, 3.0, 4.0), (200.0, 3.0, 4.0)]
    battery = [(0.0, 0.8), (100.0, 0.25), (200.0, 0.9)]
    r = compute_report(EVENTS, odom, [], battery)
    assert r.orders_total == 3
    assert (r.delivered, r.failed, r.canceled) == (2, 1, 0)
    assert r.success_rate == pytest.approx(2 / 3)
    assert r.duration_s == pytest.approx(210.0)
    assert r.deliveries_per_hour == pytest.approx(2 / 210.0 * 3600)
    assert r.mean_order_time_s == pytest.approx(((61 - 1) + (200 - 80)) / 2)
    assert r.mean_queue_time_s == pytest.approx((1 + 61 + 79) / 3)
    assert r.distance_m == pytest.approx(5.0)
    assert r.dock_visits == 2
    assert (r.battery_start, r.battery_min, r.battery_end) == (0.8, 0.25, 0.9)
    assert r.failures_by_code == {1: 1}
    assert 'success rate' in format_report(r)


def test_path_length_ignores_jitter():
    jitter = [(i * 0.1, 0.001 * (i % 2), 0.0) for i in range(100)]
    assert path_length(jitter) == 0.0
    assert path_length([(0, 0, 0), (1, 1, 0), (2, 1, 1)]) == pytest.approx(2.0)


def test_distance_keeps_bag_order_when_stamps_tie():
    # Sim-time receive stamps repeat (10 Hz /clock); the order of samples must be kept.
    odom = [(1.0, 0.1 * i, 0.0) for i in range(10, 0, -1)]  # x = 1.0, 0.9, ... 0.1
    r = compute_report([], odom, [], [])
    assert r.distance_m == pytest.approx(0.9)


def test_near_misses_use_hysteresis():
    ranges = [1.0, 0.3, 0.2, 0.4, 0.3, 0.6, 0.3, 1.0, math.inf, None]
    count, closest = count_near_misses([(i, r) for i, r in enumerate(ranges)], 0.35, 0.5)
    assert count == 2  # 0.3..0.3 is one episode (never back above 0.5), then 0.3 again
    assert closest == 0.2


def test_docking_is_not_a_near_miss():
    windows = docking_windows(EVENTS, margin_s=0.0)
    assert windows[0] == (81.0, 155.0)
    scan = [(90.0, 0.1), (120.0, 0.1), (170.0, 0.2)]
    r = compute_report(EVENTS, [], scan, [], ignore_windows=windows)
    assert r.near_misses == 1  # only the one at t=170 counts


def test_empty_run():
    r = compute_report([], [], [], [])
    assert r.orders_total == 0 and r.success_rate == 0.0 and r.mean_order_time_s is None
    format_report(r)


def test_bag_round_trip(tmp_path):
    """Writes a small MCAP bag with the real message types and reads it with metrics.read_bag."""
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import BatteryState, LaserScan
    from wdr_interfaces.msg import OrderEvent

    from wdr_tools.metrics import read_bag

    uri = str(tmp_path / 'bag')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for i, (name, typ) in enumerate([('/wdr/order_events', 'wdr_interfaces/msg/OrderEvent'),
                                     ('/odom', 'nav_msgs/msg/Odometry'),
                                     ('/scan', 'sensor_msgs/msg/LaserScan'),
                                     ('/battery_state', 'sensor_msgs/msg/BatteryState')]):
        writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=typ,
                                                     serialization_format='cdr'))

    def write(topic, msg, t):
        writer.write(topic, serialize_message(msg), int(t * 1e9))

    for t, oid, event, state, code in [ev(1, 'A', 'queued'), ev(2, 'A', 'started'),
                                       ev(50, 'A', 'succeeded')]:
        e = OrderEvent(order_id=oid, event=event, state=state, error_code=code)
        e.stamp.sec = t
        write('/wdr/order_events', e, t)
    for t, x in [(1, 0.0), (25, 2.0), (50, 4.0)]:
        o = Odometry()
        o.header.stamp.sec = t
        o.pose.pose.position.x = x
        write('/odom', o, t)
    for t, r in [(10, 1.0), (20, 0.3), (30, 1.0)]:
        s = LaserScan(range_min=0.1, range_max=12.0, ranges=[r, 5.0, float('inf')])
        s.header.stamp.sec = t
        write('/scan', s, t)
    b = BatteryState(percentage=0.7)
    b.header.stamp.sec = 1
    write('/battery_state', b, 1)
    del writer  # closes the bag

    events, odom, scan_min, battery, missing = read_bag(uri)
    assert missing == []
    r = compute_report(events, odom, scan_min, battery)
    assert r.delivered == 1 and r.success_rate == 1.0
    assert r.mean_order_time_s == pytest.approx(48.0)
    assert r.distance_m == pytest.approx(4.0)
    assert r.near_misses == 1
    assert r.min_clearance_m == pytest.approx(0.3)
