"""Reads a recorded run (rosbag2, MCAP or sqlite3) and prints the delivery KPIs.

    ros2 run wdr_tools metrics ~/wdr_bags/run_20261020_153000
    ros2 run wdr_tools metrics BAG --json            machine-readable output
    ros2 run wdr_tools metrics BAG --near-miss 0.4   different near-miss threshold (m)

Near misses come from the lidar: an episode starts when the closest return is nearer than the
threshold (default 0.35 m from the lidar, which sits ~4 cm behind the centre of a robot with a
0.17 m radius, so ~0.15 m of free space) and ends once it is back above 0.5 m. Driving onto and
off the charger is excluded.
"""

import argparse
import dataclasses
import json
import math
import os
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from wdr_tools.metrics_core import compute_report, docking_windows, format_report

TOPICS = ['/wdr/order_events', '/odom', '/scan', '/battery_state']


def read_bag(path):
    import rosbag2_py  # imported here so the pure helpers above import without ROS bag libs

    storage = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(path)) else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    msg_types = {name: get_message(types[name]) for name in TOPICS if name in types}

    # Times come from the bag's receive timestamps, not the message headers: one clock for
    # every topic (sim time when recorded with --use-sim-time), and no zero stamps from
    # messages published before /clock started.
    events, odom, scan_min, battery = [], [], [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        msg = deserialize_message(data, msg_types[topic])
        if topic == '/wdr/order_events':
            events.append((t, msg.order_id, msg.event, msg.state, msg.error_code))
        elif topic == '/odom':
            odom.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))
        elif topic == '/scan':
            valid = [r for r in msg.ranges
                     if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
            scan_min.append((t, min(valid) if valid else None))
        elif topic == '/battery_state':
            battery.append((t, msg.percentage))
    missing = [t for t in TOPICS if t not in types]
    return events, odom, scan_min, battery, missing


def main(argv=None):
    parser = argparse.ArgumentParser(description='Delivery KPIs from a recorded bag.')
    parser.add_argument('bag', help='bag directory (as written by ros2 bag record)')
    parser.add_argument('--near-miss', type=float, default=0.35, help='threshold in metres')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)

    if not os.path.isdir(args.bag):
        print(f'{args.bag} is not a bag directory', file=sys.stderr)
        return 2
    events, odom, scan_min, battery, missing = read_bag(args.bag)
    if missing:
        print('note: topics not in the bag: ' + ', '.join(missing), file=sys.stderr)
    report = compute_report(events, odom, scan_min, battery, near_miss_m=args.near_miss,
                            ignore_windows=docking_windows(events))
    if args.json:
        print(json.dumps(dataclasses.asdict(report), indent=2))
    else:
        print(format_report(report))
    return 0


if __name__ == '__main__':
    sys.exit(main())
