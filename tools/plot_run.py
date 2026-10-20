#!/usr/bin/env python3
"""Plots a recorded run: the path driven over the warehouse, and a timeline of the mission.

    python3 tools/plot_run.py ~/wdr_bags/demo docs/images

Writes run_trajectory.png and run_timeline.png. Offscreen (matplotlib Agg), needs a sourced
ROS 2 environment for rosbag2_py.
"""

import math
import os
import sys

import matplotlib
import numpy as np
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, 'src', 'wdr_bringup')
PHASE_COLORS = {
    'UNDOCKING': '#9e9e9e', 'TO_PICKUP': '#1f77b4', 'WAITING_FOR_LOAD': '#aec7e8',
    'TO_DROPOFF': '#9467bd', 'WAITING_FOR_UNLOAD': '#c5b0d5', 'GOING_TO_CHARGE': '#d62728',
    'CHARGING': '#2ca02c', 'NAV_RETRY': '#ff7f0e', 'RETURNING_TO_DOCK': '#8c564b',
}


def read(bag):
    storage = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(bag)) else 'sqlite3'
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = ['/amcl_pose', '/odom', '/battery_state', '/wdr/order_events', '/wdr/obstacles']
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cls = {t: get_message(types[t]) for t in want if t in types}
    pose, speed, battery, events, obstacles = [], [], [], [], {}
    while r.has_next():
        topic, data, t_ns = r.read_next()
        t = t_ns * 1e-9
        m = deserialize_message(data, cls[topic])
        if topic == '/amcl_pose':
            pose.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
        elif topic == '/odom':
            speed.append((t, math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)))
        elif topic == '/battery_state':
            battery.append((t, m.percentage))
        elif topic == '/wdr/order_events':
            events.append((t, m.order_id, m.event, m.state))
        elif topic == '/wdr/obstacles':
            for mk in m.markers:
                obstacles.setdefault(mk.text, []).append((mk.pose.position.x, mk.pose.position.y))
    return pose, speed, battery, events, obstacles


def load_map():
    with open(os.path.join(PKG, 'maps', 'warehouse.yaml')) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(PKG, 'maps', 'warehouse.pgm'), 'rb') as f:
        f.readline()
        w, h = map(int, f.readline().split())
        f.readline()
        img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    return img, [ox, ox + w * res, oy, oy + h * res]


def order_of(t, spans):
    for oid, t0, t1 in spans:
        if t0 <= t <= t1:
            return oid
    return None


def main(bag, out_dir):
    pose, speed, battery, events, obstacles = read(bag)
    with open(os.path.join(PKG, 'config', 'warehouse_layout.yaml')) as f:
        layout = yaml.safe_load(f)
    t0 = min(e[0] for e in events) if events else pose[0][0]

    # order spans: started -> finished
    starts, spans = {}, []
    for t, oid, ev, _ in events:
        if ev == 'started':
            starts[oid] = t
        elif ev in ('succeeded', 'failed', 'canceled') and oid in starts:
            spans.append((oid, starts.pop(oid), t))

    # ---------------------------------------------------------------- trajectory
    img, extent = load_map()
    fig, ax = plt.subplots(figsize=(12, 8.2), dpi=110)
    shade = np.where(img == 0, 0.2, np.where(img == 205, 0.85, 1.0))
    ax.imshow(shade, cmap='gray', vmin=0, vmax=1, extent=extent, origin='upper')
    for z in layout['keepout_zones']:
        x0, y0, x1, y1 = z['rect']
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, color='red', alpha=0.25))
    for z in layout['speed_zones']:
        x0, y0, x1, y1 = z['rect']
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, color='gold', alpha=0.2))
    for name, pts in obstacles.items():
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color='orange', lw=6, alpha=0.25, solid_capstyle='round')
        ax.annotate(name.replace('_', ' '), (xs[0], ys[0]), fontsize=8, color='darkorange',
                    xytext=(5, 5), textcoords='offset points')
    colors = plt.cm.tab10.colors
    order_ids = [s[0] for s in spans]
    for i, oid in enumerate(order_ids):
        seg = [(x, y) for t, x, y in pose if order_of(t, spans) == oid]
        if seg:
            xs, ys = zip(*seg)
            ax.plot(xs, ys, color=colors[i % 10], lw=2.2, label=oid)
    idle = [(x, y) for t, x, y in pose if order_of(t, spans) is None]
    if idle:
        ax.plot(*zip(*idle), '.', color='gray', ms=2, label='between orders')
    for st in layout['stations']:
        x, y, _ = st['pose']
        ax.plot(x, y, 's', color='black', ms=6)
        ax.annotate(st['name'], (x, y), xytext=(4, -11), textcoords='offset points', fontsize=8)
    dx, dy, _ = layout['docks'][0]['pose']
    ax.plot(dx, dy, '*', color='green', ms=16, label='charging dock')
    ax.set_xlim(-0.5, 24.5)
    ax.set_ylim(-0.5, 16.5)
    ax.set_aspect('equal')
    ax.legend(loc='upper center', ncol=6, fontsize=8, bbox_to_anchor=(0.5, -0.06))
    ax.set_title('Recorded Gazebo run: path driven per order (AMCL pose), red = keep-out, '
                 'yellow = slow zones, orange = where the moving obstacles went', fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'run_trajectory.png'))
    plt.close(fig)

    # ---------------------------------------------------------------- timeline
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(12, 7), dpi=110, sharex=True,
                                     gridspec_kw={'height_ratios': [1.2, 1, 0.8]})
    if battery:
        bt, bp = zip(*battery)
        a1.plot([(t - t0) / 60 for t in bt], [p * 100 for p in bp], color='green')
    a1.axhline(30, color='red', ls='--', lw=1)
    a1.text(0.1, 31, 'charge before the next order below 30 %', color='red', fontsize=8)
    a1.set_ylabel('battery %')
    a1.set_ylim(-5, 105)
    if speed:
        st_, sv = zip(*speed)
        a2.plot([(t - t0) / 60 for t in st_], sv, color='#1f77b4', lw=0.6)
    a2.set_ylabel('speed m/s')
    # mission phases as coloured bands
    states = [(t, oid, s) for t, oid, ev, s in events if ev == 'state' and s]
    end = max([e[0] for e in events] + [battery[-1][0] if battery else t0])
    for i, (t, oid, s) in enumerate(states):
        t_next = states[i + 1][0] if i + 1 < len(states) else end
        a3.axvspan((t - t0) / 60, (t_next - t0) / 60, color=PHASE_COLORS.get(s, '#dddddd'))
    for oid, ts, te in spans:
        a3.text((ts - t0) / 60 + 0.05, 0.5, oid, fontsize=8, va='center')
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in PHASE_COLORS.values()]
    a3.legend(handles, [k.lower().replace('_', ' ') for k in PHASE_COLORS], ncol=5, fontsize=7,
              loc='upper center', bbox_to_anchor=(0.5, -0.45))
    a3.set_yticks([])
    a3.set_ylabel('mission')
    a3.set_xlabel('minutes')
    a1.set_title('Recorded Gazebo run: battery, speed (slower in the slow zones and near obstacles) '
                 'and what the mission was doing', fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'run_timeline.png'))
    plt.close(fig)
    print('wrote', out_dir)


if __name__ == '__main__':
    main(os.path.expanduser(sys.argv[1]),
         sys.argv[2] if len(sys.argv) > 2 else os.path.join(REPO, 'docs', 'images'))
