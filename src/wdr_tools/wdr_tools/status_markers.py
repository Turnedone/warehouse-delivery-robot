"""RViz status overlay: who is doing what, as text markers on /wdr/status_markers.

- above the robot: current order, what it is doing in plain words, battery
- above each moving obstacle: its name and the direction it is moving
- at each station: its name
- above the warehouse: a status board with the queue and delivered/failed counts

Reads only what the system already publishes (/wdr/order_events, /battery_state,
/wdr/obstacles, TF), so it can also run against a replayed bag.
"""

import math

import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from wdr_interfaces.msg import OrderEvent

ACTIVITY = {
    'CHECKING_BATTERY': 'checking battery',
    'GOING_TO_CHARGE': 'battery low: driving to the charger',
    'CHARGING': 'charging',
    'UNDOCKING': 'leaving the charger',
    'TO_PICKUP': 'driving to pick-up {pickup}',
    'WAITING_FOR_LOAD': 'waiting to be loaded at {pickup}',
    'TO_DROPOFF': 'delivering to {dropoff}',
    'WAITING_FOR_UNLOAD': 'waiting to be unloaded at {dropoff}',
    'NAV_RETRY': 'path blocked: clearing costmaps and retrying',
    'TO_EXCEPTION_AREA': 'order failed: parking the load at qa_hold',
    'DONE': 'order done',
    'RETURNING_TO_DOCK': 'idle: returning to the charger',
}


def describe(state, pickup='', dropoff=''):
    """Plain-words activity for a mission state."""
    return ACTIVITY.get(state, state.lower().replace('_', ' ') if state else 'idle').format(
        pickup=pickup or '?', dropoff=dropoff or '?')


def heading_word(dx, dy):
    """Compass direction of a movement in the map frame (+y is north), or 'standing'."""
    if math.hypot(dx, dy) < 0.02:
        return 'standing'
    angle = math.degrees(math.atan2(dy, dx)) % 360
    return ['east', 'north', 'west', 'south'][int(((angle + 45) % 360) // 90)]


class StatusTracker:
    """Mission state reconstructed from order events (no ROS here, unit tested)."""

    def __init__(self):
        self.orders = {}       # id -> {'pickup', 'dropoff', 'state'}
        self.queue = []        # queued, not yet started, in arrival order
        self.current = None
        self.idle_state = ''
        self.delivered = 0
        self.failed = 0
        self.canceled = 0

    def on_event(self, order_id, event, state, pickup='', dropoff=''):
        if not order_id:
            self.idle_state = state if event == 'state' else self.idle_state
            return
        rec = self.orders.setdefault(order_id, {'pickup': pickup, 'dropoff': dropoff, 'state': ''})
        rec['pickup'] = pickup or rec['pickup']
        rec['dropoff'] = dropoff or rec['dropoff']
        if event == 'queued' and order_id not in self.queue:
            self.queue.append(order_id)
        elif event == 'started':
            self.queue = [o for o in self.queue if o != order_id]
            self.current = order_id
            self.idle_state = ''
        elif event == 'state':
            rec['state'] = state
        elif event in ('succeeded', 'failed', 'canceled'):
            self.queue = [o for o in self.queue if o != order_id]
            if self.current == order_id:
                self.current = None
            self.delivered += event == 'succeeded'
            self.failed += event == 'failed'
            self.canceled += event == 'canceled'

    def robot_text(self):
        if self.current:
            rec = self.orders[self.current]
            return (f"{self.current}  {rec['pickup']} -> {rec['dropoff']}\n"
                    f"{describe(rec['state'], rec['pickup'], rec['dropoff'])}")
        return describe(self.idle_state) if self.idle_state else 'idle: waiting for orders'

    def board_text(self):
        queue = ', '.join(self.queue[:4]) + (' ...' if len(self.queue) > 4 else '')
        return (f"delivered {self.delivered}   failed {self.failed}   canceled {self.canceled}"
                f"   |   queue ({len(self.queue)}): {queue or 'empty'}")


class StatusMarkers(Node):

    def __init__(self):
        super().__init__('status_markers')
        stations_file = self.declare_parameter('stations_file', '').value
        self.stations = {}
        if stations_file:
            with open(stations_file) as f:
                self.stations = yaml.safe_load(f)['stations']
        self.tracker = StatusTracker()
        self.battery = None
        self.charging = False
        self.obstacles = {}  # name -> (x, y, height, last_x, last_y, heading)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, '/wdr/status_markers', 10)
        self.create_subscription(OrderEvent, '/wdr/order_events', self.on_event, 100)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery, 10)
        self.create_subscription(MarkerArray, '/wdr/obstacles', self.on_obstacles, 10)
        self.create_timer(0.5, self.publish)

    def on_event(self, e):
        self.tracker.on_event(e.order_id, e.event, e.state, e.pickup_station, e.dropoff_station)

    def on_battery(self, msg):
        self.battery = msg.percentage
        self.charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING

    def on_obstacles(self, msg):
        for m in msg.markers:
            x, y = m.pose.position.x, m.pose.position.y
            old = self.obstacles.get(m.text)
            heading = old[5] if old else 'standing'
            if old and math.hypot(x - old[3], y - old[4]) > 0.1:
                heading = heading_word(x - old[3], y - old[4])
                lx, ly = x, y
            else:
                lx, ly = (old[3], old[4]) if old else (x, y)
            self.obstacles[m.text] = (x, y, m.scale.z, lx, ly, heading)

    def text(self, mid, x, y, z, text, size, rgb, ns):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        m.scale.z = size
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 1.0
        m.text = text
        m.lifetime = Duration(seconds=2.0).to_msg()
        return m

    def publish(self):
        out = MarkerArray()
        # Robot
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            rx, ry = t.transform.translation.x, t.transform.translation.y
            batt = ''
            if self.battery is not None:
                batt = f"\nbattery {self.battery * 100:.0f}%{' (charging)' if self.charging else ''}"
            out.markers.append(self.text(0, rx, ry + 0.9, 1.0, 'ROBOT  ' + self.tracker.robot_text()
                                         + batt, 0.38, (1.0, 1.0, 0.3), 'robot'))
        except Exception:  # not localized yet
            pass
        # Moving obstacles
        for i, (name, (x, y, h, _, _, heading)) in enumerate(sorted(self.obstacles.items())):
            label = name.replace('_', ' ')
            out.markers.append(self.text(i, x, y, h + 0.4, f'{label}: {heading}', 0.32,
                                         (1.0, 0.6, 0.2), 'obstacles'))
        # Stations
        for i, (name, st) in enumerate(sorted(self.stations.items())):
            out.markers.append(self.text(i, st['x'], st['y'] - 0.45, 0.2, name, 0.28,
                                         (0.6, 0.85, 1.0), 'stations'))
        # Status board above the north wall
        out.markers.append(self.text(0, 12.0, 17.0, 0.5, self.tracker.board_text(), 0.5,
                                     (1.0, 1.0, 1.0), 'board'))
        self.pub.publish(out)


def main(argv=None):
    rclpy.init(args=argv)
    node = StatusMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
