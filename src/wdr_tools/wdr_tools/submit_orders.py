"""Submits delivery orders to the task manager and follows them to completion.

    ros2 run wdr_tools submit_orders shelf_a1:pack_1 shelf_b2:shipping
    ros2 run wdr_tools submit_orders --random 5 --seed 7
    ros2 run wdr_tools submit_orders shelf_c1:pack_2 --priority 5 --no-wait

All orders are sent at once (the task manager queues them). Prints every state change and the
final result of each order; the exit code is the number of orders that did not succeed, so
scripts and tests can use it directly.
"""

import argparse
import random
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from wdr_interfaces.action import DeliverOrder

PICKUPS = ['shelf_a1', 'shelf_a2', 'shelf_b1', 'shelf_b2', 'shelf_c1', 'shelf_c2']
DROPOFFS = ['pack_1', 'pack_2', 'shipping']
STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}


def parse_orders(args):
    orders = []
    for spec in args.orders:
        if ':' not in spec:
            raise SystemExit(f'order "{spec}" must look like pickup:dropoff, e.g. shelf_a1:pack_1')
        pickup, dropoff = spec.split(':', 1)
        orders.append((pickup, dropoff))
    rng = random.Random(args.seed)
    for _ in range(args.random):
        orders.append((rng.choice(PICKUPS), rng.choice(DROPOFFS)))
    if not orders:
        raise SystemExit('no orders given (use pickup:dropoff arguments or --random N)')
    return orders


class OrderClient(Node):

    def __init__(self):
        super().__init__('submit_orders')
        self.client = ActionClient(self, DeliverOrder, '/deliver_order')
        self.last_state = {}

    def feedback(self, label, msg):
        fb = msg.feedback
        if self.last_state.get(label) != fb.state:
            self.last_state[label] = fb.state
            print(f'  {fb.order_id or label}: {fb.state} (battery {fb.battery_percentage * 100:.0f}%)',
                  flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Submit DeliverOrder goals.')
    parser.add_argument('orders', nargs='*', help='pickup:dropoff pairs')
    parser.add_argument('--random', type=int, default=0, help='add N random orders')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--priority', type=int, default=0)
    parser.add_argument('--no-wait', action='store_true', help='exit once the orders are queued')
    parser.add_argument('--timeout', type=float, default=3600.0, help='seconds to wait in total')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])
    orders = parse_orders(args)

    rclpy.init(args=argv)
    node = OrderClient()
    if not node.client.wait_for_server(timeout_sec=60.0):
        print('the /deliver_order action server is not available', file=sys.stderr)
        return 100

    pending = []
    for i, (pickup, dropoff) in enumerate(orders):
        goal = DeliverOrder.Goal(pickup_station=pickup, dropoff_station=dropoff,
                                 priority=args.priority)
        label = f'#{i + 1} {pickup}->{dropoff}'
        future = node.client.send_goal_async(
            goal, feedback_callback=lambda msg, label=label: node.feedback(label, msg))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            print(f'{label}: REJECTED (see the task manager log for the reason)', flush=True)
            pending.append((label, None))
            continue
        print(f'{label}: queued', flush=True)
        pending.append((label, handle.get_result_async()))

    if args.no_wait:
        return sum(1 for _, f in pending if f is None)

    failures = 0
    deadline = time.monotonic() + args.timeout
    for label, future in pending:
        if future is None:
            failures += 1
            continue
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        if not future.done():
            print(f'{label}: no result within the timeout', flush=True)
            failures += 1
            continue
        res = future.result()
        r = res.result
        status = STATUS_NAMES.get(res.status, str(res.status))
        if res.status == GoalStatus.STATUS_SUCCEEDED and r.success:
            print(f'{label}: {r.order_id} {status} in {r.duration_s:.1f} s, {r.distance_m:.1f} m, '
                  f'{r.retries} retries (queued {r.queue_time_s:.1f} s)', flush=True)
        else:
            failures += 1
            print(f'{label}: {r.order_id} {status} code={r.error_code} {r.error_msg}', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return failures


if __name__ == '__main__':
    sys.exit(main())
