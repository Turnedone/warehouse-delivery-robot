"""End-to-end mission test on Nav2's loopback simulator (headless, no Gazebo, runs in CI).

Brings up the whole stack (robot description, loopback simulator, Nav2 with costmap filters
and the docking server, task manager, battery simulator) and checks, in order:
  1. an order naming an unknown station is rejected
  2. a queued order can be canceled before it starts
  3. an order is delivered end to end (undock -> pick-up -> load -> drop-off -> unload)
  4. with a low battery the robot docks, charges, undocks and then delivers
"""

import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.action import ActionClient
from wdr_interfaces.action import DeliverOrder
from wdr_interfaces.msg import OrderEvent
from wdr_interfaces.srv import SetBatteryLevel

ORDER_TIMEOUT_S = 400.0


@pytest.mark.launch_test
def generate_test_description():
    bringup = get_package_share_directory('wdr_bringup')
    stack = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'warehouse.launch.py')),
        launch_arguments={
            'sim': 'loopback',
            'rviz': 'false',
            'operator_auto_confirm_s': '1.0',
            'battery_charge_scale': '40.0',   # charge 20% -> 90% in about a minute
            'log_level': 'info',
        }.items())
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


class TestDeliveryLoopback(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_delivery_loopback')
        cls.client = ActionClient(cls.node, DeliverOrder, '/deliver_order')
        cls.events = []
        cls.node.create_subscription(OrderEvent, '/wdr/order_events',
                                     lambda m: cls.events.append(m), 100)
        cls.battery = cls.node.create_client(SetBatteryLevel, '/battery_sim/set_level')
        assert cls.client.wait_for_server(timeout_sec=120.0), 'no /deliver_order server'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # -------------------------------------------------------------- helpers

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def send(self, pickup, dropoff, order_id=''):
        future = self.client.send_goal_async(DeliverOrder.Goal(
            order_id=order_id, pickup_station=pickup, dropoff_station=dropoff))
        self.assertTrue(self.spin_until(future.done, 30.0), 'no goal response')
        return future.result()

    def result_of(self, handle, timeout=ORDER_TIMEOUT_S):
        future = handle.get_result_async()
        self.assertTrue(self.spin_until(future.done, timeout), 'order did not finish in time')
        return future.result()

    def states_of(self, order_id):
        return [e.state for e in self.events if e.order_id == order_id and e.event == 'state']

    # -------------------------------------------------------------- tests (run in name order)

    def test_1_unknown_station_is_rejected(self):
        handle = self.send('shelf_zz', 'pack_1')
        self.assertFalse(handle.accepted)
        handle = self.send('pack_1', 'shelf_a1')  # pickup and dropoff swapped
        self.assertFalse(handle.accepted)

    def test_2_queued_order_can_be_canceled(self):
        first = self.send('shelf_c1', 'shipping', order_id='T2-FIRST')
        second = self.send('shelf_a2', 'pack_2', order_id='T2-SECOND')
        self.assertTrue(first.accepted and second.accepted)
        cancel = second.cancel_goal_async()
        self.assertTrue(self.spin_until(cancel.done, 10.0))
        res = self.result_of(second, timeout=30.0)
        self.assertEqual(res.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(res.result.error_code, DeliverOrder.Result.CANCELED)
        # The first order is still delivered
        res = self.result_of(first)
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED, res.result.error_msg)

    def test_3_order_is_delivered_end_to_end(self):
        handle = self.send('shelf_b2', 'pack_1', order_id='T3')
        self.assertTrue(handle.accepted)
        res = self.result_of(handle)
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED, res.result.error_msg)
        self.assertTrue(res.result.success)
        self.assertGreater(res.result.distance_m, 5.0)
        states = self.states_of('T3')
        for expected in ('TO_PICKUP', 'WAITING_FOR_LOAD', 'TO_DROPOFF', 'WAITING_FOR_UNLOAD',
                         'DONE'):
            self.assertIn(expected, states)
        self.assertLess(states.index('TO_PICKUP'), states.index('TO_DROPOFF'))

    def test_4_low_battery_docks_and_charges_first(self):
        self.assertTrue(self.battery.wait_for_service(timeout_sec=10.0))
        future = self.battery.call_async(SetBatteryLevel.Request(percentage=0.2))
        self.assertTrue(self.spin_until(future.done, 10.0))
        self.assertTrue(future.result().success)
        self.spin_until(lambda: False, 1.0)  # let the task manager see the new level

        handle = self.send('shelf_a1', 'shipping', order_id='T4')
        self.assertTrue(handle.accepted)
        res = self.result_of(handle)
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED, res.result.error_msg)
        states = self.states_of('T4')
        self.assertIn('GOING_TO_CHARGE', states)
        self.assertIn('CHARGING', states)
        self.assertIn('UNDOCKING', states)
        self.assertLess(states.index('CHARGING'), states.index('TO_PICKUP'))
        charged = [e.battery_percentage for e in self.events
                   if e.order_id == 'T4' and e.state == 'UNDOCKING']
        self.assertGreaterEqual(charged[-1], 0.89)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        # Exit codes from SIGINT/SIGTERM at shutdown are fine; crashes are not. One exception:
        # Nav2's component container in Jazzy can segfault while tearing down (reported through
        # GraphicsMagick's signal handler as "Magick: abort due to signal 11", exit -6), after all
        # work is done. See docs/engineering-log.md. Any other process crashing still fails.
        clean = {0, -2, -15, 2, 130, 143}
        for name in proc_info.process_names():
            allowed = clean | ({-6, -11} if name.startswith('component_container') else set())
            self.assertIn(proc_info[name].returncode, allowed, f'{name} crashed')
