"""End-to-end delivery in Gazebo (headless server, AMCL, moving obstacles).

Off by default because it needs a renderer for the gpu_lidar and takes several minutes;
enable with:  colcon build --cmake-args -DWDR_GAZEBO_TESTS=ON  and run colcon test as usual.
On a machine without a display add headless_rendering:=true (EGL).
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


@pytest.mark.launch_test
def generate_test_description():
    bringup = get_package_share_directory('wdr_bringup')
    stack = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'warehouse.launch.py')),
        launch_arguments={
            'sim': 'gazebo',
            'headless_rendering': os.environ.get('WDR_HEADLESS_RENDERING', 'false'),
            'operator_auto_confirm_s': '1.0',
        }.items())
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


class TestDeliveryGazebo(unittest.TestCase):

    def test_order_is_delivered_in_gazebo(self):
        rclpy.init()
        node = rclpy.create_node('test_delivery_gazebo')
        try:
            client = ActionClient(node, DeliverOrder, '/deliver_order')
            self.assertTrue(client.wait_for_server(timeout_sec=180.0))
            goal = client.send_goal_async(DeliverOrder.Goal(pickup_station='shelf_b2',
                                                            dropoff_station='pack_1'))
            rclpy.spin_until_future_complete(node, goal, timeout_sec=60.0)
            self.assertTrue(goal.result().accepted)
            result = goal.result().get_result_async()
            deadline = time.monotonic() + 900.0
            while not result.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.5)
            self.assertTrue(result.done(), 'no result within 15 minutes')
            self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED,
                             result.result().result.error_msg)
        finally:
            node.destroy_node()
            rclpy.shutdown()
