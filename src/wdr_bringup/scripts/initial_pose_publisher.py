#!/usr/bin/env python3
"""Publishes the robot's start pose on /initialpose until localization has picked it up.

What an operator does with RViz's "2D Pose Estimate" tool, done automatically: AMCL (Gazebo
mode) or the loopback simulator (loopback mode) needs it before the map -> odom transform
exists. Re-publishes every 0.5 s until TF map -> base_link is available, then exits.

It first waits until the robot's own static transforms (base_link -> sensor_frame, from
robot_state_publisher) have been available for a second. Nav2's loopback simulator looks up
base_link -> lidar only until it receives the first initial pose; if the pose arrives first,
its simulated lidar stays empty for the whole run.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.x = self.declare_parameter('x', 0.0).value
        self.y = self.declare_parameter('y', 0.0).value
        self.yaw = self.declare_parameter('yaw', 0.0).value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.sensor_frame = self.declare_parameter('sensor_frame', 'rplidar_link').value
        self.sensor_tf_since = None
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.done = False
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        if self.tf_buffer.can_transform('map', self.base_frame, Time(),
                                        timeout=Duration(seconds=0.0)):
            self.get_logger().info('localized: map -> %s is available' % self.base_frame)
            self.done = True
            return
        if self.pub.get_subscription_count() == 0:
            return  # nobody listening yet: a message now would be lost
        if not self.tf_buffer.can_transform(self.base_frame, self.sensor_frame, Time(),
                                            timeout=Duration(seconds=0.0)):
            self.sensor_tf_since = None
            return
        now = self.get_clock().now()
        if self.sensor_tf_since is None:
            self.sensor_tf_since = now
        if now - self.sensor_tf_since < Duration(seconds=1.0):
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.pose.covariance[0] = 0.05 ** 2
        msg.pose.covariance[7] = 0.05 ** 2
        msg.pose.covariance[35] = 0.05 ** 2
        self.pub.publish(msg)
        self.get_logger().info('published initial pose (%.2f, %.2f, %.2f)' %
                               (self.x, self.y, self.yaw))


def main():
    rclpy.init()
    node = InitialPosePublisher()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
