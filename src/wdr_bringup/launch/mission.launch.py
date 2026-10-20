"""Mission layer: the task manager (orders, behavior tree) and the simulated battery."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    bringup = get_package_share_directory('wdr_bringup')
    mission = get_package_share_directory('wdr_mission')
    with open(os.path.join(bringup, 'config', 'docks.yaml')) as f:
        docks = yaml.safe_load(f)['docks']
    dock_x, dock_y, _ = next(iter(docks.values()))['pose']

    def arg(name):
        return float(LaunchConfiguration(name).perform(context))

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    return [
        Node(package='wdr_mission', executable='task_manager', name='task_manager',
             output='screen',
             parameters=[os.path.join(mission, 'config', 'task_manager.yaml'), {
                 'use_sim_time': use_sim_time,
                 'stations_file': os.path.join(bringup, 'config', 'stations.yaml'),
                 'behavior_tree': os.path.join(mission, 'behavior_trees', 'deliver_order.xml'),
                 'operator_auto_confirm_s': arg('operator_auto_confirm_s'),
                 # the order energy estimate must know how fast the simulated battery drains
                 'energy.scale': arg('battery_drain_scale'),
             }]),
        Node(package='wdr_sim', executable='battery_sim', name='battery_sim', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'dock_x': float(dock_x), 'dock_y': float(dock_y),
                          'drain_scale': arg('battery_drain_scale'),
                          'charge_scale': arg('battery_charge_scale'),
                          'initial_percentage': arg('initial_battery')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('battery_drain_scale', default_value='1.0'),
        DeclareLaunchArgument('battery_charge_scale', default_value='1.0'),
        DeclareLaunchArgument('initial_battery', default_value='1.0'),
        DeclareLaunchArgument('operator_auto_confirm_s', default_value='3.0'),
        OpaqueFunction(function=launch_setup),
    ])
