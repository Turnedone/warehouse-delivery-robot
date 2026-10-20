"""Brings up the whole warehouse delivery robot.

    ros2 launch wdr_bringup warehouse.launch.py                  Gazebo, Nav2, mission layer
    ros2 launch wdr_bringup warehouse.launch.py sim:=loopback    Nav2's loopback simulator instead
                                                                 of Gazebo (fast, no rendering)
    ... rviz:=true      open RViz          ... gui:=true      open the Gazebo window
    ... record:=true    record a rosbag (MCAP) into bag_dir for the metrics report

Nothing opens a window unless asked: the defaults are headless.
"""

import datetime
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Topics recorded for the metrics report and for replaying a run in RViz/Foxglove
RECORDED_TOPICS = [
    '/wdr/order_events', '/battery_state', '/odom', '/scan', '/tf', '/tf_static', '/cmd_vel',
    '/speed_limit', '/collision_monitor_state', '/plan', '/amcl_pose', '/wdr/obstacles',
    '/navigate_to_pose/_action/status', '/deliver_order/_action/status',
    '/dock_robot/_action/status', '/robot_description', '/map',
]


def launch_setup(context):
    bringup = get_package_share_directory('wdr_bringup')
    launch_dir = os.path.join(bringup, 'launch')
    sim = LaunchConfiguration('sim').perform(context)
    # Gazebo drives a simulation clock (/clock). The loopback simulator runs in real time, so
    # that stack uses wall time: no /clock at all, and so no jump from t=0 to wall time when a
    # late clock publisher starts (that jump once made every Nav2 bond time out at once).
    use_sim_time = sim == 'gazebo'
    sim_time = 'true' if use_sim_time else 'false'
    with open(os.path.join(bringup, 'config', 'warehouse_layout.yaml')) as f:
        spawn_x, spawn_y, spawn_yaw = yaml.safe_load(f)['robot_spawn']

    def include(name, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments={k: v for k, v in args.items()}.items())

    actions = []
    if sim == 'gazebo':
        actions.append(include('sim_gazebo.launch.py',
                               gui=LaunchConfiguration('gui'),
                               headless_rendering=LaunchConfiguration('headless_rendering'),
                               moving_obstacles=LaunchConfiguration('moving_obstacles')))
        actions.append(include('navigation.launch.py', localization='amcl', use_sim_time=sim_time,
                               log_level=LaunchConfiguration('log_level')))
    else:
        # Nav2's loopback simulator: integrates cmd_vel into odometry, publishes map -> odom
        # (perfect localization) and ray-casts a fake scan from the map. No physics, no moving
        # obstacles; enough to test the mission layer end to end in CI.
        robot_description = ParameterValue(
            Command(['xacro ', os.path.join(bringup, 'urdf', 'wdr_tb4.urdf.xacro')]),
            value_type=str)
        actions += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'use_sim_time': False, 'robot_description': robot_description}],
                 output='screen'),
            Node(package='nav2_loopback_sim', executable='loopback_simulator',
                 name='loopback_simulator', output='screen',
                 parameters=[os.path.join(bringup, 'config', 'nav2_params.yaml'),
                             {'use_sim_time': False, 'publish_clock': False}]),
            include('navigation.launch.py', localization='none', use_sim_time=sim_time,
                    log_level=LaunchConfiguration('log_level')),
        ]

    actions.append(Node(package='wdr_bringup', executable='initial_pose_publisher.py',
                        name='initial_pose_publisher', output='screen',
                        parameters=[{'use_sim_time': use_sim_time, 'x': float(spawn_x),
                                     'y': float(spawn_y), 'yaw': float(spawn_yaw)}]))

    actions.append(include('mission.launch.py', use_sim_time=sim_time,
                           battery_drain_scale=LaunchConfiguration('battery_drain_scale'),
                           battery_charge_scale=LaunchConfiguration('battery_charge_scale'),
                           initial_battery=LaunchConfiguration('initial_battery'),
                           operator_auto_confirm_s=LaunchConfiguration('operator_auto_confirm_s')))

    if LaunchConfiguration('record').perform(context).lower() == 'true':
        bag_dir = LaunchConfiguration('bag_dir').perform(context)
        if not bag_dir:
            stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            bag_dir = os.path.join(os.path.expanduser('~'), 'wdr_bags', f'run_{stamp}')
        os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_dir]
            + (['--use-sim-time'] if use_sim_time else []) + RECORDED_TOPICS,
            output='screen', name='bag_record'))

    actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                        arguments=['-d', os.path.join(bringup, 'rviz', 'warehouse.rviz')],
                        parameters=[{'use_sim_time': use_sim_time}],
                        condition=IfCondition(LaunchConfiguration('rviz'))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='gazebo', choices=['gazebo', 'loopback']),
        DeclareLaunchArgument('gui', default_value='false', description='Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz window'),
        DeclareLaunchArgument('headless_rendering', default_value='false'),
        DeclareLaunchArgument('moving_obstacles', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_dir', default_value='',
                              description='default: ~/wdr_bags/run_<timestamp>'),
        DeclareLaunchArgument('battery_drain_scale', default_value='1.0',
                              description='>1 drains the simulated battery faster'),
        DeclareLaunchArgument('battery_charge_scale', default_value='1.0',
                              description='>1 charges the simulated battery faster'),
        DeclareLaunchArgument('initial_battery', default_value='1.0'),
        DeclareLaunchArgument('operator_auto_confirm_s', default_value='3.0',
                              description='simulated operator confirms load/unload after this '
                                          'many seconds; negative = wait for a real confirm'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=launch_setup),
    ])
