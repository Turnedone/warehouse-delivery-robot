"""Gazebo Harmonic warehouse with a TurtleBot 4, the ROS <-> Gazebo bridge and moving obstacles.

Runs the Gazebo server only (no window) unless gui:=true.
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _gz_server(context):
    world = LaunchConfiguration('world').perform(context)
    args = ['gz', 'sim', '-r', '-s', world]
    if LaunchConfiguration('headless_rendering').perform(context).lower() == 'true':
        # EGL rendering without a display, needed for the gpu_lidar on machines with no X/Wayland
        args.insert(3, '--headless-rendering')
    return [ExecuteProcess(cmd=args, output='screen', name='gz_server')]


def generate_launch_description():
    bringup = get_package_share_directory('wdr_bringup')
    tb4_desc = get_package_share_directory('nav2_minimal_tb4_description')
    use_sim_time = {'use_sim_time': True}

    with open(os.path.join(bringup, 'config', 'warehouse_layout.yaml')) as f:
        spawn_x, spawn_y, spawn_yaw = yaml.safe_load(f)['robot_spawn']

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(bringup, 'urdf', 'wdr_tb4.urdf.xacro'),
                 ' camera:=', LaunchConfiguration('camera')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(bringup, 'worlds',
                                                                  'warehouse.sdf')),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Open the Gazebo client window'),
        DeclareLaunchArgument('headless_rendering', default_value='false',
                              description='Render sensors with EGL (no display needed)'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Simulate the OAK-D camera (not used by the stack)'),
        DeclareLaunchArgument('moving_obstacles', default_value='true'),

        # Gazebo resolves package://nav2_minimal_tb4_description meshes through this path
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(Path(tb4_desc).parent.resolve())),
        # Keep Gazebo off the network: everything in our world is local
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),

        OpaqueFunction(function=_gz_server),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen', name='gz_gui',
                       condition=IfCondition(LaunchConfiguration('gui'))),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[use_sim_time, {'robot_description': robot_description}],
             output='screen'),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'turtlebot4', '-topic', 'robot_description',
                        '-x', str(spawn_x), '-y', str(spawn_y), '-z', '0.01',
                        '-Y', str(spawn_yaw)],
             parameters=[use_sim_time]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[use_sim_time,
                         {'config_file': os.path.join(bringup, 'config', 'gz_bridge.yaml')}],
             output='screen'),

        Node(package='wdr_sim', executable='obstacle_mover', output='screen',
             parameters=[use_sim_time,
                         {'obstacles_file': os.path.join(bringup, 'config', 'obstacles.yaml')}],
             condition=IfCondition(LaunchConfiguration('moving_obstacles'))),
    ])

