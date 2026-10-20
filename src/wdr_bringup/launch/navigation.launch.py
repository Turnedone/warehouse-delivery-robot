"""Nav2 for the warehouse: localization, navigation servers, docking server and costmap filters.

All Nav2 servers run as components in one container process (intra-process communication,
one process to monitor), each group brought up by its own lifecycle manager:

    lifecycle_manager_localization    map_server, amcl            (localization:=amcl)
                                      map_server                  (localization:=none)
    lifecycle_manager_filters         keep-out and speed mask servers + filter info servers
    lifecycle_manager_navigation      controller, planner, smoother, behaviors, bt_navigator,
                                      velocity_smoother, collision_monitor, docking_server

    localization:=amcl   Gazebo: AMCL localizes against the map from the simulated lidar
    localization:=none   loopback simulator: it publishes a perfect map -> odom itself

Compared with nav2_bringup's launch files this leaves out the servers the robot does not use
(route_server, waypoint_follower) and adds the costmap filters.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def write_params(bringup, source, use_sim_time):
    """Fills the '<set by launch>' paths in nav2_params.yaml and returns a temporary copy."""
    with open(source) as f:
        params = yaml.safe_load(f)
    maps = os.path.join(bringup, 'maps')

    def set_param(node, key, value):
        params[node]['ros__parameters'][key] = value

    set_param('docking_server', 'dock_database', os.path.join(bringup, 'config', 'docks.yaml'))
    set_param('keepout_filter_mask_server', 'yaml_filename',
              os.path.join(maps, 'keepout_mask.yaml'))
    set_param('speed_filter_mask_server', 'yaml_filename', os.path.join(maps, 'speed_mask.yaml'))
    params['map_server'] = {'ros__parameters': {
        'yaml_filename': os.path.join(maps, 'warehouse.yaml')}}
    for node in params.values():  # one clock for every Nav2 node
        if isinstance(node, dict) and 'ros__parameters' in node:
            node['ros__parameters']['use_sim_time'] = use_sim_time
        elif isinstance(node, dict):  # costmaps are nested one level deeper
            for inner in node.values():
                if isinstance(inner, dict) and 'ros__parameters' in inner:
                    inner['ros__parameters']['use_sim_time'] = use_sim_time
    out = tempfile.NamedTemporaryFile('w', prefix='wdr_nav2_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, out, sort_keys=False)
    out.close()
    return out.name


def launch_setup(context):
    bringup = get_package_share_directory('wdr_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params = write_params(bringup, LaunchConfiguration('params_file').perform(context),
                          use_sim_time)
    use_amcl = LaunchConfiguration('localization').perform(context) == 'amcl'

    def component(package, plugin, name, remappings=(), extra=None):
        return ComposableNode(package=package, plugin=plugin, name=name,
                              parameters=[params] + ([extra] if extra else []),
                              remappings=list(remappings))

    def manager(name, nodes):
        return ComposableNode(package='nav2_lifecycle_manager',
                              plugin='nav2_lifecycle_manager::LifecycleManager', name=name,
                              parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                                           'node_names': nodes,
                                           'bond_timeout': 10.0}])

    # Localization
    localization = [component('nav2_map_server', 'nav2_map_server::MapServer', 'map_server')]
    localization_names = ['map_server']
    if use_amcl:
        localization.append(component('nav2_amcl', 'nav2_amcl::AmclNode', 'amcl'))
        localization_names.append('amcl')

    # Costmap filter masks
    filters = [
        component('nav2_map_server', 'nav2_map_server::MapServer', 'keepout_filter_mask_server'),
        component('nav2_map_server', 'nav2_map_server::CostmapFilterInfoServer',
                  'keepout_costmap_filter_info_server'),
        component('nav2_map_server', 'nav2_map_server::MapServer', 'speed_filter_mask_server'),
        component('nav2_map_server', 'nav2_map_server::CostmapFilterInfoServer',
                  'speed_costmap_filter_info_server'),
    ]
    filter_names = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server',
                    'speed_filter_mask_server', 'speed_costmap_filter_info_server']

    # Navigation. The command-velocity chain is
    #   controller / behaviors -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
    #   -> collision_monitor -> cmd_vel -> robot
    # The docking server drives the final approach itself and publishes cmd_vel directly.
    navigation = [
        component('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server',
                  remappings=[('cmd_vel', 'cmd_vel_nav')]),
        component('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server'),
        component('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server'),
        component('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server',
                  remappings=[('cmd_vel', 'cmd_vel_nav')]),
        component('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator'),
        component('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother',
                  'velocity_smoother', remappings=[('cmd_vel', 'cmd_vel_nav')]),
        component('nav2_collision_monitor', 'nav2_collision_monitor::CollisionMonitor',
                  'collision_monitor'),
        component('opennav_docking', 'opennav_docking::DockingServer', 'docking_server'),
    ]
    navigation_names = ['controller_server', 'smoother_server', 'planner_server',
                        'behavior_server', 'bt_navigator', 'velocity_smoother',
                        'collision_monitor', 'docking_server']

    container = ComposableNodeContainer(
        name='nav2_container', namespace='', package='rclcpp_components',
        executable='component_container_isolated', output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # The params file also goes to the container process itself: the costmaps are child
        # nodes created inside controller_server/planner_server and read their parameters
        # from the process-wide --params-file, not from their parent component's parameters.
        parameters=[params, {'use_sim_time': use_sim_time, 'autostart': True}],
        composable_node_descriptions=(
            localization + filters + navigation + [
                manager('lifecycle_manager_localization', localization_names),
                manager('lifecycle_manager_filters', filter_names),
                manager('lifecycle_manager_navigation', navigation_names),
            ]))
    return [container]


def generate_launch_description():
    bringup = get_package_share_directory('wdr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(bringup, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('localization', default_value='amcl', choices=['amcl', 'none']),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=launch_setup),
    ])
