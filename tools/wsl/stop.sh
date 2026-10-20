#!/usr/bin/env bash
# Stops anything left over from a run (Gazebo, Nav2, RViz, bridge, our nodes, bag recorder).
for pattern in "gz sim" "ruby.*gz" rviz2 component_container parameter_bridge robot_state_publisher \
               map_server amcl lifecycle_manager controller_server planner_server bt_navigator \
               behavior_server smoother_server velocity_smoother collision_monitor waypoint_follower \
               route_server opennav_docking costmap_filter_info loopback_simulator \
               task_manager battery_sim obstacle_mover initial_pose_publisher "ros2 bag record" \
               "ros2 launch wdr_bringup"; do
    pkill -f "$pattern" 2>/dev/null
done
sleep 2
for pattern in "gz sim" component_container "ros2 launch wdr_bringup"; do
    pkill -9 -f "$pattern" 2>/dev/null
done
echo "stopped"
