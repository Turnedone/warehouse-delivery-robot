#!/usr/bin/env bash
# Visual demo: Gazebo + RViz windows, five random orders, fast battery so a charge stop happens.
#   bash tools/wsl/demo.sh            (stop with tools/wsl/stop.sh)
source /opt/ros/jazzy/setup.bash
source ~/wdr_ws/install/setup.bash
mkdir -p ~/wdr_logs
ros2 launch wdr_bringup warehouse.launch.py rviz:=true gui:=true \
    battery_drain_scale:=15 battery_charge_scale:=20 initial_battery:=0.6 > ~/wdr_logs/demo.log 2>&1 &
sleep 30
ros2 run wdr_tools submit_orders --random 5 --seed 3
