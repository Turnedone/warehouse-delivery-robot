#!/usr/bin/env bash
# Installs this workspace's dependencies with rosdep, on top of the ROS 2 Jazzy + Gazebo Harmonic +
# Nav2 setup from robot-sim's tools/wsl/setup-ros2-gazebo.sh.
# Run as root, e.g. from Windows:
#   wsl -d Ubuntu-24.04 -u root -- bash /mnt/c/path/to/warehouse-delivery-robot/tools/wsl/install-deps.sh
set -eo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
apt-get update
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
rosdep update --rosdistro jazzy
rosdep install --from-paths "$repo/src" --ignore-src -y --rosdistro jazzy
echo "dependencies installed"
