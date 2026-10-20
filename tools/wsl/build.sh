#!/usr/bin/env bash
# Builds (and optionally tests) the workspace in ~/wdr_ws, straight from this repository:
# colcon reads the packages from the Windows checkout, build/install/log stay in Linux.
#   bash tools/wsl/build.sh                  build everything
#   bash tools/wsl/build.sh --test           build, then run all tests
#   bash tools/wsl/build.sh --packages-select wdr_mission   (extra args go to colcon build)
set -eo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
ws="$HOME/wdr_ws"
mkdir -p "$ws"
cd "$ws"
set +u
source /opt/ros/jazzy/setup.bash
run_tests=0
args=()
for a in "$@"; do
    if [ "$a" = "--test" ]; then run_tests=1; else args+=("$a"); fi
done
colcon build --base-paths "$repo/src" --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --event-handlers console_cohesion+ "${args[@]}"
if [ "$run_tests" = 1 ]; then
    colcon test --base-paths "$repo/src" --event-handlers console_cohesion+ --return-code-on-test-failure
    colcon test-result --verbose
fi
echo "Built. Use it with:  source $ws/install/setup.bash"
