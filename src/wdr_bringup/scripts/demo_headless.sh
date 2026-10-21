#!/usr/bin/env bash
# Headless end-to-end demo, run inside the Docker image by run-demo.sh / run-demo.bat (also
# works on any machine with the workspace built). Starts the stack on Nav2's loopback
# simulator, records a bag, delivers three orders (battery starts at 50 % with fast drain, so
# the robot also stops to charge), prints the metrics report and saves it.
#   bash demo_headless.sh <output dir>
set -uo pipefail
out="${1:-$HOME/wdr_demo}"
mkdir -p "$out"
rm -rf "$out/bag"
set +u
source /opt/ros/jazzy/setup.bash
# The workspace: /ws in the Docker image, ~/wdr_ws when built with tools/wsl/build.sh
for ws in /ws/install "$HOME/wdr_ws/install"; do
    if [ -f "$ws/setup.bash" ]; then source "$ws/setup.bash"; break; fi
done
if ! ros2 pkg prefix wdr_tools >/dev/null 2>&1; then
    echo "the workspace is not built: run 'bash tools/wsl/build.sh' first" >&2
    exit 2
fi
set -u

echo "starting the robot stack (loopback simulator, headless)..."
# Job control: the launch gets its own process group and keeps SIGINT (a background job of a
# non-interactive shell would otherwise start with SIGINT ignored), so it can be stopped like Ctrl+C.
set -m
ros2 launch wdr_bringup warehouse.launch.py sim:=loopback record:=true bag_dir:="$out/bag" \
    operator_auto_confirm_s:=1.0 initial_battery:=0.5 battery_drain_scale:=15 \
    battery_charge_scale:=40 > "$out/launch.log" 2>&1 &
launch_pid=$!

ros2 run wdr_tools submit_orders shelf_b2:pack_1 shelf_a1:shipping shelf_c2:pack_2 --timeout 1200
failed=$?

echo "stopping the stack..."
# SIGINT to the whole group, as Ctrl+C does: every node shuts down and the bag recorder closes its file.
kill -INT -- "-$launch_pid" 2>/dev/null
for _ in $(seq 1 30); do
    kill -0 "$launch_pid" 2>/dev/null || break
    sleep 1
done
if kill -0 "$launch_pid" 2>/dev/null; then
    echo "the stack did not stop within 30 s; killing it"
    kill -KILL -- "-$launch_pid" 2>/dev/null
fi
wait "$launch_pid" 2>/dev/null

echo
ros2 run wdr_tools metrics "$out/bag" | tee "$out/report.txt"
if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    echo "could not read the recorded bag; see $out/launch.log"
    exit 1
fi
if [ "$failed" -ne 0 ]; then
    echo "$failed order(s) did not succeed; see $out/launch.log"
    exit 1
fi
