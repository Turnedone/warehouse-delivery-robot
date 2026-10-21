#!/usr/bin/env bash
# One-command demo: needs only Docker (no ROS install).
#   ./run-demo.sh
# Builds the image (first time ~10-15 min, then cached), starts the whole stack headless on
# Nav2's loopback simulator, delivers three orders (one after a charge stop), and prints the
# metrics report. Output (report, launch log, recorded bag) goes to ./demo-output.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p demo-output

echo "== building the image (cached after the first run)"
docker build -f docker/Dockerfile -t warehouse-delivery-robot:demo .

echo "== running the demo (about 5 minutes)"
docker run --rm -v "$PWD/demo-output:/out" warehouse-delivery-robot:demo \
  bash /ws/src/wdr_bringup/scripts/demo_headless.sh /out

echo
echo "Done. Report: demo-output/report.txt  ·  bag: demo-output/bag  ·  log: demo-output/launch.log"
