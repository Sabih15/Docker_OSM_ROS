#!/usr/bin/env bash
# Container entrypoint: serve the map UI and launch the ROS 2 graph.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# Serve the static Leaflet UI (no extra dependency — stdlib http.server).
( cd /web && python3 -m http.server 8000 ) &
WEB_PID=$!

# Forward SIGTERM/SIGINT so `docker stop` shuts everything down cleanly.
cleanup() { kill "$WEB_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

echo "[entrypoint] web UI on :8000  |  websocket on :9090  |  impl=${IMPL:-py}"
exec ros2 launch teleop_localization_py teleop.launch.py impl:="${IMPL:-py}"
