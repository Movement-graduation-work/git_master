#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SLAVE_IP="${SLAVE_IP:-192.168.0.140}"
TELEMETRY_PORT="${TELEMETRY_PORT:-5015}"
SEND_HZ="${SEND_HZ:-5.0}"
PATH_MAX_POINTS="${PATH_MAX_POINTS:-80}"

echo "[INFO] telemetry sender -> ${SLAVE_IP}:${TELEMETRY_PORT}"
echo "[INFO] topics: /cmd_vel, /odom, /robot_path"

python3 "$SCRIPT_DIR/cmd_odom_path_udp_sender.py" \
  --ros-args \
  -p target_ip:="$SLAVE_IP" \
  -p target_port:="$TELEMETRY_PORT" \
  -p send_hz:="$SEND_HZ" \
  -p path_max_points:="$PATH_MAX_POINTS"
