#!/usr/bin/env bash
set -euo pipefail

# Run this on the master to send a short, low-speed /cmd_vel test.
# Useful before using keyboard teleop.

LINEAR_X="${LINEAR_X:-0.08}"
ANGULAR_Z="${ANGULAR_Z:-0.0}"
DURATION="${DURATION:-1.0}"

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  echo "[ERROR] ROS2 setup.bash not found under /opt/ros"
  exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[TEST] publishing /cmd_vel linear.x=$LINEAR_X angular.z=$ANGULAR_Z for $DURATION sec"
timeout "$DURATION" ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: $LINEAR_X, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_Z}}" \
  -r 10 || true

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo "[OK] stop command sent"
