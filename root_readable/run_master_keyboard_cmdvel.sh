#!/usr/bin/env bash
set -euo pipefail

# Run this on the master machine after setup_master_keyboard.sh.
# It opens:
#   1) /cmd_vel -> UDP sender to slave 192.168.0.140:5005
#   2) teleop_twist_keyboard to publish /cmd_vel from the keyboard

MASTER_DIR="${MASTER_DIR:-$HOME/root_readable}"
SLAVE_IP="${SLAVE_IP:-192.168.0.140}"
SLAVE_PORT="${SLAVE_PORT:-5005}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

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
export ROS_DOMAIN_ID

if [[ ! -f "$MASTER_DIR/cmd_vel_udp_sender.py" ]]; then
  echo "[ERROR] missing $MASTER_DIR/cmd_vel_udp_sender.py"
  echo "Copy git_master/root_readable/cmd_vel_udp_sender.py to $MASTER_DIR first."
  exit 1
fi

cleanup() {
  if [[ -n "${SENDER_PID:-}" ]]; then
    kill "$SENDER_PID" >/dev/null 2>&1 || true
    wait "$SENDER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

python3 "$MASTER_DIR/cmd_vel_udp_sender.py" \
  --ros-args \
  -p slave_ip:="$SLAVE_IP" \
  -p slave_port:="$SLAVE_PORT" &
SENDER_PID=$!

sleep 1

echo "[OK] UDP sender pid=$SENDER_PID -> $SLAVE_IP:$SLAVE_PORT"
echo "[INFO] Starting keyboard teleop. Keep this terminal focused."
echo "[INFO] Use i/j/k/l/etc. Space or k stops, Ctrl-C exits."

ros2 run teleop_twist_keyboard teleop_twist_keyboard
