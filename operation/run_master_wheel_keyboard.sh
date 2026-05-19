#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MASTER_DIR="${MASTER_DIR:-$SCRIPT_DIR}"
SLAVE_IP="${SLAVE_IP:-192.168.0.140}"
SLAVE_PORT="${SLAVE_PORT:-5005}"
RIGHT_PORT="${RIGHT_PORT:-/dev/ttyUSB2}"
LEFT_PORT="${LEFT_PORT:-/dev/ttyUSB1}"

pkill -f "[m]d400t_cmdvel_bridge.py" 2>/dev/null || true
pkill -f "[m]d400t_usb2_cmdvel_bridge.py" 2>/dev/null || true
pkill -f "[c]md_vel_udp_sender.py" 2>/dev/null || true

python3 "$MASTER_DIR/md400t_usb2_cmdvel_bridge.py" \
  --ros-args \
  -p right_port:="$RIGHT_PORT" \
  -p left_port:="$LEFT_PORT" \
  -p baudrate:=57600 \
  -p dev_id:=1 \
  -p right_cmd_id:=0x82 \
  -p left_cmd_id:=0x82 \
  -p wheel_base:=0.32 \
  -p speed_scale:=400.0 \
  -p max_speed_cmd:=300 \
  -p min_effective_cmd:=80 \
  -p send_hz:=50.0 \
  > /tmp/master_md400t_bridge.log 2>&1 &
BRIDGE_PID=$!

python3 "$MASTER_DIR/cmd_vel_udp_sender.py" \
  --ros-args \
  -p slave_ip:="$SLAVE_IP" \
  -p slave_port:="$SLAVE_PORT" \
  -p wheel_base:=0.32 \
  -p speed_scale:=400.0 \
  -p max_speed_cmd:=300 \
  -p min_effective_cmd:=80 \
  > /tmp/cmd_vel_udp_sender.log 2>&1 &
UDP_PID=$!

cleanup() {
  python3 - <<'PY' >/dev/null 2>&1 || true
import rclpy
import time
from geometry_msgs.msg import Twist
rclpy.init()
node = rclpy.create_node('stop_on_exit_once')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
time.sleep(0.2)
msg = Twist()
for _ in range(5):
    pub.publish(msg)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
  kill "$UDP_PID" >/dev/null 2>&1 || true
  kill "$BRIDGE_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 2

if ! kill -0 "$BRIDGE_PID" >/dev/null 2>&1; then
  echo 'master USB2 md400t bridge failed to start'
  cat /tmp/master_md400t_bridge.log
  exit 1
fi

if ! kill -0 "$UDP_PID" >/dev/null 2>&1; then
  echo 'cmd_vel UDP sender failed to start'
  cat /tmp/cmd_vel_udp_sender.log
  exit 1s
fi

echo "[OK] master wheels enabled right=${RIGHT_PORT}, left=${LEFT_PORT}"
echo "[OK] /cmd_vel UDP sender -> ${SLAVE_IP}:${SLAVE_PORT}"
echo "[LOG] master bridge: /tmp/master_md400t_bridge.log"
echo "[LOG] slave UDP tx : /tmp/cmd_vel_udp_sender.log"
echo ""

python3 "$MASTER_DIR/pretty_master_wheel_teleop.py"