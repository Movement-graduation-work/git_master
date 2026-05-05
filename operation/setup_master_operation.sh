#!/usr/bin/env bash
set -eo pipefail

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "[ERROR] ROS2 Jazzy setup not found: /opt/ros/jazzy/setup.bash"
  echo "Install ROS2 Jazzy first, then run this script again."
  exit 1
fi

source /opt/ros/jazzy/setup.bash

echo "[INFO] Installing runtime packages"
$SUDO apt update
$SUDO apt install -y python3-serial ros-jazzy-geometry-msgs ros-jazzy-rclpy

if ! groups "$USER" | grep -q dialout; then
  $SUDO usermod -aG dialout "$USER"
  echo "[WARN] Added $USER to dialout. Log out and log back in before using /dev/ttyUSB*."
fi

chmod +x "$(dirname "$0")"/run_master_wheel_keyboard.sh
chmod +x "$(dirname "$0")"/md400t_usb2_cmdvel_bridge.py
chmod +x "$(dirname "$0")"/cmd_vel_udp_sender.py
chmod +x "$(dirname "$0")"/pretty_master_wheel_teleop.py
chmod +x "$(dirname "$0")"/md_port_test_header.py

python3 -m py_compile \
  "$(dirname "$0")"/md400t_usb2_cmdvel_bridge.py \
  "$(dirname "$0")"/cmd_vel_udp_sender.py \
  "$(dirname "$0")"/pretty_master_wheel_teleop.py \
  "$(dirname "$0")"/md_port_test_header.py

echo "[OK] master operation setup complete"
echo "Run:"
echo "  bash $(dirname "$0")/run_master_wheel_keyboard.sh"
