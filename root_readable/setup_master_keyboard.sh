#!/usr/bin/env bash
set -euo pipefail

# Run this on the master Ubuntu/Jetson machine.
# It installs ROS2 Humble basics and the keyboard teleop package used to publish /cmd_vel.

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

if [[ ! -f /etc/os-release ]]; then
  echo "[ERROR] /etc/os-release not found. This script expects Ubuntu on the master."
  exit 1
fi

. /etc/os-release
case "${VERSION_CODENAME:-}" in
  jammy) ROS_DISTRO="${ROS_DISTRO:-humble}" ;;
  noble) ROS_DISTRO="${ROS_DISTRO:-jazzy}" ;;
  *)
    ROS_DISTRO="${ROS_DISTRO:-jazzy}"
    echo "[WARN] Unknown Ubuntu codename ${VERSION_CODENAME:-unknown}; trying ROS2 $ROS_DISTRO."
    ;;
esac

$SUDO apt update
$SUDO apt install -y software-properties-common curl gnupg lsb-release locales
$SUDO locale-gen en_US en_US.UTF-8 || true

$SUDO add-apt-repository universe -y
$SUDO install -d -m 0755 /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | $SUDO tee /etc/apt/keyrings/ros-archive-keyring.gpg >/dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME:-jammy} main" \
  | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null

$SUDO apt update
$SUDO apt install -y \
  ros-${ROS_DISTRO}-ros-base \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip

if ! rosdep --version >/dev/null 2>&1; then
  echo "[WARN] rosdep command not found after install"
else
  $SUDO rosdep init 2>/dev/null || true
  rosdep update || true
fi

if ! grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" "$HOME/.bashrc"; then
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "$HOME/.bashrc"
fi

echo "[OK] master ROS2 $ROS_DISTRO keyboard setup complete"
echo "Next:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  bash ~/root_readable/run_master_keyboard_cmdvel.sh"
