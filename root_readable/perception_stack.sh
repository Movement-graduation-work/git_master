#!/usr/bin/env bash
set -e

PIDFILE="/tmp/perception_stack.pids"
ARGFILE="/tmp/perception_stack.args"

export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash

ZONE_NODE="/root/person_state_zone_node.py"
SUMMARY_NODE="/root/perception_summary_node.py"
AVOID_NODE="/root/avoid_by_zone_node.py"

DEFAULT_CONF="0.40"
DEFAULT_ON="2"
DEFAULT_OFF="2"
DEFAULT_WIN="4"

is_alive() { local pid="$1"; kill -0 "$pid" >/dev/null 2>&1; }

start() {
  # (1) 이미 실행 중이면 중복 방지
  if [[ -f "$PIDFILE" ]]; then
    read -r pz ps pa < "$PIDFILE" || true
    if [[ -n "${pz:-}" ]] && is_alive "$pz"; then
      echo "[RUNNING] already started"
      echo " - zone    pid=$pz"
      echo " - summary pid=${ps:-NA}"
      echo " - avoid   pid=${pa:-NA}"
      [[ -f "$ARGFILE" ]] && echo " - last settings: $(cat "$ARGFILE")"
      exit 0
    fi
    rm -f "$PIDFILE" "$ARGFILE"
  fi

  # (2) 옵션: start [conf] [on] [off] [win]
  local conf="${2:-$DEFAULT_CONF}"
  local on="${3:-$DEFAULT_ON}"
  local off="${4:-$DEFAULT_OFF}"
  local win="${5:-$DEFAULT_WIN}"

  local ZONE_ARGS=(--ros-args -p conf_threshold:="$conf" -p on_count:="$on" -p off_streak:="$off" -p win_size:="$win")

  # (3) zone
  python3 "$ZONE_NODE" "${ZONE_ARGS[@]}" >/tmp/person_state_zone_node.log 2>&1 &
  local ZPID=$!

  # (4) summary
  python3 "$SUMMARY_NODE" >/tmp/perception_summary_node.log 2>&1 &
  local SPID=$!

  # (5) avoid (cmd_vel)
  python3 "$AVOID_NODE" >/tmp/avoid_by_zone_node.log 2>&1 &
  local APID=$!

  echo "$ZPID $SPID $APID" > "$PIDFILE"
  echo "conf=$conf on=$on off=$off win=$win" > "$ARGFILE"

  echo "[OK] started"
  echo " - settings: conf=$conf on=$on off=$off win=$win"
  echo " - zone    pid=$ZPID  log=/tmp/person_state_zone_node.log"
  echo " - summary pid=$SPID  log=/tmp/perception_summary_node.log"
  echo " - avoid   pid=$APID  log=/tmp/avoid_by_zone_node.log"
  echo " - view: ros2 topic echo /perception/summary"
  echo " - cmd : ros2 topic echo /cmd_vel"
}

stop() {
  [[ ! -f "$PIDFILE" ]] && { echo "[NOT RUNNING]"; exit 0; }

  read -r ZPID SPID APID < "$PIDFILE" || true

  [[ -n "${APID:-}" ]] && is_alive "$APID" && kill "$APID" >/dev/null 2>&1 || true
  [[ -n "${SPID:-}" ]] && is_alive "$SPID" && kill "$SPID" >/dev/null 2>&1 || true
  [[ -n "${ZPID:-}" ]] && is_alive "$ZPID" && kill "$ZPID" >/dev/null 2>&1 || true

  rm -f "$PIDFILE" "$ARGFILE"
  echo "[OK] stopped"
}

status() {
  [[ ! -f "$PIDFILE" ]] && { echo "[NOT RUNNING]"; exit 0; }

  read -r ZPID SPID APID < "$PIDFILE" || true
  echo "[STATUS]"
  echo " - zone    pid=${ZPID:-NA} alive=$(is_alive "${ZPID:-0}" && echo yes || echo no)"
  echo " - summary pid=${SPID:-NA} alive=$(is_alive "${SPID:-0}" && echo yes || echo no)"
  echo " - avoid   pid=${APID:-NA} alive=$(is_alive "${APID:-0}" && echo yes || echo no)"
  [[ -f "$ARGFILE" ]] && echo " - settings: $(cat "$ARGFILE")"
}

usage() {
  echo "usage:"
  echo "  $0 start [conf] [on] [off] [win]"
  echo "  $0 stop"
  echo "  $0 status"
  echo "examples:"
  echo "  $0 start"
  echo "  $0 start 0.35 1 6 4"
}

case "${1:-}" in
  start)  start "$@" ;;
  stop)   stop ;;
  status) status ;;
  *)      usage; exit 1 ;;
esac
