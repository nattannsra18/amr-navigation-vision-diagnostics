#!/usr/bin/env bash
set -euo pipefail

ROBOT_HOST="odroid@100.67.28.7"
REMOTE_ENV="source /opt/ros/jazzy/setup.bash; source /home/odroid/amr_ws/install/setup.bash"

remote_terminal() {
  local title="$1"
  local command="$2"
  gnome-terminal --title="$title" -- bash -lc \
    "ssh -tt ${ROBOT_HOST} \"${REMOTE_ENV}; ${command}\"; exec bash"
}

case "${1:-station}" in
  core)
    ssh -tt "${ROBOT_HOST}" \
      "${REMOTE_ENV}; ros2 launch amr_base_driver mapping.launch.py enable_motors:=true"
    ;;
  teleop)
    ssh -tt "${ROBOT_HOST}" \
      "${REMOTE_ENV}; ros2 run amr_base_driver safe_keyboard_teleop --ros-args -p linear_speed:=0.08 -p angular_speed:=0.45 -p key_timeout:=0.35"
    ;;
  sensors)
    ssh -tt "${ROBOT_HOST}" \
      "${REMOTE_ENV}; ros2 run amr_base_driver sensor_dashboard"
    ;;
  stop)
    ssh "${ROBOT_HOST}" \
      "${REMOTE_ENV}; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'"
    ;;
  shutdown)
    "$0" stop || true
    ssh "${ROBOT_HOST}" \
      "pkill -INT -f '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch amr_base_driver mapping.launch.py' || true; pkill -INT -f '^/usr/bin/python3 .*/amr_base_driver/safe_keyboard_teleop' || true; pkill -INT -f '^/usr/bin/python3 .*/amr_base_driver/sensor_dashboard' || true"
    ;;
  station)
    remote_terminal "AMR Core - motors enabled" \
      "ros2 launch amr_base_driver mapping.launch.py enable_motors:=true"
    sleep 5
    remote_terminal "AMR Keyboard Control" \
      "ros2 run amr_base_driver safe_keyboard_teleop --ros-args -p linear_speed:=0.08 -p angular_speed:=0.45 -p key_timeout:=0.35"
    remote_terminal "AMR Sensor Dashboard" \
      "ros2 run amr_base_driver sensor_dashboard"
    ;;
  *)
    echo "Usage: $0 {station|core|teleop|sensors|stop|shutdown}" >&2
    exit 2
    ;;
esac
