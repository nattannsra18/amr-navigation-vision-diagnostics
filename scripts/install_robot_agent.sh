#!/usr/bin/env bash
set -Eeuo pipefail

readonly SERVICE_NAME="indoor-delivery-robot-agent.service"
readonly SERVICE_USER="indoor-robot"
readonly CONFIG_DIR="/etc/indoor-delivery-robot"
readonly STATE_DIR="/var/lib/indoor-delivery-robot"
readonly AGENT_WORKSPACE="/opt/indoor-delivery-robot/ros"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PACKAGE_ROOT="$REPOSITORY_ROOT/src/amr_web_bridge"
PROFILE_FILE=""
CONTROL_URL=""
REGISTRY_URL=""
TOKEN_FILE=""
ROS_DISTRO_NAME="jazzy"
DRY_RUN=false
NO_START=false

usage() {
  cat <<'EOF'
Install the Indoor Delivery Robot Agent as a hardened systemd service.

Usage:
  sudo ./scripts/install_robot_agent.sh \
    --profile /path/to/robot-agent.yaml \
    --control-url wss://control.example.com \
    [--registry-url https://control.example.com/robots] \
    [--enrollment-token-file /path/to/token] \
    [--ros-distro jazzy] [--no-start] [--dry-run]

The enrollment token is read from --enrollment-token-file,
ROBOT_ENROLLMENT_TOKEN, or a hidden interactive prompt. Never pass it as a
command-line value. The profile must contain the robot's real unique serial.
EOF
}

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

info() {
  printf '==> %s\n' "$*"
}

run() {
  if "$DRY_RUN"; then
    printf '[dry-run]'
    printf ' %q' "$@"
    printf '\n'
    return 0
  fi
  "$@"
}

while (($#)); do
  case "$1" in
    --profile)
      (($# >= 2)) || fail "--profile requires a path"
      PROFILE_FILE="$2"
      shift 2
      ;;
    --control-url)
      (($# >= 2)) || fail "--control-url requires a URL"
      CONTROL_URL="${2%/}"
      shift 2
      ;;
    --registry-url)
      (($# >= 2)) || fail "--registry-url requires a URL"
      REGISTRY_URL="${2%/}"
      shift 2
      ;;
    --enrollment-token-file)
      (($# >= 2)) || fail "--enrollment-token-file requires a path"
      TOKEN_FILE="$2"
      shift 2
      ;;
    --ros-distro)
      (($# >= 2)) || fail "--ros-distro requires a name"
      ROS_DISTRO_NAME="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --no-start)
      NO_START=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "Unknown option: $1"
      ;;
  esac
done

[[ -n "$PROFILE_FILE" ]] || fail "--profile is required"
[[ -f "$PROFILE_FILE" ]] || fail "Profile does not exist: $PROFILE_FILE"
[[ -r "$PROFILE_FILE" ]] || fail "Profile is not readable: $PROFILE_FILE"
[[ -n "$CONTROL_URL" ]] || fail "--control-url is required"
[[ "$CONTROL_URL" =~ ^wss?://[^[:space:]]+$ ]] || fail "Control URL must use ws:// or wss://"
[[ "$ROS_DISTRO_NAME" =~ ^[a-z0-9_]+$ ]] || fail "Invalid ROS distribution name"
[[ -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]] || fail "ROS setup not found: /opt/ros/$ROS_DISTRO_NAME/setup.bash"
[[ -f "$PACKAGE_ROOT/package.xml" ]] || fail "amr_web_bridge source package was not found"
command -v colcon >/dev/null || fail "colcon is required"
command -v systemctl >/dev/null || fail "systemctl is required"
if ! bash -c "source '/opt/ros/$ROS_DISTRO_NAME/setup.bash' && python3 -c 'import websockets, yaml'"; then
  fail "Python packages websockets and yaml are required by the Agent"
fi
for ROS_PACKAGE in ament_index_python action_msgs diagnostic_msgs geometry_msgs lifecycle_msgs nav2_msgs nav_msgs rclpy std_srvs tf2_ros; do
  if ! bash -c "source '/opt/ros/$ROS_DISTRO_NAME/setup.bash' && ros2 pkg prefix '$ROS_PACKAGE' >/dev/null 2>&1"; then
    fail "Required ROS package is unavailable: $ROS_PACKAGE"
  fi
done

grep -Eq '^[[:space:]]*robot_serial_number:[[:space:]]*[^[:space:]]+' "$PROFILE_FILE" || fail "Profile must define robot_serial_number"
grep -Eq '^[[:space:]]*profile_version:[[:space:]]*[^[:space:]]+' "$PROFILE_FILE" || fail "Profile must define profile_version"
grep -Eq '^[[:space:]]*agent_capabilities:[[:space:]]*[^[:space:]]+' "$PROFILE_FILE" || fail "Profile must define agent_capabilities"
if grep -Eq 'REPLACE-|CHANGE-ME|CHANGEME' "$PROFILE_FILE"; then
  fail "Profile still contains a placeholder value"
fi

if [[ -n "$TOKEN_FILE" ]]; then
  [[ -f "$TOKEN_FILE" && -r "$TOKEN_FILE" ]] || fail "Enrollment token file is not readable"
  ENROLLMENT_TOKEN="$(tr -d '\r\n' < "$TOKEN_FILE")"
else
  ENROLLMENT_TOKEN="${ROBOT_ENROLLMENT_TOKEN:-}"
fi
if [[ -z "$ENROLLMENT_TOKEN" && "$DRY_RUN" == false && -t 0 ]]; then
  read -r -s -p 'Enrollment token: ' ENROLLMENT_TOKEN
  printf '\n'
fi
[[ -n "$ENROLLMENT_TOKEN" ]] || fail "Provide the enrollment token by file, environment, or interactive prompt"
[[ "$ENROLLMENT_TOKEN" =~ ^[A-Za-z0-9._~-]+$ ]] || fail "Enrollment token contains unsupported characters"

if [[ -z "$REGISTRY_URL" ]]; then
  REGISTRY_URL="$CONTROL_URL"
  REGISTRY_URL="${REGISTRY_URL/ws:\/\//http:\/\/}"
  REGISTRY_URL="${REGISTRY_URL/wss:\/\//https:\/\/}"
  REGISTRY_URL="$REGISTRY_URL/robots"
fi
[[ "$REGISTRY_URL" =~ ^https?://[^[:space:]]+$ ]] || fail "Registry URL must use http:// or https://"

if [[ "$EUID" -ne 0 && "$DRY_RUN" == false ]]; then
  fail "Run this installer with sudo, or use --dry-run"
fi
if [[ "$CONTROL_URL" == ws://* && "$CONTROL_URL" != ws://localhost* && "$CONTROL_URL" != ws://127.0.0.1* ]]; then
  printf 'WARNING: ws:// is unencrypted; use wss:// outside local development.\n' >&2
fi

BUILD_DIR="$(mktemp -d)"
cleanup() {
  rm -rf -- "$BUILD_DIR"
}
trap cleanup EXIT

info "Validated ROS $ROS_DISTRO_NAME, robot profile, and control-plane URLs"
if "$DRY_RUN"; then
  info "Dry run: build and system changes are shown but not executed"
else
  install -d -m 0755 "$BUILD_DIR/src"
  cp -a "$PACKAGE_ROOT" "$BUILD_DIR/src/amr_web_bridge"
  info "Building a relocatable amr_web_bridge installation"
  bash -c "source '/opt/ros/$ROS_DISTRO_NAME/setup.bash' && cd '$BUILD_DIR' && colcon build --merge-install --packages-select amr_web_bridge"
fi

run install -d -m 0755 "$AGENT_WORKSPACE"
if "$DRY_RUN"; then
  printf '[dry-run] install built Agent workspace into %q\n' "$AGENT_WORKSPACE"
else
  cp -a "$BUILD_DIR/install/." "$AGENT_WORKSPACE/"
fi

if ! id "$SERVICE_USER" >/dev/null 2>&1; then
  run useradd --system --home-dir "$STATE_DIR" --shell /usr/sbin/nologin --user-group "$SERVICE_USER"
fi
run install -d -o "$SERVICE_USER" -g "$SERVICE_USER" -m 0700 "$STATE_DIR"
run install -d -o root -g "$SERVICE_USER" -m 0750 "$CONFIG_DIR"
run install -o root -g "$SERVICE_USER" -m 0640 "$PROFILE_FILE" "$CONFIG_DIR/robot-agent.yaml"

ENVIRONMENT_TMP="$BUILD_DIR/agent.env"
{
  printf 'ROBOT_CONTROL_URL=%s\n' "$CONTROL_URL"
  printf 'ROBOT_WORKSPACE=%s\n' "$AGENT_WORKSPACE"
  printf 'ROBOT_PROFILE_FILE=%s/robot-agent.yaml\n' "$CONFIG_DIR"
  printf 'ROBOT_CREDENTIAL_FILE=%s/agent-credential.json\n' "$STATE_DIR"
  printf 'ROBOT_ENROLLMENT_TOKEN=%s\n' "$ENROLLMENT_TOKEN"
  printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO_NAME"
  printf 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n'
  printf 'ROS_DOMAIN_ID=0\n'
} > "$ENVIRONMENT_TMP"
chmod 0600 "$ENVIRONMENT_TMP"
run install -o root -g "$SERVICE_USER" -m 0640 "$ENVIRONMENT_TMP" "$CONFIG_DIR/agent.env"
run install -o root -g root -m 0644 "$PACKAGE_ROOT/deploy/systemd/$SERVICE_NAME" "/etc/systemd/system/$SERVICE_NAME"

run systemctl daemon-reload
run systemctl enable "$SERVICE_NAME"
if "$NO_START"; then
  info "Installed without starting the service (--no-start)"
else
  run systemctl restart "$SERVICE_NAME"
fi

printf '\nRobot Agent installation complete.\n'
printf 'Robot Registry: %s\n' "$REGISTRY_URL"
printf 'Service status: sudo systemctl status %s\n' "$SERVICE_NAME"
printf 'Live logs: sudo journalctl -u %s -f\n' "$SERVICE_NAME"
printf 'After pairing, remove ROBOT_ENROLLMENT_TOKEN from %s and restart the service.\n' "$CONFIG_DIR/agent.env"

if [[ "$NO_START" == false && "$DRY_RUN" == false ]]; then
  info "Waiting briefly for the Agent pairing code"
  for _attempt in {1..15}; do
    PAIRING_LINE="$(journalctl -u "$SERVICE_NAME" -n 100 --no-pager 2>/dev/null | grep -F 'Robot pairing required. Code:' | tail -n 1 || true)"
    if [[ -n "$PAIRING_LINE" ]]; then
      printf '%s\n' "$PAIRING_LINE"
      exit 0
    fi
    if [[ -f "$STATE_DIR/agent-credential.json" ]]; then
      printf 'An existing robot credential was found; pairing is already complete.\n'
      exit 0
    fi
    sleep 1
  done
  printf 'Pairing code is not available yet. Follow the live logs command above.\n'
fi
