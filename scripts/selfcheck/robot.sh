#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

: "${ROS_LOG_DIR:=/tmp/ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

WAIT_SECONDS=18
WITH_HZ=0
LAUNCH_ARGS=()

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--wait SECONDS] [--with-hz] [-- <launch_args...>]

Examples:
  # 車端快速自檢（推薦）
  ./${SCRIPT_NAME}

  # 車端完整自檢（包含頻率採樣）
  ./${SCRIPT_NAME} --with-hz

  # 傳遞 launch 參數
  ./${SCRIPT_NAME} -- --config_file:=/abs/path/override_config.yaml
EOF
}

log() {
  printf "[ROBOT-CHECK] %s\n" "$*"
}

warn() {
  printf "[ROBOT-CHECK][WARN] %s\n" "$*"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --wait)
      WAIT_SECONDS="$2"
      shift 2
      ;;
    --with-hz)
      WITH_HZ=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      LAUNCH_ARGS=("$@")
      break
      ;;
    *)
      printf "[ERROR] Unknown argument: %s\n" "$1" >&2
      usage
      exit 2
      ;;
  esac
done

cd "${ROOT_DIR}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  printf "[ERROR] /opt/ros/humble/setup.bash not found\n" >&2
  exit 1
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "${ROOT_DIR}/install/setup.bash"
  set -u
else
  printf "[ERROR] install/setup.bash missing. Build workspace first.\n" >&2
  exit 1
fi

log "Hardware quick check: serial device candidates"
SERIAL_HITS="$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)"
if [[ -n "${SERIAL_HITS}" ]]; then
  printf "%s\n" "${SERIAL_HITS}"
else
  warn "No /dev/ttyACM* or /dev/ttyUSB* found. If using virtual device this is expected."
fi

if id -nG "${USER}" | grep -qw dialout; then
  log "User ${USER} is in dialout group"
else
  warn "User ${USER} is not in dialout group; serial open may fail."
fi

if ping -c 1 -W 1 192.168.12.1 >/dev/null 2>&1; then
  log "192.168.12.1 reachable (referee/video network path looks up)"
else
  warn "192.168.12.1 unreachable (may be normal if network not connected yet)"
fi

SELF_CHECK_ARGS=(--runtime-only --launch --wait "${WAIT_SECONDS}")
if (( WITH_HZ == 0 )); then
  SELF_CHECK_ARGS+=(--skip-hz)
fi
if (( ${#LAUNCH_ARGS[@]} > 0 )); then
  SELF_CHECK_ARGS+=(-- "${LAUNCH_ARGS[@]}")
fi

log "Running runtime self-check suite"
"${ROOT_DIR}/scripts/selfcheck/sentry.sh" "${SELF_CHECK_ARGS[@]}"

log "Robot self-check completed"
