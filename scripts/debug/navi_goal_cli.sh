#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [navi_goal_cli_pub.py args]

Purpose:
  Interactive /ly/navi/goal publisher for nav-link joint testing.

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --hz 5
  ./${SCRIPT_NAME} --goal-topic /ly/navi/goal --speed-topic /ly/navi/speed_level
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

source_ros() {
  if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    return
  fi
  local distro
  for distro in humble iron jazzy rolling; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "/opt/ros/${distro}/setup.bash"
      set -u
      return
    fi
  done
  echo "[ERROR] No ROS2 setup found under /opt/ros." >&2
  exit 1
}

source_workspace() {
  local setup_file="${ROOT_DIR}/install/setup.bash"
  if [[ ! -f "${setup_file}" ]]; then
    echo "[ERROR] Workspace setup not found: ${setup_file}" >&2
    echo "        Run: colcon build" >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1091
  source "${setup_file}"
  set -u
}

source_ros
source_workspace

exec python3 "${ROOT_DIR}/scripts/feature_test/standalone/tools/navi_goal_cli_pub.py" "$@"
