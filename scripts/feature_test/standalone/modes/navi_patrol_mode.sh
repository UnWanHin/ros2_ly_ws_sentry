#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
LOCK_FILE="/tmp/sentry_standalone_test.lock"
DEFAULT_PLAN_FILE="${ROOT_DIR}/src/behavior_tree/Scripts/ConfigJson/navi_debug_points.json"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/feature_test/standalone/lib/common.sh"

PLAN_FILE="${DEFAULT_PLAN_FILE}"
FORWARD_ARGS=()

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--plan-file PATH] [nav_goal_patrol_pub.py args]

Purpose:
  Standalone /ly/navi/goal patrol publisher for temporary navigation point testing.
  Reads the same JSON plan file used by behavior_tree NaviDebug mode.

Examples:
  ./scripts/feature_test/standalone/modes/navi_patrol_mode.sh
  ./scripts/feature_test/standalone/modes/navi_patrol_mode.sh --plan test_site_sequence
  ./scripts/feature_test/standalone/modes/navi_patrol_mode.sh --team blue --hz 5
EOF
}

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

  st_err "No ROS2 setup found under /opt/ros."
  exit 1
}

source_workspace_if_exists() {
  local setup_file="${ROOT_DIR}/install/setup.bash"
  if [[ -f "${setup_file}" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${setup_file}"
    set -u
  fi
}

cleanup() {
  st_release_lock "${LOCK_FILE}"
}

trap cleanup EXIT INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --plan-file)
      PLAN_FILE="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      FORWARD_ARGS+=("$1")
      shift
      ;;
  esac
done

st_acquire_lock "${LOCK_FILE}"
source_ros
source_workspace_if_exists

st_info "Start nav patrol publisher with plan file: ${PLAN_FILE}"
python3 "${ROOT_DIR}/scripts/feature_test/standalone/tools/nav_goal_patrol_pub.py" \
  --plan-file "${PLAN_FILE}" \
  "${FORWARD_ARGS[@]}"
