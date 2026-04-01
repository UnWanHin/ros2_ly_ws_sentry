#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [entry] [args...]

Entries:
  armor_test            Competition-style armor test preset.
  navi-debug            behavior_tree-only navigation debug.
  standalone            Standalone submenu (armor/buff/outpost/spin/navi).
  navi_goal             JSON-driven /ly/navi/goal patrol.
  navi-goal-cli         Interactive /ly/navi/goal publisher.
  ballistic-log         /rosout ballistic log filter.
  shooting-table-calib  Shooting table calibration/debug.
  control-angles-test    Publish one /ly/control/angles GimbalAngles command.
  rotate_level          /ly/control/firecode rotate level cycle test.
  move_rotate           Rotate + /ly/control/vel sine-translate test.
  posture-test          /ly/control/posture cycle test and /ly/gimbal/posture watch.

Examples:
  ./scripts/debug.sh
  ./scripts/debug.sh armor_test --mode regional
  ./scripts/debug.sh navi-debug --with-gate
EOF
}

run_entry() {
  local entry="${1:-}"
  shift || true
  case "${entry,,}" in
    1|armor-test|armor_test|armor)
      exec "${ROOT_DIR}/debug/armor_test.sh" "$@"
      ;;
    2|navi-debug|navi_debug)
      exec "${ROOT_DIR}/debug/navi_debug.sh" "$@"
      ;;
    3|standalone|standalone-test|standalone_test)
      exec "${ROOT_DIR}/debug/standalone.sh" "$@"
      ;;
    4|navi-goal|navi_goal)
      exec "${ROOT_DIR}/debug/navi_goal.sh" "$@"
      ;;
    5|navi-goal-cli|navi_goal_cli)
      exec "${ROOT_DIR}/debug/navi_goal_cli.sh" "$@"
      ;;
    6|ballistic-log|ballistic_error_log|ballistic-log)
      exec "${ROOT_DIR}/debug/ballistic_error_log.sh" "$@"
      ;;
    7|shooting-table|shooting_table|shooting-table-calib|shooting_table_calib|calib)
      exec "${ROOT_DIR}/debug/shooting_table_calib.sh" "$@"
      ;;
    8|control-angles-test|control_angles_test|control-angles|control_angles)
      exec "${ROOT_DIR}/debug/control_angles_test.sh" "$@"
      ;;
    9|rotate-level|rotate_level)
      exec "${ROOT_DIR}/debug/rotate_level.sh" "$@"
      ;;
    10|move-rotate|move_rotate)
      exec "${ROOT_DIR}/debug/move_rotate.sh" "$@"
      ;;
    11|posture-test|posture_test|posture)
      exec "${ROOT_DIR}/debug/posture_test.sh" "$@"
      ;;
    ""|menu)
      ;;
    *)
      echo "[ERROR] Unknown debug entry: ${entry}" >&2
      usage >&2
      exit 2
      ;;
  esac
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

if (( $# > 0 )); then
  run_entry "$@"
fi

echo "Select debug entry:"
echo "  1) armor_test"
echo "  2) navi-debug"
echo "  3) standalone"
echo "  4) navi_goal"
echo "  5) navi-goal-cli"
echo "  6) ballistic-log"
echo "  7) shooting-table-calib"
echo "  8) control-angles-test"
echo "  9) rotate_level"
echo " 10) move_rotate"
echo " 11) posture-test"
read -r -p "Input 1-11 [default: 1]: " choice
choice="${choice:-1}"
run_entry "${choice}"
