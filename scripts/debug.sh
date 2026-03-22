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
  competition-autoaim   Competition-style autoaim preset for debug/verification.
  autoaim-debug         Detector/tracker/predictor/mapper split debug.
  autoaim-test          Quick standalone armor autoaim test.
  navi-debug            behavior_tree-only navigation debug.
  standalone            Standalone submenu (armor/buff/outpost/spin/navi).
  navi-patrol           JSON-driven /ly/navi/goal patrol.
  navi-goal-cli         Interactive /ly/navi/goal publisher.
  ballistic-log         /rosout ballistic log filter.
  shooting-table-calib  Shooting table calibration/debug.
  shooting-table-autoaim Shooting-table direct autoaim verification.
  control-angles-test    Publish one /ly/control/angles GimbalAngles command.
  chassis-gyro          FireCode.Rotate chain test.
  chassis-gyro-translate FireCode.Rotate + /ly/control/vel round-trip test.
  rotate-level-test     /ly/control/firecode rotate level cycle test.
  posture-test          /ly/control/posture cycle test and /ly/gimbal/posture watch.

Examples:
  ./scripts/debug.sh
  ./scripts/debug.sh competition-autoaim --mode league
  ./scripts/debug.sh autoaim-debug --online
  ./scripts/debug.sh navi-debug --with-gate
EOF
}

run_entry() {
  local entry="${1:-}"
  shift || true
  case "${entry,,}" in
    1|competition|competition-autoaim|competition_autoaim)
      exec "${ROOT_DIR}/debug/competition_autoaim.sh" "$@"
      ;;
    2|autoaim-debug|autoaim_debug)
      exec "${ROOT_DIR}/debug/autoaim_debug.sh" "$@"
      ;;
    3|autoaim-test|autoaim_test)
      exec "${ROOT_DIR}/debug/autoaim_test.sh" "$@"
      ;;
    4|navi-debug|navi_debug)
      exec "${ROOT_DIR}/debug/navi_debug.sh" "$@"
      ;;
    5|standalone|standalone-test|standalone_test)
      exec "${ROOT_DIR}/debug/standalone.sh" "$@"
      ;;
    6|navi-patrol|navi_patrol)
      exec "${ROOT_DIR}/debug/navi_patrol.sh" "$@"
      ;;
    7|navi-goal-cli|navi_goal_cli)
      exec "${ROOT_DIR}/debug/navi_goal_cli.sh" "$@"
      ;;
    8|ballistic-log|ballistic_error_log|ballistic-log)
      exec "${ROOT_DIR}/debug/ballistic_error_log.sh" "$@"
      ;;
    9|shooting-table|shooting_table|shooting-table-calib|shooting_table_calib|calib)
      exec "${ROOT_DIR}/debug/shooting_table_calib.sh" "$@"
      ;;
    10|shooting-table-autoaim|shooting_table_autoaim)
      exec "${ROOT_DIR}/debug/shooting_table_autoaim.sh" "$@"
      ;;
    11|control-angles-test|control_angles_test|control-angles|control_angles)
      exec "${ROOT_DIR}/debug/control_angles_test.sh" "$@"
      ;;
    12|chassis-gyro|chassis_gyro|gyro)
      exec "${ROOT_DIR}/debug/chassis_gyro.sh" "$@"
      ;;
    13|chassis-gyro-translate|chassis_gyro_translate|gyro-translate|gyro_translate)
      exec "${ROOT_DIR}/debug/chassis_spin_translate.sh" "$@"
      ;;
    14|rotate-level-test|rotate_level_test|rotate-level|rotate_level)
      exec "${ROOT_DIR}/debug/rotate_level_test.sh" "$@"
      ;;
    15|posture-test|posture_test|posture)
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
echo "  1) competition-autoaim"
echo "  2) autoaim-debug"
echo "  3) autoaim-test"
echo "  4) navi-debug"
echo "  5) standalone"
echo "  6) navi-patrol"
echo "  7) navi-goal-cli"
echo "  8) ballistic-log"
echo "  9) shooting-table-calib"
echo " 10) shooting-table-autoaim"
echo " 11) control-angles-test"
echo " 12) chassis-gyro"
echo " 13) chassis-gyro-translate"
echo " 14) rotate-level-test"
echo " 15) posture-test"
read -r -p "Input 1-15 [default: 2]: " choice
choice="${choice:-2}"
run_entry "${choice}"
