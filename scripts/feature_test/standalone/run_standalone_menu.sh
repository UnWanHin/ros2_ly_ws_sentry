#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--select 1|2|3|4|5|6] [mode_args...]

Menu:
  1) Armor board (auto_aim + mapper)
  2) Buff (standalone)
  3) Outpost (standalone)
  4) Chassis spin (firecode.rotate)
  5) Navi patrol (/ly/navi/goal)
  6) Chassis spin + translate (/ly/control/firecode + /ly/control/vel)

Examples:
  ./scripts/feature_test/standalone/run_standalone_menu.sh
  ./scripts/feature_test/standalone/run_standalone_menu.sh --select 1 --online
  ./scripts/feature_test/standalone/run_standalone_menu.sh --select 4 --rotate-level 2
  ./scripts/feature_test/standalone/run_standalone_menu.sh --select 6 --speed-x 30 --period-sec 6
  ./scripts/feature_test/standalone/run_standalone_menu.sh --select 5 --plan test_site_sequence
EOF
}

SELECTION=""
FORWARD_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --select)
      SELECTION="$2"
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

if [[ -z "${SELECTION}" ]]; then
  echo "Select standalone test mode:"
  echo "  1) Armor board"
  echo "  2) Buff"
  echo "  3) Outpost"
  echo "  4) Chassis spin"
  echo "  5) Navi patrol"
  echo "  6) Chassis spin + translate"
  read -r -p "Input 1-6: " SELECTION
fi

MODE_SCRIPT=""
case "${SELECTION}" in
  1)
    MODE_SCRIPT="${ROOT_DIR}/scripts/feature_test/standalone/modes/armor_mode.sh"
    ;;
  2)
    MODE_SCRIPT="${ROOT_DIR}/scripts/feature_test/standalone/modes/buff_mode.sh"
    ;;
  3)
    MODE_SCRIPT="${ROOT_DIR}/scripts/feature_test/standalone/modes/outpost_mode.sh"
    ;;
  4)
    MODE_SCRIPT="${ROOT_DIR}/scripts/feature_test/standalone/modes/chassis_spin_mode.sh"
    ;;
  5)
    MODE_SCRIPT="${ROOT_DIR}/scripts/feature_test/standalone/modes/navi_patrol_mode.sh"
    ;;
  6)
    MODE_SCRIPT="${ROOT_DIR}/scripts/feature_test/standalone/modes/chassis_spin_translate_mode.sh"
    ;;
  *)
    echo "[ERROR] Invalid selection: ${SELECTION}" >&2
    usage
    exit 2
    ;;
esac

exec "${MODE_SCRIPT}" "${FORWARD_ARGS[@]}"
