#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

AUTO_FIRE="true"
TARGET_TYPE="4"
DRY_RUN=0
PASSTHROUGH_ARGS=()
EXTRA_LAUNCH_ARGS=()

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [start_shooting_table_calib options...] [autoaim options] [-- <extra ros2 launch args...>]

Purpose:
  Start shooting_table_calib in direct autoaim mode:
    - reuse shooting_table_calib's own detector + tracker + ballistic solve
    - continuously lock target like repeated 'a'
    - publish /ly/control/angles directly
    - only fire after yaw/pitch both converge
    - do not use predictor / behavior_tree

Autoaim options:
  --aim-only                        Keep auto lock but never auto fire.
  --target-type <name|id>           Auto target armor type. Default: infantry2 (4).
  --dry-run                         Print final command only.
  -h, --help                        Show help.

Target type mapping:
  hero=1 engineer=2 infantry1=3 infantry2=4 infantry3=5 sentry=6 outpost=7

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --team blue
  ./${SCRIPT_NAME} --aim-only
  ./${SCRIPT_NAME} --target-type hero
  ./${SCRIPT_NAME} --use-gimbal false --output log -- detector_config.use_video:=true detector_config.video_path:=src/record/record.mkv

Notes:
  - Other options are forwarded to scripts/debug/shooting_table_calib.sh unchanged.
  - This script defaults to --no-auto-fit, because it is for direct autoaim verification rather than calibration fitting.
EOF
}

require_value() {
  local opt="$1"
  local value="${2-}"
  if [[ -z "${value}" || "${value}" == --* ]]; then
    echo "[ERROR] ${opt} requires a value." >&2
    usage >&2
    exit 2
  fi
}

normalize_target_type() {
  local raw="${1,,}"
  case "${raw}" in
    1|hero)
      echo "1"
      ;;
    2|engineer|eng)
      echo "2"
      ;;
    3|infantry1|infantry_1|inf1)
      echo "3"
      ;;
    4|infantry2|infantry_2|inf2)
      echo "4"
      ;;
    5|infantry3|infantry_3|inf3)
      echo "5"
      ;;
    6|sentry)
      echo "6"
      ;;
    7|outpost)
      echo "7"
      ;;
    *)
      echo "[ERROR] Unsupported target type: ${1}" >&2
      echo "[ERROR] Supported: hero engineer infantry1 infantry2 infantry3 sentry outpost or 1..7" >&2
      return 1
      ;;
  esac
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --aim-only)
      AUTO_FIRE="false"
      shift
      ;;
    --target-type|--auto-target-type)
      require_value "$1" "${2-}"
      TARGET_TYPE="$(normalize_target_type "$2")"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_LAUNCH_ARGS=("$@")
      break
      ;;
    *)
      PASSTHROUGH_ARGS+=("$1")
      shift
      ;;
  esac
done

CMD=(
  "${ROOT_DIR}/scripts/debug/shooting_table_calib.sh"
  --no-auto-fit
  "${PASSTHROUGH_ARGS[@]}"
  --
  "auto_lock_fire:=true"
  "auto_fire:=${AUTO_FIRE}"
  "auto_target_type:=${TARGET_TYPE}"
  "${EXTRA_LAUNCH_ARGS[@]}"
)

echo "[INFO] shooting_table_autoaim mode=shooting_table_only auto_lock_fire=true auto_fire=${AUTO_FIRE} auto_target_type=${TARGET_TYPE}"
echo "[INFO] forwarding to scripts/debug/shooting_table_calib.sh"

if (( DRY_RUN == 1 )); then
  printf "[INFO] dry-run cmd:"
  printf " %q" "${CMD[@]}"
  printf "\n"
  exit 0
fi

exec "${CMD[@]}"
