#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
LOCK_FILE="/tmp/sentry_standalone_test.lock"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/feature_test/standalone/lib/common.sh"

OFFLINE_MODE=0
MODE_ARG="regional"
ROTATE_LEVEL=1
SPEED_X=20
SPEED_Y=0
PERIOD_SEC=4
HZ=20
WAIT_SEC=4
CONFIG_FILE=""
LAUNCH_PID=""
TEST_PID=""
LAUNCH_LOG=""

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--offline|--online] [--mode 1|2|3|league|regional|showcase] [--rotate-level 0..3] [--speed-x N] [--speed-y N] [--period-sec SEC] [--hz N] [--wait SEC] [--config-file PATH]

Purpose:
  Start minimal stack (gimbal_driver only) and publish:
    - /ly/control/firecode Rotate bits
    - /ly/control/vel back-and-forth translation
  This is for chassis spin + translation link testing without BT/referee gate.

Examples:
  ./scripts/feature_test/standalone/modes/chassis_spin_translate_mode.sh
  ./scripts/feature_test/standalone/modes/chassis_spin_translate_mode.sh --offline --rotate-level 2 --speed-x 30 --period-sec 6
EOF
}

cleanup() {
  st_stop_pid "${TEST_PID}" "chassis_spin_translate_pub"
  st_stop_pid "${LAUNCH_PID}" "minimal_stack"
  st_release_lock "${LOCK_FILE}"
}

trap cleanup EXIT INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --offline)
      OFFLINE_MODE=1
      shift
      ;;
    --online)
      OFFLINE_MODE=0
      shift
      ;;
    --mode)
      MODE_ARG="$2"
      shift 2
      ;;
    --rotate-level)
      ROTATE_LEVEL="$2"
      shift 2
      ;;
    --speed-x)
      SPEED_X="$2"
      shift 2
      ;;
    --speed-y)
      SPEED_Y="$2"
      shift 2
      ;;
    --period-sec)
      PERIOD_SEC="$2"
      shift 2
      ;;
    --hz)
      HZ="$2"
      shift 2
      ;;
    --wait)
      WAIT_SEC="$2"
      shift 2
      ;;
    --config-file)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      st_err "Unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

if ! [[ "${ROTATE_LEVEL}" =~ ^[0-3]$ ]]; then
  st_err "--rotate-level must be 0..3"
  exit 2
fi

st_acquire_lock "${LOCK_FILE}"

mkdir -p /tmp/ros2_logs
export ROS_LOG_DIR=/tmp/ros2_logs

LAUNCH_ARGS=(
  "use_gimbal:=true"
  "use_detector:=false"
  "use_tracker:=false"
  "use_predictor:=false"
  "use_outpost:=false"
  "use_buff:=false"
  "use_behavior_tree:=false"
)

if (( OFFLINE_MODE == 1 )); then
  LAUNCH_ARGS+=("offline:=true")
fi
if [[ -n "${CONFIG_FILE}" ]]; then
  LAUNCH_ARGS+=("config_file:=${CONFIG_FILE}")
fi

LAUNCH_CMD=(
  "${ROOT_DIR}/scripts/start/sentry_all.sh"
  "--mode" "${MODE_ARG}"
  "--no-prompt"
  "--"
  "${LAUNCH_ARGS[@]}"
)

LAUNCH_LOG="$(mktemp /tmp/chassis_spin_translate_launch.XXXXXX.log)"
st_info "Launching minimal stack, log=${LAUNCH_LOG}"
"${LAUNCH_CMD[@]}" >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID="$!"

sleep "${WAIT_SEC}"
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  st_err "Stack exited early, check log: ${LAUNCH_LOG}"
  tail -n 80 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

st_info "Start chassis spin+translate publisher: rotate=${ROTATE_LEVEL}, vel=(${SPEED_X},${SPEED_Y}), period=${PERIOD_SEC}, hz=${HZ}"
python3 "${ROOT_DIR}/scripts/feature_test/chassis_spin_translate_test.py" \
  --rotate-level "${ROTATE_LEVEL}" \
  --speed-x "${SPEED_X}" \
  --speed-y "${SPEED_Y}" \
  --period-sec "${PERIOD_SEC}" \
  --hz "${HZ}" \
  --vel-topic "/ly/control/vel" \
  --firecode-topic "/ly/control/firecode" &
TEST_PID="$!"

wait "${TEST_PID}"
