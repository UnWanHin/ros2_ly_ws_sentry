#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
LOCK_FILE="/tmp/sentry_standalone_test.lock"
LOG_DIR="/tmp/ros2_standalone_logs"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/feature_test/standalone/lib/common.sh"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/feature_test/lib/common.sh"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/feature_test/lib/guard.sh"

FT_CMD_TIMEOUT_SEC=5

OFFLINE_MODE=0
MODE_ARG="regional"
WAIT_SEC=6
CONFIG_FILE=""
ENABLE_FIRE="false"
FIRE_HZ=20
TARGET_TIMEOUT_SEC=1.0
ALLOW_MULTI_CONTROL=0
CLEANUP_EXISTING=1

LAUNCH_PID=""
MODE_PUB_PID=""
BRIDGE_PID=""
LAUNCH_LOG=""

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [options]

Purpose:
  Standalone buff mode:
  - Launch minimal stack (gimbal_driver + detector + buff_hitter, no behavior_tree)
  - Publish mode switch topics (/ly/aa_enable=false, /ly/ra_enable=true)
  - Bridge /ly/buff/target -> /ly/control/angles,/ly/control/firecode

Options:
  --offline|--online
  --mode 1|2|3|league|regional|showcase
  --wait SEC
  --config-file PATH
  --enable-fire true|false
  --fire-hz N
  --target-timeout-sec SEC
  --allow-multi-control
  --cleanup-existing|--no-cleanup-existing

Examples:
  ./scripts/feature_test/standalone/modes/buff_mode.sh
  ./scripts/feature_test/standalone/modes/buff_mode.sh --offline --enable-fire true
EOF
}

cleanup() {
  st_stop_pid "${BRIDGE_PID}" "target_bridge"
  st_stop_pid "${MODE_PUB_PID}" "mode_publisher"
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
    --wait)
      WAIT_SEC="$2"
      shift 2
      ;;
    --config-file)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --enable-fire)
      ENABLE_FIRE="$2"
      shift 2
      ;;
    --fire-hz)
      FIRE_HZ="$2"
      shift 2
      ;;
    --target-timeout-sec)
      TARGET_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --allow-multi-control)
      ALLOW_MULTI_CONTROL=1
      shift
      ;;
    --cleanup-existing)
      CLEANUP_EXISTING=1
      shift
      ;;
    --no-cleanup-existing)
      CLEANUP_EXISTING=0
      shift
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

mkdir -p "${LOG_DIR}"
: "${ROS_LOG_DIR:=/tmp/ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

ft_source_ros
ft_source_workspace "${ROOT_DIR}"

st_acquire_lock "${LOCK_FILE}"

if (( CLEANUP_EXISTING == 1 )); then
  pkill -f "${ROOT_DIR}/scripts/feature_test/standalone/tools/control_mode_publisher.py" >/dev/null 2>&1 || true
  pkill -f "${ROOT_DIR}/scripts/feature_test/standalone/tools/target_to_control_bridge.py" >/dev/null 2>&1 || true
fi

if (( ALLOW_MULTI_CONTROL == 0 )); then
  ft_require_behavior_tree_absent
  ft_require_topic_no_publisher "/ly/control/angles"
  ft_require_topic_no_publisher "/ly/control/firecode"
else
  st_warn "allow_multi_control=true: single-controller guard disabled."
fi

LAUNCH_ARGS=(
  "use_gimbal:=true"
  "use_detector:=true"
  "use_tracker:=false"
  "use_predictor:=false"
  "use_outpost:=false"
  "use_buff:=true"
  "use_behavior_tree:=false"
)

if (( OFFLINE_MODE == 1 )); then
  LAUNCH_ARGS+=("offline:=true")
fi
if [[ -n "${CONFIG_FILE}" ]]; then
  LAUNCH_ARGS+=("config_file:=${CONFIG_FILE}")
fi

START_ARGS=("--mode" "${MODE_ARG}" "--no-prompt")
if (( CLEANUP_EXISTING == 1 )); then
  START_ARGS+=("--cleanup-existing")
else
  START_ARGS+=("--no-cleanup-existing")
fi

LAUNCH_CMD=(
  "${ROOT_DIR}/scripts/start/sentry_all.sh"
  "${START_ARGS[@]}"
  --
  "${LAUNCH_ARGS[@]}"
)

LAUNCH_LOG="${LOG_DIR}/buff_mode_$(date +%Y%m%d_%H%M%S).log"
st_info "Launch minimal stack log: ${LAUNCH_LOG}"
"${LAUNCH_CMD[@]}" >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID="$!"

sleep "${WAIT_SEC}"
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  st_err "Minimal stack exited early. log=${LAUNCH_LOG}"
  tail -n 120 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

python3 "${ROOT_DIR}/scripts/feature_test/standalone/tools/control_mode_publisher.py" \
  --aa-enable false \
  --ra-enable true \
  --outpost-enable false \
  --bt-target 6 \
  --hz 5 &
MODE_PUB_PID="$!"

python3 "${ROOT_DIR}/scripts/feature_test/standalone/tools/target_to_control_bridge.py" \
  --target-topic "/ly/buff/target" \
  --angles-topic "/ly/control/angles" \
  --firecode-topic "/ly/control/firecode" \
  --timeout-sec "${TARGET_TIMEOUT_SEC}" \
  --publish-hz 50 \
  --enable-fire "${ENABLE_FIRE}" \
  --fire-hz "${FIRE_HZ}" \
  --safe-firecode 0 &
BRIDGE_PID="$!"

st_info "Buff standalone mode running. Press Ctrl+C to stop."
set +e
wait -n "${LAUNCH_PID}" "${MODE_PUB_PID}" "${BRIDGE_PID}"
RC=$?
set -e

if (( RC != 0 )); then
  st_err "A process exited with code ${RC}."
else
  st_warn "A process exited; shutting down."
  RC=1
fi
exit "${RC}"
