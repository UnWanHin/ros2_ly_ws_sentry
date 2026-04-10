#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

MODE="both"
INTERVAL_SEC=10
TX_TOPIC="/ly/control/posture"
RX_TOPIC="/ly/gimbal/posture"
ECHO_TOPIC="/ly/control/posture"
VALUES=(1 2 3)
LAUNCH_GIMBAL=1
WAIT_SEC=3
USE_VIRTUAL_DEVICE="false"
OUTPUT_MODE="screen"
CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
GIMBAL_PID=""
STACK_NODE_REGEX="/(gimbal_driver_node)([[:space:]]|$)"
STACK_LAUNCH_REGEX="ros2 launch gimbal_driver gimbal_driver.launch.py"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [tx|rx|echo|both] [options]

Modes:
  tx      Publish posture commands in a loop: 1 -> 2 -> 3 -> ...
  rx      Echo posture feedback topic (${RX_TOPIC})
  echo    Echo command topic (${ECHO_TOPIC})
  both    Start tx loop and echo feedback topic together

Options:
  --interval SEC   Publish interval for tx/both. Default: ${INTERVAL_SEC}
  --tx-topic TOPIC Command topic. Default: ${TX_TOPIC}
  --rx-topic TOPIC Feedback topic. Default: ${RX_TOPIC}
  --echo-topic TOPIC Echo command topic. Default: ${ECHO_TOPIC}
  --launch-gimbal / --no-launch-gimbal
                   Whether to start gimbal_driver automatically. Default: launch.
  --wait SEC       Wait time after starting gimbal_driver. Default: ${WAIT_SEC}
  --config-file    Params YAML for gimbal_driver. Default: ${CONFIG_FILE}
  --use-virtual-device true|false
                   Whether to force virtual device in gimbal_driver. Default: ${USE_VIRTUAL_DEVICE}
  --output screen|log
                   gimbal_driver launch output. Default: ${OUTPUT_MODE}
  -h, --help       Show help.

Posture values:
  1 = Attack
  2 = Defense
  3 = Move
EOF
}

cleanup() {
  if [[ -n "${GIMBAL_PID:-}" ]] && kill -0 "${GIMBAL_PID}" 2>/dev/null; then
    kill -INT "${GIMBAL_PID}" 2>/dev/null || true
    sleep 1
    kill -TERM "${GIMBAL_PID}" 2>/dev/null || true
    wait "${GIMBAL_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RX_PID:-}" ]] && kill -0 "${RX_PID}" 2>/dev/null; then
    kill -TERM "${RX_PID}" 2>/dev/null || true
    wait "${RX_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    tx|rx|echo|both)
      MODE="$1"
      shift
      ;;
    --interval)
      INTERVAL_SEC="$2"
      shift 2
      ;;
    --tx-topic)
      TX_TOPIC="$2"
      shift 2
      ;;
    --rx-topic)
      RX_TOPIC="$2"
      shift 2
      ;;
    --echo-topic)
      ECHO_TOPIC="$2"
      shift 2
      ;;
    --launch-gimbal)
      LAUNCH_GIMBAL=1
      shift
      ;;
    --no-launch-gimbal)
      LAUNCH_GIMBAL=0
      shift
      ;;
    --wait)
      WAIT_SEC="$2"
      shift 2
      ;;
    --config-file)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --use-virtual-device)
      USE_VIRTUAL_DEVICE="$2"
      shift 2
      ;;
    --output)
      OUTPUT_MODE="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

source_ros_workspace "${ROOT_DIR}"

launch_gimbal_driver() {
  if (( LAUNCH_GIMBAL == 0 )); then
    return 0
  fi

  if [[ ! -f "${CONFIG_FILE}" ]]; then
    echo "[ERROR] Config file not found: ${CONFIG_FILE}" >&2
    exit 1
  fi

  cleanup_existing_stack "1" "${STACK_NODE_REGEX}" "${STACK_LAUNCH_REGEX}"

  echo "[POSTURE-TEST][INFO] Launching gimbal_driver with config=${CONFIG_FILE} use_virtual_device=${USE_VIRTUAL_DEVICE}" >&2
  ros2 launch gimbal_driver gimbal_driver.launch.py \
    "config_file:=${CONFIG_FILE}" \
    "use_virtual_device:=${USE_VIRTUAL_DEVICE}" \
    "output:=${OUTPUT_MODE}" &
  GIMBAL_PID="$!"
  sleep "${WAIT_SEC}"

  if ! kill -0 "${GIMBAL_PID}" 2>/dev/null; then
    echo "[ERROR] gimbal_driver exited early." >&2
    exit 1
  fi
}

publish_once() {
  local value="$1"
  echo "[POSTURE-TEST][TX] topic=${TX_TOPIC} data=${value}" >&2
  ros2 topic pub "${TX_TOPIC}" std_msgs/msg/UInt8 "{data: ${value}}" -1 >/dev/null
}

run_tx_loop() {
  local idx=0
  while true; do
    publish_once "${VALUES[$idx]}"
    idx=$(((idx + 1) % ${#VALUES[@]}))
    sleep "${INTERVAL_SEC}"
  done
}

launch_gimbal_driver

case "${MODE}" in
  tx)
    run_tx_loop
    ;;
  rx)
    exec ros2 topic echo "${RX_TOPIC}"
    ;;
  echo)
    exec ros2 topic echo "${ECHO_TOPIC}"
    ;;
  both)
    ros2 topic echo "${RX_TOPIC}" &
    RX_PID="$!"
    run_tx_loop
    ;;
esac
