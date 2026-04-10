#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

MODE="loop"
YAW="0.0"
PITCH="0.0"
TOPIC="/ly/control/angles"
LAUNCH_GIMBAL=1
WAIT_SEC=3
INTERVAL_SEC=1
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
  ${SCRIPT_NAME} [loop|once|echo] [options]

Modes:
  loop    Publish the same GimbalAngles message repeatedly
  once    Publish one GimbalAngles message to ${TOPIC}
  echo    Echo the topic and do not publish

Options:
  --yaw DEG      Yaw angle in degrees. Default: ${YAW}
  --pitch DEG    Pitch angle in degrees. Default: ${PITCH}
  --topic TOPIC  Target topic. Default: ${TOPIC}
  --interval SEC Publish interval for loop mode. Default: ${INTERVAL_SEC}
  --launch-gimbal / --no-launch-gimbal
                   Whether to start gimbal_driver automatically. Default: launch.
  --wait SEC       Wait time after starting gimbal_driver. Default: ${WAIT_SEC}
  --config-file    Params YAML for gimbal_driver. Default: ${CONFIG_FILE}
  --use-virtual-device true|false
                   Whether to force virtual device in gimbal_driver. Default: ${USE_VIRTUAL_DEVICE}
  --output screen|log
                   gimbal_driver launch output. Default: ${OUTPUT_MODE}
  -h, --help     Show help.

Examples:
  ./${SCRIPT_NAME} --yaw 10 --pitch 2
  ./${SCRIPT_NAME} loop --yaw 10 --pitch 2 --interval 1
  ./${SCRIPT_NAME} once --yaw -30 --pitch 5
  ./${SCRIPT_NAME} echo
EOF
}

cleanup() {
  if [[ -n "${GIMBAL_PID:-}" ]] && kill -0 "${GIMBAL_PID}" 2>/dev/null; then
    kill -INT "${GIMBAL_PID}" 2>/dev/null || true
    sleep 1
    kill -TERM "${GIMBAL_PID}" 2>/dev/null || true
    wait "${GIMBAL_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    loop|once|echo)
      MODE="$1"
      shift
      ;;
    --yaw)
      YAW="$2"
      shift 2
      ;;
    --pitch)
      PITCH="$2"
      shift 2
      ;;
    --topic)
      TOPIC="$2"
      shift 2
      ;;
    --interval)
      INTERVAL_SEC="$2"
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

  echo "[CONTROL-ANGLES-TEST][INFO] Launching gimbal_driver with config=${CONFIG_FILE} use_virtual_device=${USE_VIRTUAL_DEVICE}" >&2
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

launch_gimbal_driver

publish_once() {
  echo "[CONTROL-ANGLES-TEST][TX] topic=${TOPIC} yaw=${YAW} pitch=${PITCH}" >&2
  ros2 topic pub "${TOPIC}" gimbal_driver/msg/GimbalAngles "{yaw: ${YAW}, pitch: ${PITCH}}" -1 >/dev/null
}

case "${MODE}" in
  loop)
    while true; do
      publish_once
      sleep "${INTERVAL_SEC}"
    done
    ;;
  once)
    exec ros2 topic pub "${TOPIC}" gimbal_driver/msg/GimbalAngles "{yaw: ${YAW}, pitch: ${PITCH}}" -1
    ;;
  echo)
    exec ros2 topic echo "${TOPIC}"
    ;;
esac
