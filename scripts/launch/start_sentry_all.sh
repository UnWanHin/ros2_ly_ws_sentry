#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
CLEANUP_EXISTING=1
OFFLINE_MODE=0
MODE_ARG=""
LAUNCH_ARGS=()
DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/scripts/config/stack/base_competition.yaml"
DEFAULT_DETECTOR_CONFIG_FILE="${ROOT_DIR}/scripts/config/stack/detector_competition.yaml"
DEFAULT_PREDICTOR_CONFIG_FILE="${ROOT_DIR}/scripts/config/stack/predictor_competition.yaml"
DEFAULT_OUTPOST_CONFIG_FILE="${ROOT_DIR}/scripts/config/stack/outpost_competition.yaml"
DEFAULT_BUFF_CONFIG_FILE="${ROOT_DIR}/scripts/config/stack/buff_competition.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/scripts/config/stack/override_none.yaml"

STACK_LAUNCH_REGEX="ros2 launch behavior_tree sentry_all.launch.py"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF2
Usage:
  ${SCRIPT_NAME} [--cleanup-existing|--no-cleanup-existing] [--offline] [--mode 1|2|3|league|regional|showcase] [-- <launch_args...>]

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --no-cleanup-existing
  ./${SCRIPT_NAME} --offline
  ./${SCRIPT_NAME} --mode 1
  ./${SCRIPT_NAME} --mode regional --no-prompt
  ./${SCRIPT_NAME} --mode 3 --no-prompt
  ./${SCRIPT_NAME} -- use_buff:=false use_outpost:=false
EOF2
}

has_launch_arg_key() {
  local key="$1"
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "${arg}" == "${key}:="* ]]; then
      return 0
    fi
  done
  return 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cleanup-existing)
      CLEANUP_EXISTING=1
      shift
      ;;
    --no-cleanup-existing)
      CLEANUP_EXISTING=0
      shift
      ;;
    --offline)
      OFFLINE_MODE=1
      shift
      ;;
    --mode)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --mode requires a value: 1|2|3|league|regional|showcase" >&2
        exit 2
      fi
      MODE_ARG="$2"
      shift 2
      ;;
    --no-prompt)
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      LAUNCH_ARGS=("$@")
      break
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

source_ros_workspace "${ROOT_DIR}"
cleanup_existing_launch_tree "${CLEANUP_EXISTING}" "${STACK_LAUNCH_REGEX}"

if ! has_launch_arg_key "base_config_file"; then
  LAUNCH_ARGS=("base_config_file:=${DEFAULT_BASE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default base_config_file=${DEFAULT_BASE_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == base_config_file:=* ]] && echo "[INFO] override base_config_file=${arg#base_config_file:=}"; done
fi

if ! has_launch_arg_key "detector_config_file"; then
  LAUNCH_ARGS=("detector_config_file:=${DEFAULT_DETECTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default detector_config_file=${DEFAULT_DETECTOR_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == detector_config_file:=* ]] && echo "[INFO] override detector_config_file=${arg#detector_config_file:=}"; done
fi

if ! has_launch_arg_key "predictor_config_file"; then
  LAUNCH_ARGS=("predictor_config_file:=${DEFAULT_PREDICTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default predictor_config_file=${DEFAULT_PREDICTOR_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == predictor_config_file:=* ]] && echo "[INFO] override predictor_config_file=${arg#predictor_config_file:=}"; done
fi

if ! has_launch_arg_key "outpost_config_file"; then
  LAUNCH_ARGS=("outpost_config_file:=${DEFAULT_OUTPOST_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default outpost_config_file=${DEFAULT_OUTPOST_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == outpost_config_file:=* ]] && echo "[INFO] override outpost_config_file=${arg#outpost_config_file:=}"; done
fi

if ! has_launch_arg_key "buff_config_file"; then
  LAUNCH_ARGS=("buff_config_file:=${DEFAULT_BUFF_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default buff_config_file=${DEFAULT_BUFF_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == buff_config_file:=* ]] && echo "[INFO] override buff_config_file=${arg#buff_config_file:=}"; done
fi

if ! has_launch_arg_key "config_file"; then
  LAUNCH_ARGS=("config_file:=${DEFAULT_OVERRIDE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default config_file(override)=${DEFAULT_OVERRIDE_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == config_file:=* ]] && echo "[INFO] override config_file=${arg#config_file:=}"; done
fi

if [[ -n "${MODE_ARG}" ]]; then
  LAUNCH_ARGS=("mode:=${MODE_ARG}" "${LAUNCH_ARGS[@]}")
fi
if (( OFFLINE_MODE == 1 )); then
  LAUNCH_ARGS=("offline:=true" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch behavior_tree sentry_all.launch.py "${LAUNCH_ARGS[@]}"
