#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
ENABLE_FIRE=1
FIRE_HZ="20.0"
LAUNCH_ARGS=()

DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
DEFAULT_DETECTOR_CONFIG_FILE="${ROOT_DIR}/src/detector/config/detector_config.yaml"
DEFAULT_BUFF_CONFIG_FILE="${ROOT_DIR}/src/buff_hitter/config/buff_config.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--fire|--no-fire] [--fire-hz <hz>] [-- <launch_args...>]

Purpose:
  Pure buff test:
  - no behavior_tree
  - no patrol strategy
  - force /ly/aa/enable=false and /ly/ra/enable=true
  - bridge /ly/buff/target -> /ly/control/* for gimbal debug/firing

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --no-fire
  ./${SCRIPT_NAME} --fire-hz 15
  ./${SCRIPT_NAME} -- bridge_timeout_sec:=1.2 detector_config.show:=true
EOF
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
    --fire)
      ENABLE_FIRE=1
      shift
      ;;
    --no-fire)
      ENABLE_FIRE=0
      shift
      ;;
    --fire-hz)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --fire-hz requires a value" >&2
        exit 2
      fi
      FIRE_HZ="$2"
      shift 2
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
cleanup_existing_stack "1" "/(gimbal_driver_node|detector_node|buff_hitter_node|behavior_tree_node|buff_test_bridge)([[:space:]]|$)" "ros2 launch (detector|behavior_tree) (buff_test|buff|sentry_all|competition_autoaim|showcase|chase_only)\\.launch.py"

if [[ -f "${DEFAULT_BASE_CONFIG_FILE}" ]] && ! has_launch_arg_key "base_config_file"; then
  LAUNCH_ARGS=("base_config_file:=${DEFAULT_BASE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_DETECTOR_CONFIG_FILE}" ]] && ! has_launch_arg_key "detector_config_file"; then
  LAUNCH_ARGS=("detector_config_file:=${DEFAULT_DETECTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_BUFF_CONFIG_FILE}" ]] && ! has_launch_arg_key "buff_config_file"; then
  LAUNCH_ARGS=("buff_config_file:=${DEFAULT_BUFF_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_OVERRIDE_CONFIG_FILE}" ]] && ! has_launch_arg_key "config_file"; then
  LAUNCH_ARGS=("config_file:=${DEFAULT_OVERRIDE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "bridge_enable_fire"; then
  if (( ENABLE_FIRE == 1 )); then
    LAUNCH_ARGS=("bridge_enable_fire:=true" "${LAUNCH_ARGS[@]}")
  else
    LAUNCH_ARGS=("bridge_enable_fire:=false" "${LAUNCH_ARGS[@]}")
  fi
fi
if ! has_launch_arg_key "bridge_fire_hz"; then
  LAUNCH_ARGS=("bridge_fire_hz:=${FIRE_HZ}" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch detector buff_test.launch.py "${LAUNCH_ARGS[@]}"
