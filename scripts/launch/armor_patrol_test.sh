#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
USE_NOGATE=1
OFFLINE_MODE=0
LAUNCH_ARGS=()
DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
DEFAULT_DETECTOR_CONFIG_FILE="${ROOT_DIR}/src/detector/config/detector_config.yaml"
DEFAULT_PREDICTOR_CONFIG_FILE="${ROOT_DIR}/src/predictor/config/predictor_config.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--nogate|--with-gate] [--online|--offline] [-- <launch_args...>]

Purpose:
  Armor-style patrol preset without firing.
  - behavior_tree config: Scripts/ConfigJson/armor_patrol_test.json
  - fire disabled, rotate enabled, gimbal patrol enabled
  - default publish_navi_goal=false to keep this mode static-focused
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
    --nogate)
      USE_NOGATE=1
      shift
      ;;
    --with-gate)
      USE_NOGATE=0
      shift
      ;;
    --online)
      OFFLINE_MODE=0
      shift
      ;;
    --offline)
      OFFLINE_MODE=1
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
cleanup_existing_stack "1" "/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|outpost_hitter_node|buff_hitter_node|behavior_tree_node)([[:space:]]|$)" "ros2 launch behavior_tree (armor_patrol_test|competition_autoaim|sentry_all|chase_only|showcase)\\.launch.py"

if [[ -f "${DEFAULT_BASE_CONFIG_FILE}" ]] && ! has_launch_arg_key "base_config_file"; then
  LAUNCH_ARGS=("base_config_file:=${DEFAULT_BASE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi

if [[ -f "${DEFAULT_DETECTOR_CONFIG_FILE}" ]] && ! has_launch_arg_key "detector_config_file"; then
  LAUNCH_ARGS=("detector_config_file:=${DEFAULT_DETECTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi

if [[ -f "${DEFAULT_PREDICTOR_CONFIG_FILE}" ]] && ! has_launch_arg_key "predictor_config_file"; then
  LAUNCH_ARGS=("predictor_config_file:=${DEFAULT_PREDICTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi

if [[ -f "${DEFAULT_OVERRIDE_CONFIG_FILE}" ]] && ! has_launch_arg_key "config_file"; then
  LAUNCH_ARGS=("config_file:=${DEFAULT_OVERRIDE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi

if (( USE_NOGATE == 1 )) && ! has_launch_arg_key "debug_bypass_is_start"; then
  LAUNCH_ARGS=("debug_bypass_is_start:=true" "${LAUNCH_ARGS[@]}")
fi
if (( USE_NOGATE == 0 )) && ! has_launch_arg_key "debug_bypass_is_start"; then
  LAUNCH_ARGS=("debug_bypass_is_start:=false" "${LAUNCH_ARGS[@]}")
fi
if (( OFFLINE_MODE == 1 )) && ! has_launch_arg_key "offline"; then
  LAUNCH_ARGS=("offline:=true" "${LAUNCH_ARGS[@]}")
fi

if ! has_launch_arg_key "detector_config.show" && ! has_launch_arg_key "detector_config/show"; then
  LAUNCH_ARGS=("detector_config.show:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "detector_config.draw" && ! has_launch_arg_key "detector_config/draw"; then
  LAUNCH_ARGS=("detector_config.draw:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "detector_config.web_show" && ! has_launch_arg_key "detector_config/web_show"; then
  LAUNCH_ARGS=("detector_config.web_show:=true" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch behavior_tree armor_patrol_test.launch.py "${LAUNCH_ARGS[@]}"
