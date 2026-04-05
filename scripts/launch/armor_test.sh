#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
USE_NOGATE=1
DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/scripts/config/stack/base_competition.yaml"
DEFAULT_DETECTOR_CONFIG_FILE="${ROOT_DIR}/scripts/config/stack/detector_competition.yaml"
DEFAULT_PREDICTOR_CONFIG_FILE="${ROOT_DIR}/scripts/config/stack/predictor_competition.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/scripts/config/stack/override_none.yaml"
LAUNCH_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--nogate|--with-gate] [-- <launch_args...>]

Purpose:
  Thin wrapper for behavior_tree/competition_autoaim.launch.py.
  Defaults to the competition config and enables detector visualization for test runs.
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
cleanup_existing_stack "1" "/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|outpost_hitter_node|buff_hitter_node|behavior_tree_node)([[:space:]]|$)" "ros2 launch behavior_tree (competition_autoaim|sentry_all)\\.launch.py"

if (( USE_NOGATE == 1 )); then
  LAUNCH_ARGS=("debug_bypass_is_start:=true" "${LAUNCH_ARGS[@]}")
else
  LAUNCH_ARGS=("debug_bypass_is_start:=false" "${LAUNCH_ARGS[@]}")
fi

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

if ! has_launch_arg_key "detector_config.show" && ! has_launch_arg_key "detector_config/show"; then
  LAUNCH_ARGS=("detector_config.show:=true" "${LAUNCH_ARGS[@]}")
fi

if ! has_launch_arg_key "detector_config.draw" && ! has_launch_arg_key "detector_config/draw"; then
  LAUNCH_ARGS=("detector_config.draw:=true" "${LAUNCH_ARGS[@]}")
fi

if ! has_launch_arg_key "detector_config.web_show" && ! has_launch_arg_key "detector_config/web_show"; then
  LAUNCH_ARGS=("detector_config.web_show:=true" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch behavior_tree competition_autoaim.launch.py "${LAUNCH_ARGS[@]}"
