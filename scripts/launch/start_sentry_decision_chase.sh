#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
USE_NOGATE=1
OFFLINE_MODE=0
MODE_ARG="league"
BASE_FRAME="base_link"
FALLBACK_BASE_FRAME="baselink"
MAP_FRAME="map"
PUBLISH_TARGET_MAP=1
USE_MSG_FRAME_ID=1
LAUNCH_ARGS=()
DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
DEFAULT_DETECTOR_CONFIG_FILE="${ROOT_DIR}/src/detector/config/detector_config.yaml"
DEFAULT_PREDICTOR_CONFIG_FILE="${ROOT_DIR}/src/predictor/config/predictor_config.yaml"
DEFAULT_OUTPOST_CONFIG_FILE="${ROOT_DIR}/src/outpost_hitter/config/outpost_config.yaml"
DEFAULT_BUFF_CONFIG_FILE="${ROOT_DIR}/src/buff_hitter/config/buff_config.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <extra launch args...>]

Purpose:
  Start decision-chase test stack:
  1) behavior_tree/chase_only.launch.py
  2) navi_tf_bridge/target_rel_to_goal_pos_node

Options:
  --nogate | --with-gate
  --online | --offline
  --mode league|regional|showcase
  --base-frame <frame>              (default: base_link)
  --fallback-base-frame <frame>     (default: baselink)
  --map-frame <frame>               (default: map)
  --publish-target-map | --no-publish-target-map
  --use-msg-frame-id | --ignore-msg-frame-id
  --help

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --base-frame baselink
  ./${SCRIPT_NAME} --no-publish-target-map -- --detector_config.show:=true
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
    --mode)
      MODE_ARG="${2:-}"
      shift 2
      ;;
    --base-frame)
      BASE_FRAME="${2:-}"
      shift 2
      ;;
    --fallback-base-frame)
      FALLBACK_BASE_FRAME="${2:-}"
      shift 2
      ;;
    --map-frame)
      MAP_FRAME="${2:-}"
      shift 2
      ;;
    --publish-target-map)
      PUBLISH_TARGET_MAP=1
      shift
      ;;
    --no-publish-target-map)
      PUBLISH_TARGET_MAP=0
      shift
      ;;
    --use-msg-frame-id)
      USE_MSG_FRAME_ID=1
      shift
      ;;
    --ignore-msg-frame-id)
      USE_MSG_FRAME_ID=0
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
cleanup_existing_stack "1" \
  "/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|outpost_hitter_node|buff_hitter_node|behavior_tree_node|target_rel_to_goal_pos_node)([[:space:]]|$)" \
  "ros2 launch (behavior_tree|navi_tf_bridge) (sentry_all|competition_autoaim|showcase|chase_only|decision_chase|target_rel_to_goal_pos)\\.launch.py"

if [[ -f "${DEFAULT_BASE_CONFIG_FILE}" ]] && ! has_launch_arg_key "base_config_file"; then
  LAUNCH_ARGS=("base_config_file:=${DEFAULT_BASE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_DETECTOR_CONFIG_FILE}" ]] && ! has_launch_arg_key "detector_config_file"; then
  LAUNCH_ARGS=("detector_config_file:=${DEFAULT_DETECTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_PREDICTOR_CONFIG_FILE}" ]] && ! has_launch_arg_key "predictor_config_file"; then
  LAUNCH_ARGS=("predictor_config_file:=${DEFAULT_PREDICTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_OUTPOST_CONFIG_FILE}" ]] && ! has_launch_arg_key "outpost_config_file"; then
  LAUNCH_ARGS=("outpost_config_file:=${DEFAULT_OUTPOST_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_BUFF_CONFIG_FILE}" ]] && ! has_launch_arg_key "buff_config_file"; then
  LAUNCH_ARGS=("buff_config_file:=${DEFAULT_BUFF_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_OVERRIDE_CONFIG_FILE}" ]] && ! has_launch_arg_key "config_file"; then
  LAUNCH_ARGS=("config_file:=${DEFAULT_OVERRIDE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi

LAUNCH_ARGS=("mode:=${MODE_ARG}" "${LAUNCH_ARGS[@]}")
LAUNCH_ARGS=("map_frame:=${MAP_FRAME}" "${LAUNCH_ARGS[@]}")
LAUNCH_ARGS=("base_frame:=${BASE_FRAME}" "${LAUNCH_ARGS[@]}")
LAUNCH_ARGS=("fallback_base_frame:=${FALLBACK_BASE_FRAME}" "${LAUNCH_ARGS[@]}")
if (( PUBLISH_TARGET_MAP == 1 )); then
  LAUNCH_ARGS=("publish_target_map:=true" "${LAUNCH_ARGS[@]}")
else
  LAUNCH_ARGS=("publish_target_map:=false" "${LAUNCH_ARGS[@]}")
fi
if (( USE_MSG_FRAME_ID == 1 )); then
  LAUNCH_ARGS=("use_msg_frame_id:=true" "${LAUNCH_ARGS[@]}")
else
  LAUNCH_ARGS=("use_msg_frame_id:=false" "${LAUNCH_ARGS[@]}")
fi
if (( OFFLINE_MODE == 1 )); then
  LAUNCH_ARGS=("offline:=true" "${LAUNCH_ARGS[@]}")
fi
if (( USE_NOGATE == 1 )); then
  LAUNCH_ARGS=("debug_bypass_is_start:=true" "${LAUNCH_ARGS[@]}")
else
  LAUNCH_ARGS=("debug_bypass_is_start:=false" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch navi_tf_bridge decision_chase.launch.py "${LAUNCH_ARGS[@]}"
