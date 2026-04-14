#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

RAW_TOPIC="/ly/navi/goal_pos_raw"
GOAL_TOPIC="/ly/navi/goal_pos"
RAW_FRAME="map"
MAP_FRAME=""
BASE_FRAME=""
FALLBACK_BASE_FRAME=""
INPUT_UNIT="cm"
LAUNCH_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <extra launch args...>]

Purpose:
  Manual TF coordinate conversion tool.
  - Start navi_tf_bridge/manual_goal_tf_bridge.launch.py in background
  - Ask for x y continuously in terminal
  - Publish raw input to /ly/navi/goal_pos_raw
  - Read converted /ly/navi/goal_pos output in terminal

Options:
  --raw-topic <topic>               (default: /ly/navi/goal_pos_raw)
  --goal-topic <topic>              (default: /ly/navi/goal_pos)
  --raw-frame <frame>               (default: map)
  --map-frame <frame>               (default from tf_config.yaml)
  --base-frame <frame>              (default from tf_config.yaml)
  --fallback-base-frame <frame>     (default from tf_config.yaml)
  --input-unit cm|m                 (default: cm)
  --help

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --raw-frame world
  ./${SCRIPT_NAME} --input-unit m
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
    --raw-topic)
      RAW_TOPIC="${2:-}"
      shift 2
      ;;
    --goal-topic)
      GOAL_TOPIC="${2:-}"
      shift 2
      ;;
    --raw-frame)
      RAW_FRAME="${2:-}"
      shift 2
      ;;
    --map-frame)
      MAP_FRAME="${2:-}"
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
    --input-unit)
      INPUT_UNIT="${2:-}"
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

if [[ -z "${RAW_TOPIC}" || -z "${GOAL_TOPIC}" || -z "${RAW_FRAME}" ]]; then
  echo "[ERROR] --raw-topic/--goal-topic/--raw-frame cannot be empty." >&2
  exit 1
fi
if [[ "${INPUT_UNIT}" != "cm" && "${INPUT_UNIT}" != "m" ]]; then
  echo "[ERROR] --input-unit must be 'cm' or 'm'." >&2
  exit 1
fi

source_ros_workspace "${ROOT_DIR}"
cleanup_existing_stack "1" "/(target_rel_to_goal_pos_node|manual_goal_input_node)([[:space:]]|$)" "ros2 launch navi_tf_bridge (manual_goal_tf_bridge|target_rel_to_goal_pos)\\.launch.py"

if ! has_launch_arg_key "input_goal_pos_raw_topic"; then
  LAUNCH_ARGS=("input_goal_pos_raw_topic:=${RAW_TOPIC}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "output_goal_pos_topic"; then
  LAUNCH_ARGS=("output_goal_pos_topic:=${GOAL_TOPIC}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "goal_pos_raw_frame"; then
  LAUNCH_ARGS=("goal_pos_raw_frame:=${RAW_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if [[ -n "${MAP_FRAME}" ]] && ! has_launch_arg_key "map_frame"; then
  LAUNCH_ARGS=("map_frame:=${MAP_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if [[ -n "${BASE_FRAME}" ]] && ! has_launch_arg_key "base_frame"; then
  LAUNCH_ARGS=("base_frame:=${BASE_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if [[ -n "${FALLBACK_BASE_FRAME}" ]] && ! has_launch_arg_key "fallback_base_frame"; then
  LAUNCH_ARGS=("fallback_base_frame:=${FALLBACK_BASE_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "debug_export_point_pairs"; then
  LAUNCH_ARGS=("debug_export_point_pairs:=false" "${LAUNCH_ARGS[@]}")
fi

echo "[INFO] Launch TF bridge with args: ${LAUNCH_ARGS[*]}"
ros2 launch navi_tf_bridge manual_goal_tf_bridge.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

cleanup() {
  local rc=$?
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true
  exit "${rc}"
}
trap cleanup EXIT INT TERM

sleep 1
echo "[INFO] Start manual input node. Type q to quit."
ros2 run navi_tf_bridge manual_goal_input_node --ros-args \
  -p raw_topic:="${RAW_TOPIC}" \
  -p goal_topic:="${GOAL_TOPIC}" \
  -p input_unit:="${INPUT_UNIT}"
