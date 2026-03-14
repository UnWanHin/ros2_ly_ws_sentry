#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
DEFAULT_CONFIG_FILE="${ROOT_DIR}/scripts/config/auto_aim_config_competition.yaml"
USE_NOGATE=1
START_ARGS=()
LAUNCH_ARGS=()

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--nogate|--with-gate] [start_sentry_all.sh options] [-- <launch_args...>]

Purpose:
  Launch competition-like autoaim only:
    - gimbal_driver + detector + tracker_solver + predictor + behavior_tree
    - disable buff_hitter / outpost_hitter by default
    - default real-camera config
    - default nogate for field/debug testing

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --mode league
  ./${SCRIPT_NAME} --offline
  ./${SCRIPT_NAME} --with-gate
  ./${SCRIPT_NAME} -- use_buff:=true use_outpost:=true
EOF
}

has_launch_arg_key() {
  local key="$1"
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "${arg}" == "${key}:="* ]] || [[ "${arg}" == "--${key}:="* ]]; then
      return 0
    fi
  done
  return 1
}

append_default_launch_arg() {
  local key="$1"
  local value="$2"
  if ! has_launch_arg_key "${key}"; then
    LAUNCH_ARGS+=("${key}:=${value}")
  fi
}

has_start_arg() {
  local key="$1"
  local arg
  for arg in "${START_ARGS[@]}"; do
    if [[ "${arg}" == "${key}" ]]; then
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
      START_ARGS+=("$1")
      shift
      ;;
  esac
done

if (( USE_NOGATE == 1 )); then
  TARGET_SCRIPT="${ROOT_DIR}/scripts/start_sentry_all_nogate.sh"
else
  TARGET_SCRIPT="${ROOT_DIR}/scripts/start_sentry_all.sh"
fi

append_default_launch_arg "config_file" "${DEFAULT_CONFIG_FILE}"
append_default_launch_arg "use_gimbal" "true"
append_default_launch_arg "use_detector" "true"
append_default_launch_arg "use_tracker" "true"
append_default_launch_arg "use_predictor" "true"
append_default_launch_arg "use_outpost" "false"
append_default_launch_arg "use_buff" "false"
append_default_launch_arg "use_behavior_tree" "true"

cd "${ROOT_DIR}"

if ! has_start_arg "--mode"; then
  START_ARGS+=("--mode" "regional")
fi

exec "${TARGET_SCRIPT}" \
  "${START_ARGS[@]}" \
  --no-prompt \
  -- \
  "${LAUNCH_ARGS[@]}"
