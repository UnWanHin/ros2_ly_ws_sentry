#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_DIR="${ROOT_DIR}/scripts/feature_test"
SCRIPT_NAME="$(basename "$0")"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/common.sh"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/guard.sh"

CONFIG_FILE="${ROOT_DIR}/scripts/config/sentry_feature_test.yaml"
LOCK_FILE="/tmp/sentry_feature_test.lock"
LOG_DIR="/tmp/ros2_feature_test_logs"
DRY_RUN=0

OVERRIDE_ALLOW_MULTI_CONTROL=""
OVERRIDE_CLEANUP_EXISTING=""

declare -a CHILD_PIDS=()
declare -a CHILD_LABELS=()
declare -a CHILD_LOGS=()

FT_CMD_TIMEOUT_SEC=5

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--config FILE] [--dry-run] [--allow-multi-control] [--cleanup-existing|--no-cleanup-existing]

Description:
  配置驱动的 BT 外功能测试入口（Phase 1）。
  当前稳定支持：
    - 云台: armor / scan
    - 底盘: velocity

Examples:
  # 默认配置运行
  ./scripts/feature_test/run_feature_test.sh

  # 只检查将要执行的命令
  ./scripts/feature_test/run_feature_test.sh --dry-run

  # 指定配置
  ./scripts/feature_test/run_feature_test.sh --config ./scripts/config/sentry_feature_test.yaml
EOF
}

cleanup_children() {
  trap - EXIT INT TERM
  local idx
  for ((idx=${#CHILD_PIDS[@]} - 1; idx >= 0; idx--)); do
    local pid="${CHILD_PIDS[$idx]}"
    local label="${CHILD_LABELS[$idx]}"
    if kill -0 "${pid}" 2>/dev/null; then
      ft_info "Stopping ${label} (PID=${pid})"
      kill -INT "${pid}" 2>/dev/null || true
      sleep 1
      kill -TERM "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  ft_release_lock "${LOCK_FILE}"
}

trap cleanup_children EXIT INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --allow-multi-control)
      OVERRIDE_ALLOW_MULTI_CONTROL="1"
      shift
      ;;
    --cleanup-existing)
      OVERRIDE_CLEANUP_EXISTING="1"
      shift
      ;;
    --no-cleanup-existing)
      OVERRIDE_CLEANUP_EXISTING="0"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      ft_err "Unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

if [[ ! -f "${CONFIG_FILE}" ]]; then
  ft_err "Config file not found: ${CONFIG_FILE}"
  exit 2
fi

load_config_env() {
  python3 - "${CONFIG_FILE}" <<'PY'
import shlex
import sys
import yaml

cfg_path = sys.argv[1]
with open(cfg_path, "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}

def get(path, default):
    cur = cfg
    for key in path.split("."):
        if isinstance(cur, dict) and key in cur:
            cur = cur[key]
        else:
            return default
    return cur

def to_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "y", "on"}:
            return True
        if lowered in {"0", "false", "no", "n", "off"}:
            return False
    return default

def emit(name, value):
    if isinstance(value, bool):
        out = "1" if value else "0"
    elif isinstance(value, list):
        out = ",".join(str(x) for x in value)
    elif value is None:
        out = ""
    else:
        out = str(value)
    print(f"{name}={shlex.quote(out)}")

emit("CFG_GLOBAL_OFFLINE", to_bool(get("global.offline", True), True))
emit("CFG_GLOBAL_CLEANUP_EXISTING", to_bool(get("global.cleanup_existing", True), True))
emit("CFG_GLOBAL_ALLOW_MULTI_CONTROL", to_bool(get("global.allow_multi_control", False), False))
emit("CFG_GLOBAL_WAIT_STACK_SEC", get("global.wait_stack_sec", 12))
emit("CFG_GLOBAL_CMD_TIMEOUT_SEC", get("global.cmd_timeout_sec", 5))
emit("CFG_GLOBAL_CONFIG_FILE", get("global.config_file", ""))

emit("CFG_RUN_GIMBAL", to_bool(get("run.gimbal", True), True))
emit("CFG_RUN_CHASSIS", to_bool(get("run.chassis", False), False))

emit("CFG_GIMBAL_MODE_ARMOR", to_bool(get("gimbal.mode.armor", True), True))
emit("CFG_GIMBAL_MODE_OUTPOST", to_bool(get("gimbal.mode.outpost", False), False))
emit("CFG_GIMBAL_MODE_BUFF", to_bool(get("gimbal.mode.buff", False), False))
emit("CFG_GIMBAL_MODE_SCAN", to_bool(get("gimbal.mode.scan", False), False))
emit("CFG_GIMBAL_FIRE_ENABLE", to_bool(get("gimbal.fire.enable", True), True))
emit("CFG_GIMBAL_FIRE_AUTO_FIRE", to_bool(get("gimbal.fire.auto_fire", True), True))
emit("CFG_GIMBAL_TARGET_PRIORITY", get("gimbal.target.priority", [6, 3, 4, 5]))
emit("CFG_GIMBAL_TARGET_FALLBACK_ID", get("gimbal.target.fallback_id", 6))
emit("CFG_GIMBAL_TARGET_TIMEOUT_SEC", get("gimbal.target.timeout_sec", 1.0))
emit("CFG_GIMBAL_DIAG_PERIOD", get("gimbal.diag_period", 1.0))
emit("CFG_GIMBAL_SCAN_YAW_MIN", get("gimbal.scan.yaw_min", -15.0))
emit("CFG_GIMBAL_SCAN_YAW_MAX", get("gimbal.scan.yaw_max", 15.0))
emit("CFG_GIMBAL_SCAN_PITCH", get("gimbal.scan.pitch", 8.0))
emit("CFG_GIMBAL_SCAN_STEP_DEG", get("gimbal.scan.step_deg", 1.0))
emit("CFG_GIMBAL_SCAN_HZ", get("gimbal.scan.hz", 20.0))

emit("CFG_CHASSIS_MODE_VELOCITY", to_bool(get("chassis.mode.velocity", False), False))
emit("CFG_CHASSIS_MODE_SPIN", to_bool(get("chassis.mode.spin", False), False))
emit("CFG_CHASSIS_MODE_NAV_GOAL", to_bool(get("chassis.mode.nav_goal", False), False))
emit("CFG_CHASSIS_MODE_PATROL", to_bool(get("chassis.mode.patrol", False), False))
emit("CFG_CHASSIS_VELOCITY_PROFILE", get("chassis.velocity.profile", "constant"))
emit("CFG_CHASSIS_VELOCITY_SPEED_X", get("chassis.velocity.speed_x", 20))
emit("CFG_CHASSIS_VELOCITY_SPEED_Y", get("chassis.velocity.speed_y", 0))
emit("CFG_CHASSIS_VELOCITY_PUBLISH_HZ", get("chassis.velocity.publish_hz", 20.0))
emit("CFG_CHASSIS_VELOCITY_PERIOD_SEC", get("chassis.velocity.period_sec", 8.0))
PY
}

eval "$(load_config_env)"

if [[ -n "${OVERRIDE_ALLOW_MULTI_CONTROL}" ]]; then
  CFG_GLOBAL_ALLOW_MULTI_CONTROL="${OVERRIDE_ALLOW_MULTI_CONTROL}"
fi
if [[ -n "${OVERRIDE_CLEANUP_EXISTING}" ]]; then
  CFG_GLOBAL_CLEANUP_EXISTING="${OVERRIDE_CLEANUP_EXISTING}"
fi

FT_CMD_TIMEOUT_SEC="${CFG_GLOBAL_CMD_TIMEOUT_SEC}"

validate_config() {
  if (( CFG_RUN_GIMBAL == 0 && CFG_RUN_CHASSIS == 0 )); then
    ft_err "Config invalid: run.gimbal and run.chassis cannot both be false."
    exit 2
  fi

  if (( CFG_RUN_GIMBAL == 1 )); then
    local gimbal_mode_count
    gimbal_mode_count=$((CFG_GIMBAL_MODE_ARMOR + CFG_GIMBAL_MODE_OUTPOST + CFG_GIMBAL_MODE_BUFF + CFG_GIMBAL_MODE_SCAN))
    if (( gimbal_mode_count != 1 )); then
      ft_err "Config invalid: exactly one gimbal.mode.* must be true when run.gimbal=true."
      exit 2
    fi

    if (( CFG_GIMBAL_MODE_OUTPOST == 1 || CFG_GIMBAL_MODE_BUFF == 1 )); then
      ft_err "Current Phase 1 implementation supports gimbal mode: armor / scan only."
      exit 2
    fi
  fi

  if (( CFG_RUN_CHASSIS == 1 )); then
    if (( CFG_CHASSIS_MODE_VELOCITY == 0 )); then
      ft_err "Current Phase 1 implementation supports chassis mode: velocity only."
      exit 2
    fi
    if (( CFG_CHASSIS_MODE_SPIN == 1 || CFG_CHASSIS_MODE_NAV_GOAL == 1 || CFG_CHASSIS_MODE_PATROL == 1 )); then
      ft_warn "spin/nav_goal/patrol are not enabled in this Phase 1 runtime. They are ignored."
    fi
  fi
}

cleanup_stale_feature_processes() {
  if (( CFG_GLOBAL_CLEANUP_EXISTING == 0 )); then
    return
  fi
  pkill -f "${ROOT_DIR}/scripts/feature_test/scan_gimbal_test.py" >/dev/null 2>&1 || true
  pkill -f "${ROOT_DIR}/scripts/feature_test/chassis_vel_test.py" >/dev/null 2>&1 || true
}

start_bg() {
  local label="$1"
  shift
  local cmd=("$@")
  local ts
  ts="$(date +%Y%m%d_%H%M%S)"
  local log_file="${LOG_DIR}/${label}_${ts}.log"
  ft_info "${label} cmd: $(ft_print_cmd "${cmd[@]}")"
  if (( DRY_RUN == 1 )); then
    return 0
  fi
  "${cmd[@]}" >"${log_file}" 2>&1 &
  local pid="$!"
  CHILD_PIDS+=("${pid}")
  CHILD_LABELS+=("${label}")
  CHILD_LOGS+=("${log_file}")
  ft_info "Started ${label}, pid=${pid}, log=${log_file}"
}

validate_config

mkdir -p "${LOG_DIR}"
: "${ROS_LOG_DIR:=/tmp/ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

ft_source_ros
ft_source_workspace "${ROOT_DIR}"
cd "${ROOT_DIR}"

if (( DRY_RUN == 0 )); then
  ft_acquire_lock "${LOCK_FILE}"
fi

cleanup_stale_feature_processes

declare -a guard_topics=()
if (( CFG_RUN_GIMBAL == 1 )); then
  guard_topics+=("/ly/control/angles" "/ly/control/firecode")
fi
if (( CFG_RUN_CHASSIS == 1 )) && (( CFG_CHASSIS_MODE_VELOCITY == 1 )); then
  guard_topics+=("/ly/control/vel")
fi

if (( CFG_GLOBAL_ALLOW_MULTI_CONTROL == 0 )); then
  ft_require_behavior_tree_absent
  local_topic=""
  for local_topic in "${guard_topics[@]}"; do
    ft_require_topic_no_publisher "${local_topic}"
  done
else
  ft_warn "allow_multi_control=true: single-controller guard is disabled by config."
fi

if (( CFG_RUN_GIMBAL == 1 )) && (( CFG_GIMBAL_MODE_ARMOR == 1 )); then
  gimbal_cmd=(
    "${ROOT_DIR}/scripts/launch/start_autoaim_debug.sh"
    --mode fire
    --wait "${CFG_GLOBAL_WAIT_STACK_SEC}"
    --cmd-timeout "${CFG_GLOBAL_CMD_TIMEOUT_SEC}"
    --target-id "${CFG_GIMBAL_TARGET_FALLBACK_ID}"
    --target-priority "${CFG_GIMBAL_TARGET_PRIORITY}"
    --publish-team false
    --enable-fire "$(ft_bool_to_word "${CFG_GIMBAL_FIRE_ENABLE}")"
    --auto-fire "$(ft_bool_to_word "${CFG_GIMBAL_FIRE_AUTO_FIRE}")"
    --target-timeout "${CFG_GIMBAL_TARGET_TIMEOUT_SEC}"
    --diag-period "${CFG_GIMBAL_DIAG_PERIOD}"
  )
  if (( CFG_GLOBAL_OFFLINE == 1 )); then
    gimbal_cmd+=(--offline)
  else
    gimbal_cmd+=(--online)
  fi
  if [[ -n "${CFG_GLOBAL_CONFIG_FILE}" ]]; then
    gimbal_cmd+=(--config-file "${CFG_GLOBAL_CONFIG_FILE}")
  fi
  if (( CFG_GLOBAL_CLEANUP_EXISTING == 1 )); then
    gimbal_cmd+=(--cleanup-existing)
  else
    gimbal_cmd+=(--no-cleanup-existing)
  fi
  if (( CFG_GLOBAL_ALLOW_MULTI_CONTROL == 1 )); then
    gimbal_cmd+=(--allow-multi-control)
  fi
  start_bg "gimbal_armor" "${gimbal_cmd[@]}"
fi

if (( CFG_RUN_GIMBAL == 1 )) && (( CFG_GIMBAL_MODE_SCAN == 1 )); then
  scan_cmd=(
    python3 "${ROOT_DIR}/scripts/feature_test/scan_gimbal_test.py"
    --yaw-min "${CFG_GIMBAL_SCAN_YAW_MIN}"
    --yaw-max "${CFG_GIMBAL_SCAN_YAW_MAX}"
    --pitch "${CFG_GIMBAL_SCAN_PITCH}"
    --step-deg "${CFG_GIMBAL_SCAN_STEP_DEG}"
    --hz "${CFG_GIMBAL_SCAN_HZ}"
    --angles-topic "/ly/control/angles"
    --firecode-topic "/ly/control/firecode"
    --safe-firecode 0
  )
  start_bg "gimbal_scan" "${scan_cmd[@]}"
fi

if (( CFG_RUN_CHASSIS == 1 )) && (( CFG_CHASSIS_MODE_VELOCITY == 1 )); then
  vel_cmd=(
    python3 "${ROOT_DIR}/scripts/feature_test/chassis_vel_test.py"
    --profile "${CFG_CHASSIS_VELOCITY_PROFILE}"
    --speed-x "${CFG_CHASSIS_VELOCITY_SPEED_X}"
    --speed-y "${CFG_CHASSIS_VELOCITY_SPEED_Y}"
    --hz "${CFG_CHASSIS_VELOCITY_PUBLISH_HZ}"
    --period-sec "${CFG_CHASSIS_VELOCITY_PERIOD_SEC}"
    --topic "/ly/control/vel"
  )
  start_bg "chassis_velocity" "${vel_cmd[@]}"
fi

if (( DRY_RUN == 1 )); then
  ft_info "Dry-run done."
  exit 0
fi

if (( ${#CHILD_PIDS[@]} == 0 )); then
  ft_err "No feature process started. Check configuration."
  exit 2
fi

if (( CFG_GLOBAL_ALLOW_MULTI_CONTROL == 0 )); then
  for local_topic in "${guard_topics[@]}"; do
    ft_wait_and_require_topic_single_publisher "${local_topic}" 12 1
  done
  ft_info "Single-controller guard passed for topics: ${guard_topics[*]}"
fi

ft_info "Feature test is running. Press Ctrl+C to stop."

set +e
wait -n "${CHILD_PIDS[@]}"
first_rc=$?
set -e

if (( first_rc != 0 )); then
  ft_err "A feature-test process exited with code ${first_rc}. Stopping all."
  exit "${first_rc}"
fi

ft_warn "A feature-test process exited. Stopping all."
exit 1
