#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
LOCK_FILE="/tmp/sentry_autoaim_debug.lock"
ONLINE_CONFIG_FILE="${ROOT_DIR}/scripts/config/auto_aim_config_competition.yaml"

: "${ROS_LOG_DIR:=/tmp/ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

MODE="fire"
OFFLINE_MODE=1
CLEANUP_EXISTING=1
ALLOW_MULTI_CONTROL=0
WAIT_SECONDS=12
CMD_TIMEOUT=5
OUTPUT_MODE="screen"
CONFIG_FILE=""
DRY_RUN=0
LAUNCH_ARGS=()

MAPPER_RED="true"
MAPPER_TARGET_ID="6"
MAPPER_TARGET_PRIORITY="6,3,4,5"
MAPPER_PUBLISH_TEAM="false"
MAPPER_PUBLISH_TARGET="true"
MAPPER_ENABLE_FIRE="true"
MAPPER_AUTO_FIRE="true"
MAPPER_TARGET_TIMEOUT="1.0"
MAPPER_DIAG_PERIOD="1.0"
MAPPER_ANGLES_TOPIC=""
MAPPER_FIRECODE_TOPIC=""
MAPPER_BT_TARGET_TOPIC="/ly/bt/target"

LAUNCH_PID=""
TAIL_PID=""
LAUNCH_LOG=""
LAUNCH_CMD=()
MAPPER_CMD=()
LOCK_ACQUIRED=0

STACK_NODE_REGEX="/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|mapper_node|target_to_gimbal_mapper|fire_flip_test)([[:space:]]|$)"
STACK_LAUNCH_REGEX="ros2 launch detector auto_aim.launch.py"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <auto_aim_launch_args...>]

Modes:
  --mode perception      仅拉起 auto_aim 感知链
  --mode mapper          仅拉起 mapper_node（要求感知链已运行）
  --mode fire            先拉起 auto_aim，再拉起 mapper_node（默认）

Key options:
  --offline              离线模式（默认，传 offline:=true）
  --online               在线模式（不传 offline:=true）
  --cleanup-existing     启动前清理残留进程（默认）
  --no-cleanup-existing  发现残留进程直接报错退出
  --wait SECONDS         等待感知链就绪秒数（默认: ${WAIT_SECONDS}）
  --cmd-timeout SECONDS  ros2 命令超时（默认: ${CMD_TIMEOUT}）
  --output screen|log    launch 输出模式（默认: ${OUTPUT_MODE}）
  --config-file PATH     指定 config_file:=PATH
  --allow-multi-control  允许多控制源同时写 control 话题（默认禁止）
  --dry-run              只打印命令，不执行

Mapper options:
  --red true|false
  --target-id N
  --target-priority "6,3,4,5"
  --publish-team true|false
  --publish-target true|false
  --enable-fire true|false
  --auto-fire true|false
  --target-timeout SEC
  --diag-period SEC
  --angles-topic TOPIC
  --firecode-topic TOPIC
  --bt-target-topic TOPIC

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --mode perception --offline
  ./${SCRIPT_NAME} --mode fire --offline --angles-topic /ly/control/angles --firecode-topic /ly/control/firecode
  ./${SCRIPT_NAME} --mode mapper --publish-team true --red true
  ./${SCRIPT_NAME} --online
  ./${SCRIPT_NAME} --online --config-file /abs/path/auto_aim_config.yaml
EOF
}

info() {
  printf "[AUTOAIM-DEBUG][INFO] %s\n" "$*"
}

warn() {
  printf "[AUTOAIM-DEBUG][WARN] %s\n" "$*" >&2
}

err() {
  printf "[AUTOAIM-DEBUG][ERROR] %s\n" "$*" >&2
}

require_option_value() {
  local option_name="$1"
  local option_value="${2-}"
  if [[ -z "${option_value}" || "${option_value}" == --* ]]; then
    err "Option ${option_name} requires a value."
    usage >&2
    exit 2
  fi
}

cleanup() {
  if [[ -n "${TAIL_PID}" ]] && kill -0 "${TAIL_PID}" 2>/dev/null; then
    kill -TERM "${TAIL_PID}" 2>/dev/null || true
    wait "${TAIL_PID}" 2>/dev/null || true
  fi

  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    info "Stopping auto_aim launch (PID=${LAUNCH_PID})"
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi

  if (( LOCK_ACQUIRED == 1 )) && [[ -f "${LOCK_FILE}" ]]; then
    local owner
    owner="$(cat "${LOCK_FILE}" 2>/dev/null || true)"
    if [[ "${owner}" == "$$" ]]; then
      rm -f "${LOCK_FILE}"
    fi
  fi
}

trap cleanup EXIT

acquire_lock() {
  if ( set -o noclobber; echo "$$" >"${LOCK_FILE}" ) 2>/dev/null; then
    LOCK_ACQUIRED=1
    return 0
  fi

  local owner="unknown"
  if [[ -f "${LOCK_FILE}" ]]; then
    owner="$(cat "${LOCK_FILE}" 2>/dev/null || true)"
  fi

  if [[ "${owner}" =~ ^[0-9]+$ ]] && kill -0 "${owner}" 2>/dev/null; then
    err "Another autoaim_debug instance is running (pid=${owner})."
  else
    warn "Stale lock detected, reclaiming: ${LOCK_FILE}"
    rm -f "${LOCK_FILE}" || true
    if ( set -o noclobber; echo "$$" >"${LOCK_FILE}" ) 2>/dev/null; then
      LOCK_ACQUIRED=1
      return 0
    fi
    err "Failed to acquire lock after stale cleanup: ${LOCK_FILE}"
  fi
  return 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      require_option_value "$1" "${2-}"
      MODE="$2"
      shift 2
      ;;
    --offline)
      OFFLINE_MODE=1
      shift
      ;;
    --online)
      OFFLINE_MODE=0
      shift
      ;;
    --cleanup-existing)
      CLEANUP_EXISTING=1
      shift
      ;;
    --no-cleanup-existing)
      CLEANUP_EXISTING=0
      shift
      ;;
    --allow-multi-control)
      ALLOW_MULTI_CONTROL=1
      shift
      ;;
    --wait)
      require_option_value "$1" "${2-}"
      WAIT_SECONDS="$2"
      shift 2
      ;;
    --cmd-timeout)
      require_option_value "$1" "${2-}"
      CMD_TIMEOUT="$2"
      shift 2
      ;;
    --output)
      require_option_value "$1" "${2-}"
      OUTPUT_MODE="$2"
      shift 2
      ;;
    --config-file)
      require_option_value "$1" "${2-}"
      CONFIG_FILE="$2"
      shift 2
      ;;
    --red)
      require_option_value "$1" "${2-}"
      MAPPER_RED="$2"
      shift 2
      ;;
    --target-id)
      require_option_value "$1" "${2-}"
      MAPPER_TARGET_ID="$2"
      shift 2
      ;;
    --target-priority)
      require_option_value "$1" "${2-}"
      MAPPER_TARGET_PRIORITY="$2"
      shift 2
      ;;
    --publish-team)
      require_option_value "$1" "${2-}"
      MAPPER_PUBLISH_TEAM="$2"
      shift 2
      ;;
    --publish-target)
      require_option_value "$1" "${2-}"
      MAPPER_PUBLISH_TARGET="$2"
      shift 2
      ;;
    --enable-fire)
      require_option_value "$1" "${2-}"
      MAPPER_ENABLE_FIRE="$2"
      shift 2
      ;;
    --auto-fire)
      require_option_value "$1" "${2-}"
      MAPPER_AUTO_FIRE="$2"
      shift 2
      ;;
    --target-timeout)
      require_option_value "$1" "${2-}"
      MAPPER_TARGET_TIMEOUT="$2"
      shift 2
      ;;
    --diag-period)
      require_option_value "$1" "${2-}"
      MAPPER_DIAG_PERIOD="$2"
      shift 2
      ;;
    --angles-topic)
      require_option_value "$1" "${2-}"
      MAPPER_ANGLES_TOPIC="$2"
      shift 2
      ;;
    --firecode-topic)
      require_option_value "$1" "${2-}"
      MAPPER_FIRECODE_TOPIC="$2"
      shift 2
      ;;
    --bt-target-topic)
      require_option_value "$1" "${2-}"
      MAPPER_BT_TARGET_TOPIC="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
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
      err "Unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

case "${MODE}" in
  perception|mapper|fire) ;;
  *)
    err "Invalid --mode: ${MODE}. Expected: perception|mapper|fire"
    exit 2
    ;;
esac

if [[ -z "${MAPPER_ANGLES_TOPIC}" ]]; then
  if [[ "${MODE}" == "fire" ]]; then
    MAPPER_ANGLES_TOPIC="/ly/control/angles"
  else
    MAPPER_ANGLES_TOPIC="/ly/debug/control/angles"
  fi
fi

if [[ -z "${MAPPER_FIRECODE_TOPIC}" ]]; then
  if [[ "${MODE}" == "fire" ]]; then
    MAPPER_FIRECODE_TOPIC="/ly/control/firecode"
  else
    MAPPER_FIRECODE_TOPIC="/ly/debug/control/firecode"
  fi
fi

source_ros() {
  if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    return
  fi
  local distro
  for distro in humble iron jazzy rolling; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "/opt/ros/${distro}/setup.bash"
      set -u
      return
    fi
  done
  err "No ROS2 setup found under /opt/ros"
  exit 1
}

source_workspace() {
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${ROOT_DIR}/install/setup.bash"
    set -u
    return
  fi
  err "${ROOT_DIR}/install/setup.bash not found. Run: colcon build"
  exit 1
}

select_default_config_if_needed() {
  if [[ -n "${CONFIG_FILE}" ]]; then
    return 0
  fi

  if (( OFFLINE_MODE == 1 )); then
    return 0
  fi

  if [[ -f "${ONLINE_CONFIG_FILE}" ]]; then
    CONFIG_FILE="${ONLINE_CONFIG_FILE}"
    info "Online mode: defaulting to on-robot config ${CONFIG_FILE}"
    return 0
  fi

  warn "Online default config not found: ${ONLINE_CONFIG_FILE}. Falling back to launch default config."
  return 0
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

wait_for_nodes_ready() {
  local deadline
  deadline=$((SECONDS + WAIT_SECONDS))
  local required=("/gimbal_driver" "/detector" "/tracker_solver" "/predictor_node")
  while (( SECONDS < deadline )); do
    local nodes
    nodes="$(timeout "${CMD_TIMEOUT}" ros2 node list 2>/dev/null || true)"
    local miss=0
    local node
    for node in "${required[@]}"; do
      if ! grep -qx "${node}" <<<"${nodes}"; then
        miss=1
        break
      fi
    done
    if (( miss == 0 )); then
      info "Perception chain online: ${required[*]}"
      return 0
    fi
    sleep 1
  done
  warn "Node readiness check timed out after ${WAIT_SECONDS}s"
  return 1
}

ensure_single_control_source() {
  if (( ALLOW_MULTI_CONTROL == 1 )); then
    return 0
  fi
  local nodes
  nodes="$(timeout "${CMD_TIMEOUT}" ros2 node list 2>/dev/null || true)"
  if grep -qx "/behavior_tree" <<<"${nodes}"; then
    err "Detected /behavior_tree online. Refuse to start mapper on control topics."
    err "Use --allow-multi-control to override (not recommended)."
    return 1
  fi

  local topic count
  for topic in "/ly/control/angles" "/ly/control/firecode"; do
    count="$(topic_publisher_count "${topic}")"
    if [[ -z "${count}" ]]; then
      count="0"
    fi
    if (( count > 0 )); then
      err "Topic ${topic} already has ${count} publisher(s). Refuse to take over."
      topic_publisher_nodes "${topic}" | while IFS= read -r node; do
        [[ -n "${node}" ]] && err "  existing publisher: ${node}"
      done
      err "Use --allow-multi-control to override (not recommended)."
      return 1
    fi
  done
  return 0
}

topic_publisher_count() {
  local topic="$1"
  local output
  output="$(timeout "${CMD_TIMEOUT}" ros2 topic info "${topic}" -v 2>/dev/null || true)"
  awk -F ':' '
    /Publisher count:/ {
      gsub(/[[:space:]]/, "", $2)
      print $2
      found=1
      exit
    }
    END {
      if (!found) {
        print "0"
      }
    }
  ' <<< "${output}"
}

topic_publisher_nodes() {
  local topic="$1"
  local output
  output="$(timeout "${CMD_TIMEOUT}" ros2 topic info "${topic}" -v 2>/dev/null || true)"
  awk '
    /Node name:/ {
      line=$0
      sub(/^[[:space:]]*Node name:[[:space:]]*/, "", line)
      sub(/,[[:space:]]*Node namespace:.*/, "", line)
      sub(/[[:space:]]+$/, "", line)
      if (line != "") {
        if (line !~ /^\//) {
          line="/" line
        }
        print line
      }
    }
  ' <<< "${output}" | sort -u
}

cleanup_existing_stack_if_needed() {
  mapfile -t existing_procs < <(
    {
      pgrep -af "${STACK_NODE_REGEX}" || true
      pgrep -af "${STACK_LAUNCH_REGEX}" || true
    } | awk '!seen[$0]++'
  )

  if (( ${#existing_procs[@]} == 0 )); then
    return 0
  fi

  warn "Detected existing auto_aim-related processes:"
  printf "  %s\n" "${existing_procs[@]}" >&2

  if (( CLEANUP_EXISTING == 0 )); then
    err "Existing processes found. Re-run with --cleanup-existing."
    return 1
  fi

  info "Cleaning up stale processes..."
  pkill -f "${STACK_NODE_REGEX}" || true
  pkill -f "${STACK_LAUNCH_REGEX}" || true
  sleep 1
  return 0
}

build_launch_cmd() {
  LAUNCH_CMD=(ros2 launch detector auto_aim.launch.py "output:=${OUTPUT_MODE}" "use_mapper:=false")
  if [[ -n "${CONFIG_FILE}" ]]; then
    LAUNCH_CMD+=("config_file:=${CONFIG_FILE}")
  fi
  if (( OFFLINE_MODE == 1 )); then
    if ! has_launch_arg_key "offline"; then
      LAUNCH_CMD+=("offline:=true")
    fi
  fi
  if (( ${#LAUNCH_ARGS[@]} > 0 )); then
    LAUNCH_CMD+=("${LAUNCH_ARGS[@]}")
  fi
}

build_mapper_cmd() {
  MAPPER_CMD=(ros2 run detector mapper_node
    --red "${MAPPER_RED}"
    --target-id "${MAPPER_TARGET_ID}"
    --publish-team "${MAPPER_PUBLISH_TEAM}"
    --publish-target "${MAPPER_PUBLISH_TARGET}"
    --enable-fire "${MAPPER_ENABLE_FIRE}"
    --auto-fire "${MAPPER_AUTO_FIRE}"
    --target-timeout "${MAPPER_TARGET_TIMEOUT}"
    --diag-period "${MAPPER_DIAG_PERIOD}")

  if [[ -n "${MAPPER_TARGET_PRIORITY}" ]]; then
    MAPPER_CMD+=(--target-priority "${MAPPER_TARGET_PRIORITY}")
  fi

  MAPPER_CMD+=(--ros-args
    -r "/ly/control/angles:=${MAPPER_ANGLES_TOPIC}"
    -r "/ly/control/firecode:=${MAPPER_FIRECODE_TOPIC}")

  if [[ -n "${MAPPER_BT_TARGET_TOPIC}" && "${MAPPER_BT_TARGET_TOPIC}" != "/ly/bt/target" ]]; then
    MAPPER_CMD+=(-r "/ly/bt/target:=${MAPPER_BT_TARGET_TOPIC}")
  fi
}

print_cmd() {
  printf "%q " "$@"
}

source_ros
source_workspace
cd "${ROOT_DIR}"
acquire_lock

cleanup_existing_stack_if_needed
select_default_config_if_needed

build_launch_cmd
build_mapper_cmd

info "Mode=${MODE}, offline=${OFFLINE_MODE}, output=${OUTPUT_MODE}"
info "Mapper outputs: angles=${MAPPER_ANGLES_TOPIC}, firecode=${MAPPER_FIRECODE_TOPIC}, bt_target=${MAPPER_BT_TARGET_TOPIC}"
info "Launch cmd: $(print_cmd "${LAUNCH_CMD[@]}")"
if [[ "${MODE}" != "perception" ]]; then
  info "Mapper cmd: $(print_cmd "${MAPPER_CMD[@]}")"
fi

if (( DRY_RUN == 1 )); then
  exit 0
fi

if [[ "${MODE}" == "mapper" ]]; then
  if [[ "${MAPPER_ANGLES_TOPIC}" == "/ly/control/angles" || "${MAPPER_FIRECODE_TOPIC}" == "/ly/control/firecode" ]]; then
    ensure_single_control_source
  fi
  "${MAPPER_CMD[@]}"
  exit $?
fi

if [[ "${MODE}" == "perception" ]]; then
  "${LAUNCH_CMD[@]}"
  exit $?
fi

if [[ "${MAPPER_ANGLES_TOPIC}" == "/ly/control/angles" || "${MAPPER_FIRECODE_TOPIC}" == "/ly/control/firecode" ]]; then
  ensure_single_control_source
fi

LAUNCH_LOG="$(mktemp /tmp/autoaim_debug_launch.XXXXXX.log)"
info "Starting auto_aim launch in background (log: ${LAUNCH_LOG})"
"${LAUNCH_CMD[@]}" >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

wait_for_nodes_ready || true

info "Launching mapper in foreground; Ctrl+C will stop both mapper and launch."
"${MAPPER_CMD[@]}"
exit $?
