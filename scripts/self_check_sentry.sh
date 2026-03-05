#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

: "${ROS_LOG_DIR:=/tmp/ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

AUTO_LAUNCH=0
WAIT_SECONDS=18
CMD_TIMEOUT=6
HZ_SECONDS=6
SKIP_HZ=0
STATIC_ONLY=0
RUNTIME_ONLY=0
LAUNCH_ARGS=()

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

LAUNCH_PID=""
LAUNCH_LOG=""

if [[ -t 1 ]]; then
  C_RESET=$'\033[0m'
  C_RED=$'\033[31m'
  C_YELLOW=$'\033[33m'
  C_GREEN=$'\033[32m'
  C_CYAN=$'\033[36m'
else
  C_RESET=""
  C_RED=""
  C_YELLOW=""
  C_GREEN=""
  C_CYAN=""
fi

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--launch] [--wait SECONDS] [--cmd-timeout SECONDS] [--hz-seconds SECONDS] [--skip-hz] [--static-only|--runtime-only] [-- <launch_args...>]

Examples:
  # 檢查當前已在運行的系統
  ./${SCRIPT_NAME}

  # 自動啟動整套節點後再檢查
  ./${SCRIPT_NAME} --launch

  # 只做離車靜態檢查（不依賴 ROS 圖）
  ./${SCRIPT_NAME} --static-only

  # 只做運行時圖檢查（跳過文件/配置靜態檢查）
  ./${SCRIPT_NAME} --runtime-only --launch

  # 自動啟動 + 自定義 launch 參數
  ./${SCRIPT_NAME} --launch -- --config_file:=/abs/path/auto_aim_config.yaml use_buff:=false

Options:
  --launch               自動啟動 sentry_all.launch.py，檢查結束後自動停止
  --wait SECONDS         啟動後等待秒數（默認: ${WAIT_SECONDS}）
  --cmd-timeout SECONDS  單次 ros2 命令超時（默認: ${CMD_TIMEOUT}）
  --hz-seconds SECONDS   ros2 topic hz 採樣時長（默認: ${HZ_SECONDS}）
  --skip-hz              跳過頻率檢查（更快）
  --static-only          只執行靜態檢查（文件、配置、BT XML）
  --runtime-only         只執行運行時檢查（node/topic/hz）
  --help                 顯示幫助
EOF
}

info() {
  printf "%s[INFO]%s %s\n" "${C_CYAN}" "${C_RESET}" "$*"
}

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf "%s[PASS]%s %s\n" "${C_GREEN}" "${C_RESET}" "$*"
}

warn() {
  WARN_COUNT=$((WARN_COUNT + 1))
  printf "%s[WARN]%s %s\n" "${C_YELLOW}" "${C_RESET}" "$*"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  printf "%s[FAIL]%s %s\n" "${C_RED}" "${C_RESET}" "$*"
}

issue() {
  local severity="$1"
  shift
  if [[ "${severity}" == "warn" ]]; then
    warn "$*"
  else
    fail "$*"
  fi
}

source_ros() {
  if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    pass "ROS2 sourced: /opt/ros/${ROS_DISTRO}/setup.bash"
    return
  fi

  local distro
  for distro in humble iron jazzy rolling; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      # shellcheck disable=SC1090
      set +u
      source "/opt/ros/${distro}/setup.bash"
      set -u
      pass "ROS2 sourced: /opt/ros/${distro}/setup.bash"
      return
    fi
  done

  fail "No ROS2 setup found under /opt/ros"
}

source_workspace() {
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u
    source "${ROOT_DIR}/install/setup.bash"
    set -u
    pass "Workspace sourced: ${ROOT_DIR}/install/setup.bash"
  else
    fail "Workspace not built: ${ROOT_DIR}/install/setup.bash missing"
  fi
}

cleanup_launch() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    info "Stopping launched stack (PID=${LAUNCH_PID})"
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}

trap cleanup_launch EXIT

while [[ $# -gt 0 ]]; do
  case "$1" in
    --launch)
      AUTO_LAUNCH=1
      shift
      ;;
    --wait)
      WAIT_SECONDS="$2"
      shift 2
      ;;
    --cmd-timeout)
      CMD_TIMEOUT="$2"
      shift 2
      ;;
    --hz-seconds)
      HZ_SECONDS="$2"
      shift 2
      ;;
    --skip-hz)
      SKIP_HZ=1
      shift
      ;;
    --static-only)
      STATIC_ONLY=1
      shift
      ;;
    --runtime-only)
      RUNTIME_ONLY=1
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
      printf "[ERROR] Unknown argument: %s\n" "$1" >&2
      usage
      exit 2
      ;;
  esac
done

if (( STATIC_ONLY == 1 && RUNTIME_ONLY == 1 )); then
  fail "--static-only and --runtime-only are mutually exclusive"
  exit 2
fi

check_cmd() {
  local cmd="$1"
  if command -v "${cmd}" >/dev/null 2>&1; then
    pass "Command available: ${cmd}"
  else
    fail "Command missing: ${cmd}"
  fi
}

check_file_exists() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    pass "File exists: ${path}"
  else
    fail "File missing: ${path}"
  fi
}

trim_text() {
  local text="$1"
  text="${text#"${text%%[![:space:]]*}"}"
  text="${text%"${text##*[![:space:]]}"}"
  printf "%s" "${text}"
}

extract_yaml_quoted_key_value() {
  local yaml_file="$1"
  local key_pattern="$2"
  local line
  local value

  line="$(grep -E "^[[:space:]]*\"${key_pattern}\"[[:space:]]*:" "${yaml_file}" | head -n1 || true)"
  if [[ -z "${line}" ]]; then
    printf "%s" ""
    return
  fi

  value="${line#*:}"
  value="${value%%#*}"
  value="$(trim_text "${value}")"

  if [[ "${value}" =~ ^\".*\"$ ]]; then
    value="${value:1:${#value}-2}"
  fi

  printf "%s" "${value}"
}

check_camera_sn_config() {
  local yaml_file="$1"
  local sn_dot
  local sn_slash

  if [[ ! -f "${yaml_file}" ]]; then
    fail "camera_sn check skipped: YAML missing: ${yaml_file}"
    return
  fi

  sn_dot="$(extract_yaml_quoted_key_value "${yaml_file}" 'camera_param\.camera_sn')"
  sn_slash="$(extract_yaml_quoted_key_value "${yaml_file}" 'camera_param/camera_sn')"

  if [[ -z "${sn_dot}" && -z "${sn_slash}" ]]; then
    fail "camera_sn missing: both \"camera_param.camera_sn\" and \"camera_param/camera_sn\" are empty or absent"
    return
  fi

  if [[ -n "${sn_dot}" ]]; then
    pass "camera_sn present: camera_param.camera_sn=${sn_dot}"
  else
    warn "camera_param.camera_sn missing; detector compatibility relies on slash key only"
  fi

  if [[ -n "${sn_slash}" ]]; then
    pass "camera_sn present: camera_param/camera_sn=${sn_slash}"
  else
    warn "camera_param/camera_sn missing; legacy compatibility relies on dot key only"
  fi

  if [[ -n "${sn_dot}" && -n "${sn_slash}" ]]; then
    if [[ "${sn_dot}" == "${sn_slash}" ]]; then
      pass "camera_sn dual-key values are consistent"
    else
      fail "camera_sn mismatch between dot/slash keys: dot=${sn_dot}, slash=${sn_slash}"
    fi
  fi
}

check_legacy_hardcoded_camera_sn() {
  local hits
  if command -v rg >/dev/null 2>&1; then
    hits="$(rg -n 'KE[0-9A-Za-z]+' "${ROOT_DIR}/src/shooting_table_calib/launch" --glob '*.launch' --glob '*.launch.py' || true)"
  else
    hits="$(find "${ROOT_DIR}/src/shooting_table_calib/launch" -type f \( -name '*.launch' -o -name '*.launch.py' \) -print0 \
      | xargs -0 grep -nE 'KE[0-9A-Za-z]+' 2>/dev/null || true)"
    warn "rg not found; fallback to grep for hardcoded camera SN scan"
  fi
  if [[ -n "${hits}" ]]; then
    warn "Legacy launch still contains hardcoded camera SN candidates: ${hits//$'\n'/; }"
  else
    pass "No hardcoded camera SN found under shooting_table_calib launch files"
  fi
}

run_ros2() {
  timeout "${CMD_TIMEOUT}s" ros2 "$@" 2>/dev/null
}

extract_endpoint_nodes() {
  local info_text="$1"
  local endpoint="$2"
  awk -v endpoint="${endpoint}" '
    /Endpoint type:/ { mode=$3 }
    /Node name:/ {
      node=$3
      if(mode == endpoint){ print node }
    }
  ' <<< "${info_text}" | sort -u
}

declare -A NODE_INFO_CACHE

get_node_info_cached() {
  local node="$1"
  if [[ -z "${NODE_INFO_CACHE[${node}]+x}" ]]; then
    NODE_INFO_CACHE["${node}"]="$(run_ros2 node info "${node}" || true)"
  fi
  printf "%s" "${NODE_INFO_CACHE[${node}]}"
}

node_has_topic_in_section() {
  local info_text="$1"
  local section="$2"
  local topic="$3"
  awk -v section="${section}" -v topic="${topic}" '
    BEGIN { in_section = 0; found = 0 }
    $0 ~ "^[[:space:]]*" section ":" { in_section = 1; next }
    in_section && $0 ~ "^[[:space:]]*[A-Za-z][A-Za-z _]*:" { in_section = 0 }
    in_section && $0 ~ "^[[:space:]]*/[^:]+:" {
      t = $1
      sub(/:$/, "", t)
      if(t == topic) { found = 1; exit }
    }
    END { exit(found ? 0 : 1) }
  ' <<< "${info_text}"
}

check_node_online() {
  local node="$1"
  local node_list="$2"
  if grep -Fxq "${node}" <<< "${node_list}"; then
    pass "Node online: ${node}"
  else
    fail "Node missing: ${node}"
  fi
}

check_node_sub() {
  local node="$1"
  local topic="$2"
  local severity="${3:-hard}"
  local info_text
  info_text="$(get_node_info_cached "${node}")"
  if [[ -z "${info_text}" ]]; then
    issue "${severity}" "Node info unavailable: ${node}"
    return
  fi
  if node_has_topic_in_section "${info_text}" "Subscribers" "${topic}"; then
    pass "Subscriber contract OK: ${node} <- ${topic}"
  else
    issue "${severity}" "Subscriber contract missing: ${node} <- ${topic}"
  fi
}

check_node_pub() {
  local node="$1"
  local topic="$2"
  local severity="${3:-hard}"
  local info_text
  info_text="$(get_node_info_cached "${node}")"
  if [[ -z "${info_text}" ]]; then
    issue "${severity}" "Node info unavailable: ${node}"
    return
  fi
  if node_has_topic_in_section "${info_text}" "Publishers" "${topic}"; then
    pass "Publisher contract OK: ${node} -> ${topic}"
  else
    issue "${severity}" "Publisher contract missing: ${node} -> ${topic}"
  fi
}

check_topic_link() {
  local topic="$1"
  local expected_type="$2"
  local expected_pubs_csv="$3"
  local expected_subs_csv="$4"
  local severity="${5:-hard}"

  local topic_type
  topic_type="$(run_ros2 topic type "${topic}" | head -n1 || true)"
  if [[ -z "${topic_type}" ]]; then
    issue "${severity}" "Topic missing: ${topic}"
    return
  fi

  if [[ "${topic_type}" == "${expected_type}" ]]; then
    pass "Topic type OK: ${topic} == ${expected_type}"
  else
    issue "${severity}" "Topic type mismatch: ${topic}, expect=${expected_type}, got=${topic_type}"
  fi

  local info_text
  info_text="$(run_ros2 topic info "${topic}" -v || true)"
  if [[ -z "${info_text}" ]]; then
    issue "${severity}" "Topic info unavailable: ${topic}"
    return
  fi

  local pub_nodes
  local sub_nodes
  pub_nodes="$(extract_endpoint_nodes "${info_text}" "PUBLISHER")"
  sub_nodes="$(extract_endpoint_nodes "${info_text}" "SUBSCRIPTION")"

  local node
  IFS=',' read -r -a _pub_nodes <<< "${expected_pubs_csv}"
  for node in "${_pub_nodes[@]}"; do
    [[ -z "${node}" ]] && continue
    if grep -Fxq "${node}" <<< "${pub_nodes}"; then
      pass "Topic publisher OK: ${topic} has ${node}"
    else
      issue "${severity}" "Topic publisher missing: ${topic} lacks ${node}"
    fi
  done

  IFS=',' read -r -a _sub_nodes <<< "${expected_subs_csv}"
  for node in "${_sub_nodes[@]}"; do
    [[ -z "${node}" ]] && continue
    if grep -Fxq "${node}" <<< "${sub_nodes}"; then
      pass "Topic subscriber OK: ${topic} has ${node}"
    else
      issue "${severity}" "Topic subscriber missing: ${topic} lacks ${node}"
    fi
  done
}

check_topic_hz() {
  local topic="$1"
  local min_rate="$2"
  local severity="${3:-warn}"

  local out_text
  out_text="$(timeout "$((HZ_SECONDS + 2))s" ros2 topic hz "${topic}" 2>/dev/null || true)"
  local avg_rate
  avg_rate="$(awk '/average rate:/ {print $3; exit}' <<< "${out_text}")"

  if [[ -z "${avg_rate}" ]]; then
    issue "${severity}" "No hz sample from ${topic} within ${HZ_SECONDS}s"
    return
  fi

  if awk -v v="${avg_rate}" -v min="${min_rate}" 'BEGIN { exit (v >= min ? 0 : 1) }'; then
    pass "Topic hz OK: ${topic} average=${avg_rate} Hz (>= ${min_rate})"
  else
    issue "${severity}" "Topic hz low: ${topic} average=${avg_rate} Hz (< ${min_rate})"
  fi
}

print_section() {
  printf "\n%s==== %s ====%s\n" "${C_CYAN}" "$1" "${C_RESET}"
}

print_section "Environment"
check_cmd bash
check_cmd ros2
check_cmd timeout
check_cmd awk
check_cmd grep
source_ros
source_workspace

if (( RUNTIME_ONLY == 0 )); then
  print_section "Static Files"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/Scripts/main.xml"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/Scripts/config.json"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/launch/sentry_all.launch.py"
  check_file_exists "${ROOT_DIR}/src/detector/config/auto_aim_config.yaml"
  check_file_exists "${ROOT_DIR}/scripts/start_sentry_all.sh"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/include/BTNodes.hpp"
  check_camera_sn_config "${ROOT_DIR}/src/detector/config/auto_aim_config.yaml"
  check_legacy_hardcoded_camera_sn

  if grep -Fq 'BTCPP_format="4"' "${ROOT_DIR}/src/behavior_tree/Scripts/main.xml"; then
    pass "BT XML format is v4"
  else
    fail "BT XML format is not v4"
  fi

  if grep -Fq '<SelectStrategyMode/>' "${ROOT_DIR}/src/behavior_tree/Scripts/main.xml"; then
    pass "BT XML contains dynamic strategy switch node"
  else
    fail "BT XML missing SelectStrategyMode node"
  fi
fi

if (( STATIC_ONLY == 0 )); then
  print_section "Runtime Graph"
fi

if (( STATIC_ONLY == 0 && AUTO_LAUNCH == 1 )); then
  LAUNCH_LOG="$(mktemp /tmp/sentry_self_check.XXXXXX.log)"
  info "Launching stack by scripts/start_sentry_all.sh (log: ${LAUNCH_LOG})"
  (
    cd "${ROOT_DIR}"
    ./scripts/start_sentry_all.sh "${LAUNCH_ARGS[@]}"
  ) >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
  sleep "${WAIT_SECONDS}"
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    pass "Auto-launch still running after ${WAIT_SECONDS}s"
  else
    fail "Auto-launch exited early. Check log: ${LAUNCH_LOG}"
    tail -n 40 "${LAUNCH_LOG}" || true
  fi
fi

if (( STATIC_ONLY == 0 )); then
  NODE_LIST="$(run_ros2 node list || true)"
  GRAPH_AVAILABLE=1
  if [[ -z "${NODE_LIST}" ]]; then
    fail "No ROS2 nodes found. Start stack first or run with --launch"
    GRAPH_AVAILABLE=0
  else
    pass "ROS2 graph has active nodes"
  fi

  if (( GRAPH_AVAILABLE == 1 )); then
  check_node_online "/gimbal_driver" "${NODE_LIST}"
  check_node_online "/detector" "${NODE_LIST}"
  check_node_online "/tracker_solver" "${NODE_LIST}"
  check_node_online "/predictor_node" "${NODE_LIST}"
  check_node_online "/outpost_hitter" "${NODE_LIST}"
  check_node_online "/buff_hitter" "${NODE_LIST}"
  check_node_online "/behavior_tree" "${NODE_LIST}"

  print_section "Node Contracts"
  # gimbal_driver
  check_node_sub "/gimbal_driver" "/ly/control/angles" hard
  check_node_sub "/gimbal_driver" "/ly/control/firecode" hard
  check_node_sub "/gimbal_driver" "/ly/control/vel" hard

  # detector
  check_node_sub "/detector" "/ly/aa/enable" hard
  check_node_sub "/detector" "/ly/ra/enable" hard
  check_node_sub "/detector" "/ly/outpost/enable" hard
  check_node_sub "/detector" "/ly/bt/target" hard
  check_node_sub "/detector" "/ly/gimbal/angles" hard
  check_node_sub "/detector" "/ly/me/is_team_red" hard

  # tracker/predictor
  check_node_sub "/tracker_solver" "/ly/detector/armors" hard
  check_node_sub "/predictor_node" "/ly/tracker/results" hard
  check_node_sub "/predictor_node" "/ly/bt/target" hard
  check_node_sub "/predictor_node" "/ly/bullet/speed" hard

  # outpost/buff
  check_node_sub "/outpost_hitter" "/ly/outpost/armors" hard
  check_node_sub "/buff_hitter" "/ly/ra/enable" hard
  check_node_sub "/buff_hitter" "/ly/aa/enable" hard
  check_node_sub "/buff_hitter" "/ly/ra/angle_image" hard

  # behavior_tree inputs
  check_node_sub "/behavior_tree" "/ly/gimbal/angles" hard
  check_node_sub "/behavior_tree" "/ly/game/is_start" hard
  check_node_sub "/behavior_tree" "/ly/game/time_left" hard
  check_node_sub "/behavior_tree" "/ly/me/is_team_red" hard
  check_node_sub "/behavior_tree" "/ly/predictor/target" hard
  check_node_sub "/behavior_tree" "/ly/buff/target" hard
  check_node_sub "/behavior_tree" "/ly/outpost/target" hard

  # behavior_tree outputs
  check_node_pub "/behavior_tree" "/ly/control/angles" hard
  check_node_pub "/behavior_tree" "/ly/control/firecode" hard
  check_node_pub "/behavior_tree" "/ly/aa/enable" hard
  check_node_pub "/behavior_tree" "/ly/ra/enable" hard
  check_node_pub "/behavior_tree" "/ly/outpost/enable" hard
  check_node_pub "/behavior_tree" "/ly/bt/target" hard
  check_node_pub "/behavior_tree" "/ly/navi/vel" hard

  print_section "Critical Topic Links"
  check_topic_link "/ly/control/angles" "gimbal_driver/msg/GimbalAngles" "/behavior_tree" "/gimbal_driver" hard
  check_topic_link "/ly/control/firecode" "std_msgs/msg/UInt8" "/behavior_tree" "/gimbal_driver" hard

  # 兼容鏈路檢查：電控側仍訂閱 /ly/control/vel，若沒有發布者視為缺口
  check_topic_link "/ly/control/vel" "gimbal_driver/msg/Vel" "/behavior_tree" "/gimbal_driver" hard

  check_topic_link "/ly/bt/target" "std_msgs/msg/UInt8" "/behavior_tree" "/detector,/predictor_node" hard
  check_topic_link "/ly/aa/enable" "std_msgs/msg/Bool" "/behavior_tree" "/detector,/buff_hitter" hard
  check_topic_link "/ly/ra/enable" "std_msgs/msg/Bool" "/behavior_tree" "/detector,/buff_hitter" hard
  check_topic_link "/ly/outpost/enable" "std_msgs/msg/Bool" "/behavior_tree" "/detector" hard

  print_section "Conditional Topics (Data-Dependent)"
  check_topic_link "/ly/gimbal/angles" "gimbal_driver/msg/GimbalAngles" "/gimbal_driver" "/behavior_tree,/detector" warn
  check_topic_link "/ly/detector/armors" "auto_aim_common/msg/Armors" "/detector" "/tracker_solver,/behavior_tree" warn
  check_topic_link "/ly/tracker/results" "auto_aim_common/msg/Trackers" "/tracker_solver" "/predictor_node" warn
  check_topic_link "/ly/predictor/target" "auto_aim_common/msg/Target" "/predictor_node" "/behavior_tree" warn
  check_topic_link "/ly/ra/angle_image" "auto_aim_common/msg/AngleImage" "/detector" "/buff_hitter" warn
  check_topic_link "/ly/buff/target" "auto_aim_common/msg/Target" "/buff_hitter" "/behavior_tree" warn
  check_topic_link "/ly/outpost/armors" "auto_aim_common/msg/Armors" "/detector" "/outpost_hitter" warn
  check_topic_link "/ly/outpost/target" "auto_aim_common/msg/Target" "/outpost_hitter" "/behavior_tree" warn

  if (( SKIP_HZ == 0 )); then
    print_section "Frequency Checks"
    check_topic_hz "/ly/control/angles" 5 hard
    check_topic_hz "/ly/control/firecode" 5 hard
    check_topic_hz "/ly/navi/vel" 1 warn
    check_topic_hz "/ly/gimbal/angles" 1 warn
    check_topic_hz "/ly/detector/armors" 1 warn
  fi
  else
    warn "Runtime graph checks skipped because no ROS2 nodes are active"
  fi
fi

print_section "Summary"
printf "PASS: %d\n" "${PASS_COUNT}"
printf "WARN: %d\n" "${WARN_COUNT}"
printf "FAIL: %d\n" "${FAIL_COUNT}"

if (( FAIL_COUNT > 0 )); then
  printf "%sResult: FAILED%s\n" "${C_RED}" "${C_RESET}"
  exit 1
fi

printf "%sResult: PASSED%s\n" "${C_GREEN}" "${C_RESET}"
exit 0
