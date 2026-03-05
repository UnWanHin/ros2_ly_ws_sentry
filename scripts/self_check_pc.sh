#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

: "${ROS_LOG_DIR:=/tmp/ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

DO_BUILD=1
DO_TEST=0
PACKAGES="behavior_tree outpost_hitter predictor buff_hitter detector gimbal_driver tracker_solver shooting_table_calib auto_aim_common"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--no-build] [--test] [--packages "pkg1 pkg2 ..."]

Examples:
  # 標準離車自檢（推薦）
  ./${SCRIPT_NAME}

  # 不重建，只做靜態契約檢查
  ./${SCRIPT_NAME} --no-build

  # 只檢部分包 + 跑 colcon test
  ./${SCRIPT_NAME} --packages "behavior_tree outpost_hitter predictor" --test
EOF
}

log() {
  printf "[PC-CHECK] %s\n" "$*"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-build)
      DO_BUILD=0
      shift
      ;;
    --test)
      DO_TEST=1
      shift
      ;;
    --packages)
      PACKAGES="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      printf "[ERROR] Unknown argument: %s\n" "$1" >&2
      usage
      exit 2
      ;;
  esac
done

cd "${ROOT_DIR}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  printf "[ERROR] /opt/ros/humble/setup.bash not found\n" >&2
  exit 1
fi

if (( DO_BUILD == 1 )); then
  log "Building packages: ${PACKAGES}"
  colcon build --packages-select ${PACKAGES}
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "${ROOT_DIR}/install/setup.bash"
  set -u
else
  printf "[ERROR] install/setup.bash missing. Build workspace first.\n" >&2
  exit 1
fi

if (( DO_TEST == 1 )); then
  log "Running tests for packages: ${PACKAGES}"
  colcon test --packages-select ${PACKAGES}
  colcon test-result --verbose
fi

log "Running static self-check suite"
"${ROOT_DIR}/scripts/self_check_sentry.sh" --static-only

log "Checking launch syntax"
ros2 launch behavior_tree sentry_all.launch.py --show-args >/dev/null

log "PC self-check completed"
