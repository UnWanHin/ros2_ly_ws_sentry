#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.


source_ros_workspace() {
  local root_dir="$1"

  : "${ROS_LOG_DIR:=/tmp/ros2_logs}"
  mkdir -p "${ROS_LOG_DIR}"
  export ROS_LOG_DIR

  if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
  else
    local distro
    for distro in humble iron jazzy rolling; do
      if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
        set +u
        # shellcheck disable=SC1090
        source "/opt/ros/${distro}/setup.bash"
        set -u
        break
      fi
    done
  fi

  if [[ ! -f "${root_dir}/install/setup.bash" ]]; then
    echo "[ERROR] ${root_dir}/install/setup.bash not found." >&2
    echo "        Run: colcon build" >&2
    exit 1
  fi

  set +u
  # shellcheck disable=SC1091
  source "${root_dir}/install/setup.bash"
  set -u

  cd "${root_dir}"
}

cleanup_existing_stack() {
  local enabled="$1"
  local node_regex="$2"
  local launch_regex="$3"

  if [[ "${enabled}" != "1" ]]; then
    return 0
  fi

  mapfile -t existing_stack_procs < <(
    {
      pgrep -af "${node_regex}" || true
      pgrep -af "${launch_regex}" || true
    } | awk '!seen[$0]++'
  )

  if (( ${#existing_stack_procs[@]} == 0 )); then
    return 0
  fi

  echo "[WARN] Detected existing stack-related processes:" >&2
  printf "  %s\n" "${existing_stack_procs[@]}" >&2
  echo "[INFO] Cleaning up stale processes before launch..." >&2
  pkill -f "${node_regex}" || true
  pkill -f "${launch_regex}" || true
  sleep 1
}
