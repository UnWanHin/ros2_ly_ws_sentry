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

collect_descendant_pids() {
  local parent_pid="$1"
  local -a children=()
  local child

  mapfile -t children < <(pgrep -P "${parent_pid}" || true)
  for child in "${children[@]}"; do
    [[ -n "${child}" ]] || continue
    echo "${child}"
    collect_descendant_pids "${child}"
  done
}

cleanup_existing_launch_tree() {
  local enabled="$1"
  local launch_regex="$2"

  if [[ "${enabled}" != "1" ]]; then
    return 0
  fi

  local -a launch_pids=()
  mapfile -t launch_pids < <(pgrep -f "${launch_regex}" || true)

  if (( ${#launch_pids[@]} == 0 )); then
    return 0
  fi

  echo "[WARN] Detected existing launch processes (strict tree cleanup):" >&2
  local pid
  local cmd
  for pid in "${launch_pids[@]}"; do
    cmd="$(ps -o args= -p "${pid}" 2>/dev/null || true)"
    if [[ -n "${cmd}" ]]; then
      echo "  ${pid} ${cmd}" >&2
    else
      echo "  ${pid}" >&2
    fi
  done

  local -a candidate_pids=()
  local child
  for pid in "${launch_pids[@]}"; do
    candidate_pids+=("${pid}")
    while IFS= read -r child; do
      [[ -n "${child}" ]] || continue
      candidate_pids+=("${child}")
    done < <(collect_descendant_pids "${pid}")
  done

  mapfile -t candidate_pids < <(printf '%s\n' "${candidate_pids[@]}" | awk 'NF && !seen[$0]++')

  echo "[INFO] Cleaning only matched launch process trees before relaunch..." >&2
  kill -INT "${launch_pids[@]}" 2>/dev/null || true
  sleep 1

  local -a alive_pids=()
  for pid in "${candidate_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      alive_pids+=("${pid}")
    fi
  done

  if (( ${#alive_pids[@]} > 0 )); then
    kill -TERM "${alive_pids[@]}" 2>/dev/null || true
    sleep 1
  fi

  alive_pids=()
  for pid in "${candidate_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      alive_pids+=("${pid}")
    fi
  done

  if (( ${#alive_pids[@]} > 0 )); then
    kill -KILL "${alive_pids[@]}" 2>/dev/null || true
  fi
}
