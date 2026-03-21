#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.


ft_info() {
  printf "[FEATURE-TEST][INFO] %s\n" "$*"
}

ft_warn() {
  printf "[FEATURE-TEST][WARN] %s\n" "$*" >&2
}

ft_err() {
  printf "[FEATURE-TEST][ERROR] %s\n" "$*" >&2
}

ft_bool_to_word() {
  local value="${1:-0}"
  if [[ "${value}" == "1" ]]; then
    printf "true"
  else
    printf "false"
  fi
}

ft_source_ros() {
  if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    return 0
  fi

  local distro
  for distro in humble iron jazzy rolling; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "/opt/ros/${distro}/setup.bash"
      set -u
      return 0
    fi
  done

  ft_err "No ROS2 setup found under /opt/ros."
  return 1
}

ft_source_workspace() {
  local root_dir="$1"
  local setup_file="${root_dir}/install/setup.bash"
  if [[ ! -f "${setup_file}" ]]; then
    ft_err "Workspace setup not found: ${setup_file}. Run: colcon build"
    return 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

ft_print_cmd() {
  printf "%q " "$@"
}
