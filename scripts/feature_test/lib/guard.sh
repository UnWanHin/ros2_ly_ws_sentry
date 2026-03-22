#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.


ft_topic_publisher_count() {
  local topic="$1"
  local output
  output="$(timeout "${FT_CMD_TIMEOUT_SEC}s" ros2 topic info "${topic}" -v 2>/dev/null || true)"
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

ft_topic_publisher_nodes() {
  local topic="$1"
  local output
  output="$(timeout "${FT_CMD_TIMEOUT_SEC}s" ros2 topic info "${topic}" -v 2>/dev/null || true)"
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

ft_require_topic_single_publisher() {
  local topic="$1"
  local count
  count="$(ft_topic_publisher_count "${topic}")"
  if [[ -z "${count}" ]]; then
    count="0"
  fi
  if (( count > 1 )); then
    ft_err "Topic ${topic} has ${count} publishers (expected <=1)."
    local nodes
    nodes="$(ft_topic_publisher_nodes "${topic}" || true)"
    if [[ -n "${nodes}" ]]; then
      while IFS= read -r node; do
        ft_err "  publisher: ${node}"
      done <<< "${nodes}"
    fi
    return 1
  fi
  return 0
}

ft_require_topic_no_publisher() {
  local topic="$1"
  local count
  count="$(ft_topic_publisher_count "${topic}")"
  if [[ -z "${count}" ]]; then
    count="0"
  fi
  if (( count != 0 )); then
    ft_err "Topic ${topic} already has ${count} publisher(s); external test refuses takeover."
    local nodes
    nodes="$(ft_topic_publisher_nodes "${topic}" || true)"
    if [[ -n "${nodes}" ]]; then
      while IFS= read -r node; do
        ft_err "  existing publisher: ${node}"
      done <<< "${nodes}"
    fi
    return 1
  fi
  return 0
}

ft_wait_and_require_topic_single_publisher() {
  local topic="$1"
  local wait_seconds="${2:-8}"
  local require_present="${3:-1}"
  local deadline=$((SECONDS + wait_seconds))
  local count

  while (( SECONDS < deadline )); do
    count="$(ft_topic_publisher_count "${topic}")"
    if [[ -z "${count}" ]]; then
      count="0"
    fi
    if (( count > 1 )); then
      ft_require_topic_single_publisher "${topic}"
      return 1
    fi
    if (( require_present == 0 )) || (( count == 1 )); then
      return 0
    fi
    sleep 1
  done

  count="$(ft_topic_publisher_count "${topic}")"
  if (( require_present == 1 )) && (( count == 0 )); then
    ft_err "Topic ${topic} has no publisher after ${wait_seconds}s."
    return 1
  fi
  ft_require_topic_single_publisher "${topic}"
}

ft_require_behavior_tree_absent() {
  local nodes
  nodes="$(timeout "${FT_CMD_TIMEOUT_SEC}s" ros2 node list 2>/dev/null || true)"
  if grep -qx "/behavior_tree" <<< "${nodes}"; then
    ft_err "Detected /behavior_tree online. Refuse to enter external feature-test control."
    return 1
  fi
  return 0
}

ft_acquire_lock() {
  local lock_file="$1"
  if ( set -o noclobber; echo "$$" >"${lock_file}" ) 2>/dev/null; then
    return 0
  fi

  local owner="unknown"
  if [[ -f "${lock_file}" ]]; then
    owner="$(cat "${lock_file}" 2>/dev/null || true)"
  fi
  ft_err "Another feature-test instance is active (lock=${lock_file}, owner_pid=${owner})."
  return 1
}

ft_release_lock() {
  local lock_file="$1"
  if [[ -f "${lock_file}" ]]; then
    local owner
    owner="$(cat "${lock_file}" 2>/dev/null || true)"
    if [[ "${owner}" == "$$" ]]; then
      rm -f "${lock_file}"
    fi
  fi
}
