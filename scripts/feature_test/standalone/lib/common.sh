#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.


st_info() {
  printf "[STANDALONE][INFO] %s\n" "$*"
}

st_warn() {
  printf "[STANDALONE][WARN] %s\n" "$*" >&2
}

st_err() {
  printf "[STANDALONE][ERROR] %s\n" "$*" >&2
}

st_acquire_lock() {
  local lock_file="${1:?lock_file required}"
  if ( set -o noclobber; echo "$$" >"${lock_file}" ) 2>/dev/null; then
    return 0
  fi

  local owner="unknown"
  if [[ -f "${lock_file}" ]]; then
    owner="$(cat "${lock_file}" 2>/dev/null || true)"
  fi

  if [[ "${owner}" =~ ^[0-9]+$ ]] && kill -0 "${owner}" 2>/dev/null; then
    st_err "Another standalone test is running (pid=${owner}, lock=${lock_file})."
    return 1
  fi

  st_warn "Stale lock detected, reclaiming: ${lock_file}"
  rm -f "${lock_file}" || true
  if ( set -o noclobber; echo "$$" >"${lock_file}" ) 2>/dev/null; then
    return 0
  fi
  st_err "Failed to acquire lock: ${lock_file}"
  return 1
}

st_release_lock() {
  local lock_file="${1:?lock_file required}"
  if [[ ! -f "${lock_file}" ]]; then
    return 0
  fi

  local owner
  owner="$(cat "${lock_file}" 2>/dev/null || true)"
  if [[ "${owner}" == "$$" ]]; then
    rm -f "${lock_file}" || true
  fi
}

st_stop_pid() {
  local pid="${1:-}"
  local label="${2:-process}"
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi
  st_info "Stopping ${label} (pid=${pid})"
  kill -INT "${pid}" 2>/dev/null || true
  sleep 1
  kill -TERM "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}
