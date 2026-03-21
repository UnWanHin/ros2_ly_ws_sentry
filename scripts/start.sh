#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [entry] [args...]

Entries:
  gated                 Competition start with is_start gate.
  nogate                Competition start without is_start gate.

Examples:
  ./scripts/start.sh
  ./scripts/start.sh gated --mode league
  ./scripts/start.sh nogate --mode regional

Extra direct entry:
  ./scripts/start.sh showcase
EOF
}

has_mode_arg() {
  local arg
  for arg in "$@"; do
    if [[ "${arg}" == "--mode" ]] || [[ "${arg}" == competition_profile:=* ]] || [[ "${arg}" == --competition_profile:=* ]]; then
      return 0
    fi
  done
  return 1
}

has_help_arg() {
  local arg
  for arg in "$@"; do
    if [[ "${arg}" == "--help" || "${arg}" == "-h" ]]; then
      return 0
    fi
  done
  return 1
}

ensure_match_mode() {
  if has_help_arg "$@"; then
    printf '%s\0' "$@"
    return 0
  fi

  if has_mode_arg "$@"; then
    printf '%s\0' "$@"
    return 0
  fi

  local mode=""
  if [[ -t 0 ]]; then
    echo "Select competition framework:"
    echo "  1) league"
    echo "  2) regional"
    read -r -p "Input 1-2 [default: 2]: " mode
    mode="${mode:-2}"
    case "${mode,,}" in
      1|league)
        mode="league"
        ;;
      2|regional)
        mode="regional"
        ;;
      *)
        echo "[ERROR] Invalid mode selection: ${mode}" >&2
        exit 2
        ;;
    esac
  else
    mode="regional"
  fi

  printf '%s\0' --mode "${mode}" --no-prompt "$@"
}

run_match() {
  local target="$1"
  shift || true

  local -a forwarded=()
  while IFS= read -r -d '' item; do
    forwarded+=("${item}")
  done < <(ensure_match_mode "$@")

  exec "${target}" "${forwarded[@]}"
}

run_entry() {
  local entry="${1:-}"
  shift || true
  case "${entry,,}" in
    1|gated|gate|with-gate|with_gate)
      run_match "${ROOT_DIR}/start/sentry_all.sh" "$@"
      ;;
    2|nogate|no-gate|no_gate|without-gate|without_gate)
      run_match "${ROOT_DIR}/start/sentry_all_nogate.sh" "$@"
      ;;
    showcase)
      exec "${ROOT_DIR}/start/showcase.sh" "$@"
      ;;
    ""|menu)
      ;;
    *)
      echo "[ERROR] Unknown start entry: ${entry}" >&2
      usage >&2
      exit 2
      ;;
  esac
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

if (( $# > 0 )); then
  run_entry "$@"
fi

echo "Select start entry:"
echo "  1) gated"
echo "  2) nogate"
read -r -p "Input 1-2 [default: 1]: " choice
choice="${choice:-1}"
run_entry "${choice}"
