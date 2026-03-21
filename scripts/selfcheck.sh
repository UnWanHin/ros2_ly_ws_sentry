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
  pc       Dev-machine build + static checks.
  robot    On-robot wrapper: hardware/network checks + sentry runtime launch check.
  sentry   Core suite used by both pc/robot; supports static-only or runtime-only.

Examples:
  ./scripts/selfcheck.sh
  ./scripts/selfcheck.sh pc --no-build
  ./scripts/selfcheck.sh sentry --runtime-only --launch --wait 12 --skip-hz
EOF
}

run_entry() {
  local entry="${1:-}"
  shift || true
  case "${entry,,}" in
    1|pc)
      exec "${ROOT_DIR}/selfcheck/pc.sh" "$@"
      ;;
    2|robot)
      exec "${ROOT_DIR}/selfcheck/robot.sh" "$@"
      ;;
    3|sentry|suite)
      exec "${ROOT_DIR}/selfcheck/sentry.sh" "$@"
      ;;
    ""|menu)
      ;;
    *)
      echo "[ERROR] Unknown self-check entry: ${entry}" >&2
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

echo "Select self-check entry:"
echo "  1) pc      - build + static"
echo "  2) robot   - hardware + runtime wrapper"
echo "  3) sentry  - core static/runtime suite"
read -r -p "Input 1-3 [default: 3]: " choice
choice="${choice:-3}"
run_entry "${choice}"
