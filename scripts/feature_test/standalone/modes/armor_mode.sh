#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SCRIPT_DIR="${ROOT_DIR}/scripts/feature_test/standalone/modes"
LOCK_FILE="/tmp/sentry_standalone_test.lock"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/feature_test/standalone/lib/common.sh"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [start_autoaim_debug args]

Purpose:
  Standalone armor-board mode (auto_aim + mapper), single-command and single-lock.

Defaults:
  --mode fire --online

Examples:
  ./scripts/feature_test/standalone/modes/armor_mode.sh
  ./scripts/feature_test/standalone/modes/armor_mode.sh --offline
  ./scripts/feature_test/standalone/modes/armor_mode.sh --target-priority "6,3,4,5" --target-id 6
EOF
}

HAS_NET_MODE=0
ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --offline|--online)
      HAS_NET_MODE=1
      ARGS+=("$1")
      shift
      ;;
    *)
      ARGS+=("$1")
      shift
      ;;
  esac
done

st_acquire_lock "${LOCK_FILE}"
cleanup() {
  st_release_lock "${LOCK_FILE}"
}
trap cleanup EXIT INT TERM

if (( HAS_NET_MODE == 0 )); then
  ARGS=("--online" "${ARGS[@]}")
fi

cd "${ROOT_DIR}"
"${ROOT_DIR}/scripts/launch/start_autoaim_debug.sh" --mode fire "${ARGS[@]}"
exit $?
