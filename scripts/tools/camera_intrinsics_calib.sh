#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

SIZE="8x11"
SQUARE="0.024"
PATTERN="chessboard"
CAMERA_NAME="ly_camera"
NO_SERVICE_CHECK=1

COMPRESSED_TOPIC="/ly/compressed/image"
CALIB_TOPIC="/calib/image"
RAW_TOPIC=""
NO_REPUBLISH=0

CHARUCO_MARKER_SIZE=""
ARUCO_DICT=""

EXTRA_ARGS=()
REPUBLISH_PID=""

usage() {
  cat <<'EOF'
Usage:
  camera_intrinsics_calib.sh [options] [-- <extra_cameracalibrator_args...>]

Purpose:
  Launch ROS2 camera_calibration/cameracalibrator for this workspace.
  Default input is /ly/compressed/image, auto-republished to /calib/image.

Options:
  --size <NxM>                 Chessboard interior corners. Default: 8x11
  --square <meters>            Chessboard square size in meters. Default: 0.024
  --pattern <name>             chessboard|circles|acircles|charuco. Default: chessboard
  --camera-name <name>         camera_name written in output yaml. Default: ly_camera
  --compressed-topic <topic>   Compressed source topic. Default: /ly/compressed/image
  --calib-topic <topic>        Raw topic for calibrator (and republish out). Default: /calib/image
  --raw-topic <topic>          Use existing raw topic directly (skip republish).
  --no-republish               Skip republish and subscribe calibrator to --calib-topic.
  --no-service-check           Disable service check (default enabled in this script).
  --with-service-check         Enable service check.
  --charuco-marker-size <m>    Required when --pattern charuco.
  --aruco-dict <name>          Required when --pattern charuco.
  -h, --help                   Show this help.

Examples:
  ./scripts/tools/camera_intrinsics_calib.sh
  ./scripts/tools/camera_intrinsics_calib.sh --size 11x8 --square 0.024
  ./scripts/tools/camera_intrinsics_calib.sh --raw-topic /camera/image_raw
  ./scripts/tools/camera_intrinsics_calib.sh --no-republish --calib-topic /camera/image_raw
EOF
}

require_arg() {
  local flag="$1"
  local value="${2:-}"
  if [[ -z "${value}" ]] || [[ "${value}" == -* ]]; then
    echo "[ERROR] ${flag} requires a value." >&2
    exit 2
  fi
}

cleanup() {
  if [[ -n "${REPUBLISH_PID}" ]] && kill -0 "${REPUBLISH_PID}" 2>/dev/null; then
    kill -INT "${REPUBLISH_PID}" 2>/dev/null || true
    wait "${REPUBLISH_PID}" 2>/dev/null || true
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --size)
      require_arg "$1" "${2:-}"
      SIZE="$2"
      shift 2
      ;;
    --square)
      require_arg "$1" "${2:-}"
      SQUARE="$2"
      shift 2
      ;;
    --pattern)
      require_arg "$1" "${2:-}"
      PATTERN="$2"
      shift 2
      ;;
    --camera-name)
      require_arg "$1" "${2:-}"
      CAMERA_NAME="$2"
      shift 2
      ;;
    --compressed-topic)
      require_arg "$1" "${2:-}"
      COMPRESSED_TOPIC="$2"
      shift 2
      ;;
    --calib-topic)
      require_arg "$1" "${2:-}"
      CALIB_TOPIC="$2"
      shift 2
      ;;
    --raw-topic)
      require_arg "$1" "${2:-}"
      RAW_TOPIC="$2"
      NO_REPUBLISH=1
      shift 2
      ;;
    --no-republish)
      NO_REPUBLISH=1
      shift
      ;;
    --no-service-check)
      NO_SERVICE_CHECK=1
      shift
      ;;
    --with-service-check)
      NO_SERVICE_CHECK=0
      shift
      ;;
    --charuco-marker-size)
      require_arg "$1" "${2:-}"
      CHARUCO_MARKER_SIZE="$2"
      shift 2
      ;;
    --aruco-dict)
      require_arg "$1" "${2:-}"
      ARUCO_DICT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS=("$@")
      break
      ;;
    *)
      echo "[ERROR] Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${SIZE}" != *x* ]]; then
  echo "[ERROR] --size must be in NxM format, e.g. 8x11 or 11x8." >&2
  exit 2
fi

case "${PATTERN}" in
  chessboard|circles|acircles|charuco)
    ;;
  *)
    echo "[ERROR] --pattern must be one of: chessboard|circles|acircles|charuco." >&2
    exit 2
    ;;
esac

if [[ "${PATTERN}" == "charuco" ]]; then
  if [[ -z "${CHARUCO_MARKER_SIZE}" || -z "${ARUCO_DICT}" ]]; then
    echo "[ERROR] charuco pattern requires --charuco-marker-size and --aruco-dict." >&2
    exit 2
  fi
fi

source_ros_workspace "${ROOT_DIR}"

if ! ros2 pkg executables camera_calibration 2>/dev/null | grep -q 'cameracalibrator'; then
  echo "[ERROR] camera_calibration/cameracalibrator not found in current ROS environment." >&2
  exit 1
fi

input_topic="${CALIB_TOPIC}"
if [[ -n "${RAW_TOPIC}" ]]; then
  input_topic="${RAW_TOPIC}"
fi

trap cleanup EXIT INT TERM

if (( NO_REPUBLISH == 0 )); then
  if ! ros2 pkg executables image_transport 2>/dev/null | grep -q 'republish'; then
    echo "[ERROR] image_transport/republish not found in current ROS environment." >&2
    exit 1
  fi
  echo "[INFO] republish: ${COMPRESSED_TOPIC} (compressed) -> ${CALIB_TOPIC} (raw)"
  ros2 run image_transport republish compressed raw \
    --ros-args \
    -r "in/compressed:=${COMPRESSED_TOPIC}" \
    -r "out:=${CALIB_TOPIC}" &
  REPUBLISH_PID="$!"
  sleep 1
else
  if [[ -z "${RAW_TOPIC}" ]]; then
    input_topic="${CALIB_TOPIC}"
  fi
fi

cmd=(
  ros2 run camera_calibration cameracalibrator
  --size "${SIZE}"
  --square "${SQUARE}"
  --pattern "${PATTERN}"
  --camera_name "${CAMERA_NAME}"
)

if (( NO_SERVICE_CHECK == 1 )); then
  cmd+=(--no-service-check)
fi

if [[ "${PATTERN}" == "charuco" ]]; then
  cmd+=(--charuco_marker_size "${CHARUCO_MARKER_SIZE}" --aruco_dict "${ARUCO_DICT}")
fi

if (( ${#EXTRA_ARGS[@]} > 0 )); then
  cmd+=("${EXTRA_ARGS[@]}")
fi

cmd+=(--ros-args -r "image:=${input_topic}")

echo "[INFO] calibrator input topic: ${input_topic}"
echo "[INFO] board size: ${SIZE}, square: ${SQUARE} m, pattern: ${PATTERN}"
printf '[INFO] running:'
printf ' %q' "${cmd[@]}"
printf '\n'

"${cmd[@]}"
