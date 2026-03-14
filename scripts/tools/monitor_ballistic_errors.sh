#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"

OUT_FILE=""
WITH_VALID=0
RAW_MODE=0
TIMEOUT_SEC=0

FAIL_PATTERN='calcPitchYawWithShootTable failed|calcPitchYaw failed|calculateBallisticSolution|Control invalid|No available armor|Invalid car id|New car id invalid|Car [0-9]+ is not stable|Critical input stale|Runtime safe-control published|Predictor target invalid, skip lock'
VALID_PATTERN='Control valid|Applied shoot table compensation|Predictor target valid, update auto-aim angles'

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--with-valid] [--raw] [--output <file>] [--timeout <sec>]

Purpose:
  Subscribe /rosout and filter ballistic/auto-aim related errors.
  Useful when 8081 has bounding boxes but gimbal does not lock.

Options:
  --with-valid         Also print success lines (Control valid / compensation).
  --raw                Keep raw /rosout style output and only grep matched lines.
  --output <file>      Also append matched lines to file.
  --timeout <sec>      Auto stop after N seconds (0 means run until Ctrl+C).
  -h, --help           Show this help.

Examples:
  ./scripts/start_ballistic_error_log.sh
  ./scripts/start_ballistic_error_log.sh --with-valid
  ./scripts/start_ballistic_error_log.sh --timeout 30 --output /tmp/ballistic.log
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-valid)
      WITH_VALID=1
      shift
      ;;
    --raw)
      RAW_MODE=1
      shift
      ;;
    --output)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --output requires a file path." >&2
        exit 2
      fi
      OUT_FILE="$2"
      shift 2
      ;;
    --timeout)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --timeout requires seconds." >&2
        exit 2
      fi
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 command not found. Please source ROS2 first." >&2
  exit 1
fi

PATTERN="${FAIL_PATTERN}"
if (( WITH_VALID == 1 )); then
  PATTERN="${PATTERN}|${VALID_PATTERN}"
fi

if [[ -n "${OUT_FILE}" ]]; then
  touch "${OUT_FILE}"
  echo "[INFO] Logging matched lines to: ${OUT_FILE}" >&2
fi

echo "[INFO] Watching /rosout for ballistic-related logs..." >&2
echo "[INFO] Pattern: ${PATTERN}" >&2

RUNNER=()
if [[ "${TIMEOUT_SEC}" != "0" ]]; then
  RUNNER=(timeout "${TIMEOUT_SEC}")
  echo "[INFO] Timeout: ${TIMEOUT_SEC}s" >&2
fi

if (( RAW_MODE == 1 )); then
  # Raw mode: quick grep from /rosout output.
  if [[ -n "${OUT_FILE}" ]]; then
    "${RUNNER[@]}" ros2 topic echo /rosout \
      | stdbuf -oL grep -E --line-buffered "${PATTERN}" \
      | tee -a "${OUT_FILE}"
  else
    "${RUNNER[@]}" ros2 topic echo /rosout \
      | stdbuf -oL grep -E --line-buffered "${PATTERN}"
  fi
  exit 0
fi

# Structured mode: print compact one-line records.
"${RUNNER[@]}" ros2 topic echo /rosout | awk \
  -v pat="${PATTERN}" \
  -v out="${OUT_FILE}" '
  function flush_line(line) {
    print line;
    fflush();
    if (out != "") {
      print line >> out;
      fflush(out);
    }
  }
  /^name:[[:space:]]*/ {
    name = $0;
    sub(/^name:[[:space:]]*/, "", name);
    next;
  }
  /^level:[[:space:]]*/ {
    level = $0;
    sub(/^level:[[:space:]]*/, "", level);
    next;
  }
  /^msg:[[:space:]]*/ {
    msg = $0;
    sub(/^msg:[[:space:]]*/, "", msg);
    if (msg ~ pat) {
      ts = strftime("%Y-%m-%d %H:%M:%S");
      line = sprintf("[%s] node=%s level=%s msg=%s", ts, name, level, msg);
      flush_line(line);
    }
    next;
  }
'
