#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

OUTPUT_MODE="screen"
USE_GIMBAL="true"
USE_CALIB="true"
TEAM_RED="true"
DEBUG_TEAM_BLUE="false"
WEB_SHOW="true"
DRAW_IMAGE="true"
CONFIG_FILE=""
PARAM_MANAGER_CMD=""
FIT_LATEST=0
FIT_ALL=0
FIT_CSV=""
AUTO_FIT=0
WRITE_CONFIG=""
OUTPUT_FIT_YAML=""
DEFAULT_RECORD_DIR="${HOME:-.}/workspace/record"
RECORD_DIR="${DEFAULT_RECORD_DIR}"
BASE_CONFIG_FILE=""
DETECTOR_CONFIG_FILE=""
PREDICTOR_CONFIG_FILE=""
DEFAULT_BASE_CONFIG="${ROOT_DIR}/scripts/config/stack/base_competition.yaml"
DEFAULT_DETECTOR_CONFIG="${ROOT_DIR}/scripts/config/stack/detector_competition.yaml"
DEFAULT_PREDICTOR_CONFIG="${ROOT_DIR}/scripts/config/stack/predictor_competition.yaml"
DEFAULT_OVERRIDE_CONFIG="${ROOT_DIR}/scripts/config/stack/override_none.yaml"
DEFAULT_WRITE_CONFIG="${ROOT_DIR}/scripts/config/stack/predictor_competition.yaml"
AUTO_FIT_EXPLICIT=0
WRITE_CONFIG_EXPLICIT=0
DISABLE_DEFAULT_WRITE_CONFIG=0
CSV_STRATEGY=""
CSV_PATH=""

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <extra ros2 launch args...>]

Options:
  --config <path>             Global override YAML (config_file).
  --base-config <path>        Base shared YAML (base_config_file).
  --detector-config <path>    Detector YAML (detector_config_file).
  --predictor-config <path>   Predictor YAML (predictor_config_file).
  --output <screen|log>       Node output mode. Default: screen.
  --use-gimbal <true|false>   Start gimbal_driver. Default: true.
  --use-calib <true|false>    Start shooting_table_calib node. Default: true.
  --team <red|blue>           Team color for calib node. Default: red.
  --web-show <true|false>     Enable web show. Default: true.
  --draw-image <true|false>   Enable image draw. Default: true.
  --record-dir <path>         Calibration CSV directory used by node save path and fitting.
  --csv-strategy <new|latest> CSV target selection. new=create timestamped CSV, latest=append newest CSV.
  --new-csv                   Shortcut for --csv-strategy new.
  --latest-csv                Shortcut for --csv-strategy latest.
  --csv-path <path>           Append/create a specific CSV file path.
  --fit-latest                Fit the newest shooting_table_*.csv and exit.
  --fit-all                   Fit all shooting_table_*.csv under --record-dir and exit.
  --fit-csv <path>            Fit a specific CSV file and exit.
  --auto-fit                  After calib exits, fit the current session CSV automatically. Default: disabled.
  --no-auto-fit               Explicitly keep CSV-only collection without post-exit fitting.
  --write-config <path>       Write fitted coefficients back into a YAML config file.
  --no-write-config           Do not sync fitted coefficients back into a config file.
  --output-fit-yaml <path>    Output ROS2 parameter override YAML path for fitted coefficients.
  --param-manager-cmd "<cmd>" Optional command to start param manager in background.
  -h, --help                  Show help.

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --latest-csv
  ./${SCRIPT_NAME} --team blue --web-show false
  ./${SCRIPT_NAME} --config /path/to/global_override.yaml
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --fit-latest --write-config scripts/config/stack/predictor_competition.yaml
  ./${SCRIPT_NAME} --auto-fit --write-config scripts/config/stack/predictor_competition.yaml
  ./${SCRIPT_NAME} --param-manager-cmd "bash /home/liu/workspace/scripts/param_manager.bash"
EOF
}

FIT_TOOL="${ROOT_DIR}/scripts/tools/fit_shooting_table.py"

run_fit() {
  local args=()
  if (( FIT_LATEST == 1 )); then
    args+=("--latest")
  fi
  if (( FIT_ALL == 1 )); then
    args+=("--all-in-dir")
  fi
  if [[ -n "${FIT_CSV}" ]]; then
    args+=("${FIT_CSV}")
  fi
  args+=("--record-dir" "${RECORD_DIR}")
  if [[ -n "${OUTPUT_FIT_YAML}" ]]; then
    args+=("--output-yaml" "${OUTPUT_FIT_YAML}")
  fi
  if [[ -n "${WRITE_CONFIG}" ]]; then
    args+=("--write-config" "${WRITE_CONFIG}")
  fi
  python3 "${FIT_TOOL}" "${args[@]}"
}

select_latest_csv_since() {
  local dir="$1"
  local min_mtime="$2"
  local best_csv=""
  local best_mtime=-1
  local csv=""
  local mtime=0

  shopt -s nullglob
  for csv in "${dir}"/shooting_table_*.csv; do
    mtime=$(stat -c %Y "${csv}" 2>/dev/null || echo 0)
    if (( mtime >= min_mtime && mtime > best_mtime )); then
      best_csv="${csv}"
      best_mtime=${mtime}
    fi
  done
  shopt -u nullglob

  if [[ -n "${best_csv}" ]]; then
    printf '%s\n' "${best_csv}"
    return 0
  fi
  return 1
}

normalize_csv_strategy() {
  local raw="${1:-}"
  local normalized="${raw,,}"
  case "${normalized}" in
    new)
      echo "new"
      return 0
      ;;
    latest|reuse|append)
      echo "latest"
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

EXTRA_LAUNCH_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --base-config)
      BASE_CONFIG_FILE="$2"
      shift 2
      ;;
    --detector-config)
      DETECTOR_CONFIG_FILE="$2"
      shift 2
      ;;
    --predictor-config)
      PREDICTOR_CONFIG_FILE="$2"
      shift 2
      ;;
    --output)
      OUTPUT_MODE="$2"
      shift 2
      ;;
    --use-gimbal)
      USE_GIMBAL="$2"
      shift 2
      ;;
    --use-calib)
      USE_CALIB="$2"
      shift 2
      ;;
    --team)
      case "$2" in
        red|RED)
          TEAM_RED="true"
          DEBUG_TEAM_BLUE="false"
          ;;
        blue|BLUE)
          TEAM_RED="false"
          DEBUG_TEAM_BLUE="true"
          ;;
        *)
          echo "[ERROR] --team only supports red|blue" >&2
          exit 2
          ;;
      esac
      shift 2
      ;;
    --web-show)
      WEB_SHOW="$2"
      shift 2
      ;;
    --draw-image)
      DRAW_IMAGE="$2"
      shift 2
      ;;
    --record-dir)
      RECORD_DIR="$2"
      shift 2
      ;;
    --csv-strategy)
      CSV_STRATEGY="$2"
      shift 2
      ;;
    --new-csv)
      CSV_STRATEGY="new"
      shift
      ;;
    --latest-csv)
      CSV_STRATEGY="latest"
      shift
      ;;
    --csv-path)
      CSV_PATH="$2"
      shift 2
      ;;
    --fit-latest)
      FIT_LATEST=1
      shift
      ;;
    --fit-all)
      FIT_ALL=1
      shift
      ;;
    --fit-csv)
      FIT_CSV="$2"
      shift 2
      ;;
    --auto-fit)
      AUTO_FIT=1
      AUTO_FIT_EXPLICIT=1
      shift
      ;;
    --no-auto-fit)
      AUTO_FIT=0
      AUTO_FIT_EXPLICIT=1
      shift
      ;;
    --write-config)
      WRITE_CONFIG="$2"
      WRITE_CONFIG_EXPLICIT=1
      shift 2
      ;;
    --no-write-config)
      WRITE_CONFIG=""
      DISABLE_DEFAULT_WRITE_CONFIG=1
      shift
      ;;
    --output-fit-yaml)
      OUTPUT_FIT_YAML="$2"
      shift 2
      ;;
    --param-manager-cmd)
      PARAM_MANAGER_CMD="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_LAUNCH_ARGS=("$@")
      break
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

FIT_MODE_COUNT=0
(( FIT_LATEST == 1 )) && FIT_MODE_COUNT=$((FIT_MODE_COUNT + 1))
(( FIT_ALL == 1 )) && FIT_MODE_COUNT=$((FIT_MODE_COUNT + 1))
[[ -n "${FIT_CSV}" ]] && FIT_MODE_COUNT=$((FIT_MODE_COUNT + 1))
if (( FIT_MODE_COUNT > 1 )); then
  echo "[ERROR] Only one of --fit-latest, --fit-all, --fit-csv may be specified." >&2
  exit 2
fi
if (( AUTO_FIT_EXPLICIT == 1 )) && (( AUTO_FIT == 1 )) && (( FIT_MODE_COUNT > 0 )); then
  echo "[ERROR] --auto-fit cannot be combined with --fit-latest/--fit-all/--fit-csv." >&2
  exit 2
fi
if [[ -n "${CSV_STRATEGY}" ]] && ! normalize_csv_strategy "${CSV_STRATEGY}" >/dev/null; then
  echo "[ERROR] --csv-strategy only supports new|latest" >&2
  exit 2
fi
if [[ -n "${CSV_PATH}" ]] && (( FIT_MODE_COUNT > 0 )); then
  echo "[ERROR] --csv-path is only for live calibration launch, not fit-only mode." >&2
  exit 2
fi

if (( FIT_MODE_COUNT > 0 )); then
  if [[ ! -f "${FIT_TOOL}" ]]; then
    echo "[ERROR] fit tool not found: ${FIT_TOOL}" >&2
    exit 1
  fi
  run_fit
  exit $?
fi

if [[ -z "${CSV_PATH}" ]] && [[ -z "${CSV_STRATEGY}" ]] && [[ "${USE_CALIB}" == "true" ]] && [[ -t 0 ]]; then
  echo "[PROMPT] CSV target mode:"
  echo "  1) new    - create a new timestamped CSV"
  echo "  2) latest - append to newest shooting_table_*.csv"
  read -r -p "Input 1 or 2 [default: 1]: " csv_choice
  csv_choice="${csv_choice:-1}"
  case "${csv_choice}" in
    1)
      CSV_STRATEGY="new"
      ;;
    2)
      CSV_STRATEGY="latest"
      ;;
    *)
      echo "[WARN] Invalid CSV choice: ${csv_choice}. Fallback to new." >&2
      CSV_STRATEGY="new"
      ;;
  esac
fi

if [[ -z "${CSV_STRATEGY}" ]]; then
  CSV_STRATEGY="new"
fi

if [[ -z "${BASE_CONFIG_FILE}" ]]; then
  BASE_CONFIG_FILE="${DEFAULT_BASE_CONFIG}"
fi
if [[ -z "${DETECTOR_CONFIG_FILE}" ]]; then
  DETECTOR_CONFIG_FILE="${DEFAULT_DETECTOR_CONFIG}"
fi
if [[ -z "${PREDICTOR_CONFIG_FILE}" ]]; then
  PREDICTOR_CONFIG_FILE="${DEFAULT_PREDICTOR_CONFIG}"
fi
if [[ -z "${CONFIG_FILE}" ]]; then
  CONFIG_FILE="${DEFAULT_OVERRIDE_CONFIG}"
fi
if [[ -z "${WRITE_CONFIG}" ]] && (( DISABLE_DEFAULT_WRITE_CONFIG == 0 )); then
  WRITE_CONFIG="${DEFAULT_WRITE_CONFIG}"
fi

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "[ERROR] ${ROOT_DIR}/install/setup.bash not found. Please run colcon build first." >&2
  exit 1
fi

# `install/setup.bash` may reference unset vars (for example COLCON_TRACE).
# Define it as empty by default so nounset is satisfied without enabling trace spam.
: "${COLCON_TRACE:=}"
export COLCON_TRACE
set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

PARAM_MANAGER_PID=""
cleanup() {
  if [[ -n "${PARAM_MANAGER_PID}" ]] && kill -0 "${PARAM_MANAGER_PID}" 2>/dev/null; then
    kill "${PARAM_MANAGER_PID}" 2>/dev/null || true
    wait "${PARAM_MANAGER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ -n "${PARAM_MANAGER_CMD}" ]]; then
  echo "[INFO] Starting param manager: ${PARAM_MANAGER_CMD}"
  bash -lc "${PARAM_MANAGER_CMD}" &
  PARAM_MANAGER_PID=$!
fi

LAUNCH_ARGS=(
  "output:=${OUTPUT_MODE}"
  "use_gimbal:=${USE_GIMBAL}"
  "use_calib:=${USE_CALIB}"
  "team_red:=${TEAM_RED}"
  "debug_team_blue:=${DEBUG_TEAM_BLUE}"
  "web_show:=${WEB_SHOW}"
  "draw_image:=${DRAW_IMAGE}"
  "record_dir:=${RECORD_DIR}"
  "csv_strategy:=${CSV_STRATEGY}"
)

if [[ ! -f "${BASE_CONFIG_FILE}" ]]; then
  echo "[ERROR] base config file not found: ${BASE_CONFIG_FILE}" >&2
  exit 1
fi
if [[ ! -f "${DETECTOR_CONFIG_FILE}" ]]; then
  echo "[ERROR] detector config file not found: ${DETECTOR_CONFIG_FILE}" >&2
  exit 1
fi
if [[ ! -f "${PREDICTOR_CONFIG_FILE}" ]]; then
  echo "[ERROR] predictor config file not found: ${PREDICTOR_CONFIG_FILE}" >&2
  exit 1
fi
if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "[ERROR] override config file not found: ${CONFIG_FILE}" >&2
  exit 1
fi
LAUNCH_ARGS+=("base_config_file:=${BASE_CONFIG_FILE}")
LAUNCH_ARGS+=("detector_config_file:=${DETECTOR_CONFIG_FILE}")
LAUNCH_ARGS+=("predictor_config_file:=${PREDICTOR_CONFIG_FILE}")
LAUNCH_ARGS+=("config_file:=${CONFIG_FILE}")
if [[ -n "${CSV_PATH}" ]]; then
  LAUNCH_ARGS+=("csv_path:=${CSV_PATH}")
fi

echo "Starting Shooting Table Calibration System (ROS2)..."
echo "===================================================="
echo "[INFO] output=${OUTPUT_MODE} use_gimbal=${USE_GIMBAL} use_calib=${USE_CALIB} team_red=${TEAM_RED} debug_team_blue=${DEBUG_TEAM_BLUE} web_show=${WEB_SHOW} draw_image=${DRAW_IMAGE}"
echo "[INFO] record_dir=${RECORD_DIR} csv_strategy=${CSV_STRATEGY} auto_fit=${AUTO_FIT}"
echo "[INFO] base_config_file=${BASE_CONFIG_FILE}"
echo "[INFO] detector_config_file=${DETECTOR_CONFIG_FILE}"
echo "[INFO] predictor_config_file=${PREDICTOR_CONFIG_FILE}"
echo "[INFO] config_file(override)=${CONFIG_FILE}"
if [[ -n "${CSV_PATH}" ]]; then
  echo "[INFO] csv_path=${CSV_PATH}"
fi
if [[ -n "${WRITE_CONFIG}" ]]; then
  echo "[INFO] write_config=${WRITE_CONFIG}"
fi
if [[ -n "${OUTPUT_FIT_YAML}" ]]; then
  echo "[INFO] output_fit_yaml=${OUTPUT_FIT_YAML}"
fi
cd "${ROOT_DIR}"
if (( AUTO_FIT == 1 )); then
  launch_start_time=$(date +%s)
  ros2 launch shooting_table_calib shooting_table_calib.launch.py \
    "${LAUNCH_ARGS[@]}" \
    "${EXTRA_LAUNCH_ARGS[@]}"
  launch_exit=$?
  if (( launch_exit != 0 )); then
    exit "${launch_exit}"
  fi
  FIT_LATEST=0
  FIT_ALL=0
  FIT_CSV="$(select_latest_csv_since "${DEFAULT_RECORD_DIR}" "${launch_start_time}" || true)"
  if [[ -n "${FIT_CSV}" ]]; then
    echo "[INFO] Auto-fitting current session CSV: ${FIT_CSV}"
  else
    FIT_LATEST=1
    echo "[WARN] Could not uniquely identify the current session CSV. Falling back to latest file in ${DEFAULT_RECORD_DIR}."
  fi
  run_fit
  exit $?
fi

exec ros2 launch shooting_table_calib shooting_table_calib.launch.py \
  "${LAUNCH_ARGS[@]}" \
  "${EXTRA_LAUNCH_ARGS[@]}"
