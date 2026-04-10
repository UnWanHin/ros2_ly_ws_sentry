#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

OUTPUT_MODE="screen"
BASE_CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
BUFF_CONFIG_FILE="${ROOT_DIR}/src/buff_hitter/config/buff_config.yaml"
CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"
CALIB_CONFIG_FILE="${ROOT_DIR}/src/buff_shooting_table_calib/config/buff_shooting_table_calib_config.yaml"

CALIB_MODE="static"
RECORD_DIR="${HOME:-.}/workspace/record"
CSV_STRATEGY="new"
CSV_PATH=""
REQUIRE_VALID_DEBUG="true"
SAMPLE_ON_RISING_EDGE="true"
MIN_SAMPLE_INTERVAL_SEC="0.08"

FIT_MODE=""
FIT_SCOPE=""
FIT_CSV=""
MODE_FILTER="all"
INCLUDE_SMALL_MODE=0
APPLY_ON_BIG_ONLY="true"
MIN_SAMPLES=""
WRITE_CONFIG=""
OUTPUT_FIT_YAML=""

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <extra ros2 launch args...>]

Launch options:
  --base-config <path>             Base shared YAML.
  --buff-config <path>             Buff module YAML.
  --config <path>                  Global override YAML.
  --calib-config <path>            Buff calib module YAML.
  --calib-mode <static|periodic|all>
  --record-dir <path>
  --csv-strategy <new|latest>
  --csv-path <path>
  --require-valid-debug <true|false>
  --sample-on-rising-edge <true|false>
  --min-sample-interval-sec <float>
  --output <screen|log>

Fit-only options:
  --fit-static-latest              Fit newest CSV with static model and exit.
  --fit-static-all                 Fit all CSV with static model and exit.
  --fit-static-csv <path>          Fit one CSV with static model and exit.
  --fit-periodic-latest            Fit newest CSV with periodic model and exit.
  --fit-periodic-all               Fit all CSV with periodic model and exit.
  --fit-periodic-csv <path>        Fit one CSV with periodic model and exit.
  --mode-filter <all|small|big>    Static fit mode filter. Default: all.
  --include-small-mode             Periodic fit also uses mode=1 samples.
  --apply-on-big-only <true|false> Periodic output key periodic_apply_on_big_buff_only.
  --min-samples <N>                Forward to fit script.
  --output-fit-yaml <path>         Fit output override YAML path.
  --write-config <path>            Optional in-place config write (backup is created).

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --calib-mode periodic --csv-strategy latest
  ./${SCRIPT_NAME} --fit-static-latest --write-config src/buff_hitter/config/buff_config.yaml
  ./${SCRIPT_NAME} --fit-periodic-csv ~/workspace/record/buff_shooting_table_123.csv
EOF
}

FIT_STATIC_TOOL="${ROOT_DIR}/scripts/tools/fit_buff_static_shoot_table.py"
FIT_PERIODIC_TOOL="${ROOT_DIR}/scripts/tools/fit_big_buff_moving_compensation.py"

run_fit() {
  local -a args=("--record-dir" "${RECORD_DIR}")

  if [[ "${FIT_SCOPE}" == "latest" ]]; then
    args+=("--latest")
  elif [[ "${FIT_SCOPE}" == "all" ]]; then
    args+=("--all-in-dir")
  elif [[ "${FIT_SCOPE}" == "csv" ]]; then
    args+=("${FIT_CSV}")
  fi

  if [[ -n "${MIN_SAMPLES}" ]]; then
    args+=("--min-samples" "${MIN_SAMPLES}")
  fi
  if [[ -n "${OUTPUT_FIT_YAML}" ]]; then
    args+=("--output-yaml" "${OUTPUT_FIT_YAML}")
  fi
  if [[ -n "${WRITE_CONFIG}" ]]; then
    args+=("--write-config" "${WRITE_CONFIG}")
  fi

  if [[ "${FIT_MODE}" == "static" ]]; then
    args+=("--mode-filter" "${MODE_FILTER}")
    python3 "${FIT_STATIC_TOOL}" "${args[@]}"
  elif [[ "${FIT_MODE}" == "periodic" ]]; then
    if (( INCLUDE_SMALL_MODE == 1 )); then
      args+=("--include-small-mode")
    fi
    args+=("--apply-on-big-only" "${APPLY_ON_BIG_ONLY}")
    python3 "${FIT_PERIODIC_TOOL}" "${args[@]}"
  else
    echo "[ERROR] Internal fit mode invalid: ${FIT_MODE}" >&2
    exit 2
  fi
}

EXTRA_LAUNCH_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --base-config)
      BASE_CONFIG_FILE="$2"
      shift 2
      ;;
    --buff-config)
      BUFF_CONFIG_FILE="$2"
      shift 2
      ;;
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --calib-config)
      CALIB_CONFIG_FILE="$2"
      shift 2
      ;;
    --calib-mode)
      CALIB_MODE="$2"
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
    --csv-path)
      CSV_PATH="$2"
      shift 2
      ;;
    --require-valid-debug)
      REQUIRE_VALID_DEBUG="$2"
      shift 2
      ;;
    --sample-on-rising-edge)
      SAMPLE_ON_RISING_EDGE="$2"
      shift 2
      ;;
    --min-sample-interval-sec)
      MIN_SAMPLE_INTERVAL_SEC="$2"
      shift 2
      ;;
    --output)
      OUTPUT_MODE="$2"
      shift 2
      ;;
    --fit-static-latest)
      FIT_MODE="static"
      FIT_SCOPE="latest"
      shift
      ;;
    --fit-static-all)
      FIT_MODE="static"
      FIT_SCOPE="all"
      shift
      ;;
    --fit-static-csv)
      FIT_MODE="static"
      FIT_SCOPE="csv"
      FIT_CSV="$2"
      shift 2
      ;;
    --fit-periodic-latest)
      FIT_MODE="periodic"
      FIT_SCOPE="latest"
      shift
      ;;
    --fit-periodic-all)
      FIT_MODE="periodic"
      FIT_SCOPE="all"
      shift
      ;;
    --fit-periodic-csv)
      FIT_MODE="periodic"
      FIT_SCOPE="csv"
      FIT_CSV="$2"
      shift 2
      ;;
    --mode-filter)
      MODE_FILTER="$2"
      shift 2
      ;;
    --include-small-mode)
      INCLUDE_SMALL_MODE=1
      shift
      ;;
    --apply-on-big-only)
      APPLY_ON_BIG_ONLY="$2"
      shift 2
      ;;
    --min-samples)
      MIN_SAMPLES="$2"
      shift 2
      ;;
    --output-fit-yaml)
      OUTPUT_FIT_YAML="$2"
      shift 2
      ;;
    --write-config)
      WRITE_CONFIG="$2"
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

if [[ -n "${FIT_MODE}" ]]; then
  if [[ ! -f "${FIT_STATIC_TOOL}" ]]; then
    echo "[ERROR] Missing tool: ${FIT_STATIC_TOOL}" >&2
    exit 1
  fi
  if [[ ! -f "${FIT_PERIODIC_TOOL}" ]]; then
    echo "[ERROR] Missing tool: ${FIT_PERIODIC_TOOL}" >&2
    exit 1
  fi
  run_fit
  exit $?
fi

case "${CALIB_MODE}" in
  static|periodic|all)
    ;;
  *)
    echo "[ERROR] --calib-mode only supports static|periodic|all" >&2
    exit 2
    ;;
esac

case "${CSV_STRATEGY}" in
  new|latest)
    ;;
  *)
    echo "[ERROR] --csv-strategy only supports new|latest" >&2
    exit 2
    ;;
esac

source_ros_workspace "${ROOT_DIR}"

echo "Starting Buff Shooting Table Calibration (ROS2)..."
echo "[INFO] calib_mode=${CALIB_MODE} output=${OUTPUT_MODE} record_dir=${RECORD_DIR} csv_strategy=${CSV_STRATEGY}"

exec ros2 launch buff_shooting_table_calib buff_shooting_table_calib.launch.py \
  "base_config_file:=${BASE_CONFIG_FILE}" \
  "buff_config_file:=${BUFF_CONFIG_FILE}" \
  "config_file:=${CONFIG_FILE}" \
  "calib_config_file:=${CALIB_CONFIG_FILE}" \
  "calib_mode:=${CALIB_MODE}" \
  "record_dir:=${RECORD_DIR}" \
  "csv_strategy:=${CSV_STRATEGY}" \
  "csv_path:=${CSV_PATH}" \
  "require_valid_debug:=${REQUIRE_VALID_DEBUG}" \
  "sample_on_rising_edge:=${SAMPLE_ON_RISING_EDGE}" \
  "min_sample_interval_sec:=${MIN_SAMPLE_INTERVAL_SEC}" \
  "output:=${OUTPUT_MODE}" \
  "${EXTRA_LAUNCH_ARGS[@]}"
