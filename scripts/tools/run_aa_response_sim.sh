#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="${ROOT_DIR}/scripts/tools/aa_response_sim.c"
BIN="/tmp/aa_response_sim.$$"

if ! command -v gcc >/dev/null 2>&1; then
  echo "[ERROR] gcc not found." >&2
  exit 127
fi

if ! pkg-config --exists sdl2; then
  echo "[ERROR] SDL2 dev package not found (pkg-config sdl2)." >&2
  echo "        install: sudo apt-get install libsdl2-dev" >&2
  exit 2
fi

echo "[INFO] build ${SRC} -> ${BIN}"
gcc -O2 -std=c11 -Wall -Wextra -o "${BIN}" "${SRC}" $(pkg-config --cflags --libs sdl2) -lm

echo "[INFO] run ${BIN} $*"
exec "${BIN}" "$@"
