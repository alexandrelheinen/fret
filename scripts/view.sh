#!/usr/bin/env bash
# Open the interactive MuJoCo 3D viewer for FRET showcase scenarios.
#
# Requires: pip install mujoco
# All showcase parameters must be passed explicitly (no defaults).
#
# Examples:
#   ./scripts/view.sh --model ppp --scenario ppp_warehouse \
#       --duration 30 --fps 60 --camera overview
#   ./scripts/view.sh --model ppp --scenario ppp_warehouse \
#       --duration 30 --fps 60 --camera overview --dry-run
#
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -eq 0 ]]; then
  echo "missing arguments" >&2
  exec python3 "${SCRIPT_DIR}/view_mujoco.py" --help
fi

exec python3 "${SCRIPT_DIR}/view_mujoco.py" "$@"
