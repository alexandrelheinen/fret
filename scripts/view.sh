#!/usr/bin/env bash
# Open the interactive MuJoCo 3D viewer for FRET showcase scenarios (v1.0 PPP).
#
# Requires: pip install mujoco
#
# Examples:
#   ./scripts/view.sh
#   ./scripts/view.sh --scenario ppp_warehouse --duration 45
#
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${SCRIPT_DIR}/view_mujoco.py" "$@"
