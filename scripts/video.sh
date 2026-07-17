#!/usr/bin/env bash
# Render a headless MuJoCo MP4 for FRET showcase scenarios (Dubins v1.1).
#
# Thin wrapper around scripts/render_mujoco.py.  Requires optional deps:
#   pip install mujoco imageio imageio-ffmpeg
#
# All showcase parameters must be passed explicitly (no defaults).
#
# Examples:
#   ./scripts/video.sh --model dubins --scenario dubins_race --camera overview \
#       -o /tmp/v11.mp4 --fps 30 --width 1280 --height 720 \
#       --collision-backend mujoco --planner-algorithm rrt_star --full-duration
#   ./scripts/video.sh --model dubins --scenario dubins_race --all-cameras \
#       --output-dir /tmp/showcase --fps 30 --width 1280 --height 720 \
#       --collision-backend mujoco --planner-algorithm rrt_star --full-duration
#
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -eq 0 ]]; then
  echo "missing arguments" >&2
  exec python3 "${SCRIPT_DIR}/render_mujoco.py" --help
fi

exec python3 "${SCRIPT_DIR}/render_mujoco.py" "$@"
