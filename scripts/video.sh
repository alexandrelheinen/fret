#!/usr/bin/env bash
# Render a headless MuJoCo MP4 for FRET showcase scenarios (PPP v1.0, Dubins v1.1).
#
# Thin wrapper around scripts/render_mujoco.py.  Requires optional deps:
#   pip install mujoco imageio imageio-ffmpeg
#
# Examples:
#   ./scripts/video.sh
#   ./scripts/video.sh --model ppp --scenario ppp_warehouse -o /tmp/v10.mp4
#   ./scripts/video.sh --all-cameras --output-dir /tmp/showcase
#   ./scripts/video.sh --collision-backend mujoco --all-cameras
#   ./scripts/video.sh --duration 15 --fps 24 --width 1920 --height 1080
#
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${SCRIPT_DIR}/render_mujoco.py" "$@"
