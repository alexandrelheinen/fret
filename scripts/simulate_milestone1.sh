#!/usr/bin/env bash
# scripts/simulate_milestone1.sh
#
# Runs the Milestone 1 pure-Python straight-line tracking simulation,
# generates diagnostic plots, and writes results to an output directory.
# No ROS or Gazebo required — pure Python only.
#
# Requires:
#   - Python packages: numpy, pyyaml, matplotlib
#   - fret package installed (pip install -e src/fret --no-deps)
#
# Usage:
#   bash scripts/simulate_milestone1.sh [--output <dir>]
#
# Options:
#   --output <dir>   Directory for output files (default: /tmp/sim_output)
#
# Outputs:
#   <dir>/trajectory_plots.png   — 3-panel trajectory diagnostic figure
#   <dir>/results.env            — KEY=VALUE file with simulation metrics
#
# Exit code: 0 = simulation passed (EE error ≤ 5 mm), 1 = failed.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Milestone 1 simulation"' ERR

OUTPUT_DIR="/tmp/sim_output"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output) OUTPUT_DIR="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,/^[^#]/{ /^#/{ s/^# \?//; p } }' "${BASH_SOURCE[0]}" | head -20
            exit 0
            ;;
        *) fail "Unknown argument: $1"; exit 1 ;;
    esac
done

require_command python3 "python3 is required."

mkdir -p "${OUTPUT_DIR}"

echo "=== Milestone 1 simulation (straight-line tracking) ==="
info "Output directory: ${OUTPUT_DIR}"

if python3 "${SCRIPT_DIR}/simulate_straight_line.py" \
    --output "${OUTPUT_DIR}" \
    --results-file "${OUTPUT_DIR}/results.env"; then
    ok "Simulation PASSED — EE error ≤ 5 mm"
    if [[ -f "${OUTPUT_DIR}/results.env" ]]; then
        cat "${OUTPUT_DIR}/results.env"
    fi
    exit 0
else
    fail "Simulation FAILED — EE error exceeds 5 mm threshold"
    exit 1
fi
