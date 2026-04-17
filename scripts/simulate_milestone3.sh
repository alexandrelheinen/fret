#!/usr/bin/env bash
# scripts/simulate_milestone3.sh
#
# Runs the Milestone 3 pure-Python end-to-end simulation (planning + tracking),
# generates diagnostic plots, and writes results to an output directory.
# No ROS or Gazebo required — pure Python only.
#
# Requires:
#   - Python packages: numpy, pyyaml, matplotlib
#   - fret package installed (pip install -e . --no-deps)
#
# Usage:
#   bash scripts/simulate_milestone3.sh [--output <dir>]
#
# Options:
#   --output <dir>   Directory for output files (default: /tmp/sim_output_m3)
#
# Outputs:
#   <dir>/tracking_plots.png   — 3-panel tracking diagnostic figure
#   <dir>/results.env          — KEY=VALUE file with simulation metrics
#
# Exit code: 0 = simulation passed (EE error ≤ 5 mm, no fault), 1 = failed.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Milestone 3 simulation"' ERR

OUTPUT_DIR="/tmp/sim_output_m3"

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

echo "=== Milestone 3 simulation (end-to-end planning + tracking) ==="
info "Output directory: ${OUTPUT_DIR}"

if python3 "${SCRIPT_DIR}/simulate_milestone3_pipeline.py" \
    --output "${OUTPUT_DIR}" \
    --results-file "${OUTPUT_DIR}/results.env"; then
    ok "Simulation PASSED — EE error ≤ 5 mm, no fault triggered"
    if [[ -f "${OUTPUT_DIR}/results.env" ]]; then
        cat "${OUTPUT_DIR}/results.env"
    fi
    exit 0
else
    fail "Simulation FAILED — planning + tracking pipeline did not complete successfully"
    exit 1
fi
