#!/usr/bin/env bash
# scripts/simulate_milestone4.sh
#
# Runs the Milestone 4 pure-Python workspace occupancy simulation,
# generates a 3-D occupancy scatter plot, and writes results to an output
# directory.  No ROS or Gazebo required — pure Python only.
#
# Requires:
#   - Python packages: numpy, matplotlib
#   - fret package installed (pip install -e . --no-deps)
#
# Usage:
#   bash scripts/simulate_milestone4.sh [--output <dir>]
#
# Options:
#   --output <dir>   Directory for output files (default: /tmp/sim_output_m4)
#
# Outputs:
#   <dir>/occupancy_map.png   — 3-D voxel occupancy scatter figure
#   <dir>/results.env         — KEY=VALUE file with simulation metrics
#
# Exit code: 0 = simulation passed, 1 = failed.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Milestone 4 simulation"' ERR

OUTPUT_DIR="/tmp/sim_output_m4"

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

echo "=== Milestone 4 simulation (workspace occupancy map) ==="
info "Output directory: ${OUTPUT_DIR}"

if python3 "${SCRIPT_DIR}/simulate_milestone4_pipeline.py" \
    --output "${OUTPUT_DIR}" \
    --results-file "${OUTPUT_DIR}/results.env"; then
    ok "Simulation PASSED — workspace occupancy built and validated"
    if [[ -f "${OUTPUT_DIR}/results.env" ]]; then
        cat "${OUTPUT_DIR}/results.env"
    fi
    exit 0
else
    fail "Simulation FAILED — workspace occupancy pipeline did not complete successfully"
    exit 1
fi
