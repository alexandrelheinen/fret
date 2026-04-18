#!/usr/bin/env bash
# scripts/simulate_milestone5.sh
#
# Runs the Milestone 5 pure-Python pillar-avoidance simulation (planning +
# occupancy validation + tracking), generates diagnostic plots, and writes
# results to an output directory.  No ROS or Gazebo required.
#
# Requires:
#   - Python packages: numpy, pyyaml, matplotlib
#   - fret package installed (pip install -e . --no-deps)
#
# Usage:
#   bash scripts/simulate_milestone5.sh [--output <dir>]
#
# Options:
#   --output <dir>   Directory for output files (default: /tmp/sim_output_m5)
#
# Outputs:
#   <dir>/pillar_avoidance_plots.png  — 4-panel diagnostic figure
#   <dir>/results.env                 — KEY=VALUE file with simulation metrics
#
# Exit code: 0 = simulation passed, 1 = failed.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Milestone 5 simulation"' ERR

OUTPUT_DIR="/tmp/sim_output_m5"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output) OUTPUT_DIR="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,/^[^#]/{ /^#/{ s/^# \?//; p } }' "${BASH_SOURCE[0]}" | head -25
            exit 0
            ;;
        *) fail "Unknown argument: $1"; exit 1 ;;
    esac
done

require_command python3 "python3 is required."

mkdir -p "${OUTPUT_DIR}"

echo "=== Milestone 5 simulation (pillar avoidance: planning + tracking) ==="
info "Output directory: ${OUTPUT_DIR}"

if python3 "${SCRIPT_DIR}/simulate_milestone5_pipeline.py" \
    --output "${OUTPUT_DIR}" \
    --results-file "${OUTPUT_DIR}/results.env"; then
    ok "Simulation PASSED — pillars detected, path clear, EE error ≤ 5 mm"
    if [[ -f "${OUTPUT_DIR}/results.env" ]]; then
        cat "${OUTPUT_DIR}/results.env"
    fi
    exit 0
else
    fail "Simulation FAILED — pillar-avoidance pipeline did not complete successfully"
    exit 1
fi
