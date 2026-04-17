#!/usr/bin/env bash
# scripts/simulate_arc.sh
#
# Runs the arc scenario (SC-05) pure-Python simulation, generates diagnostic
# plots, and writes results to an output directory.
# No ROS or Gazebo required — pure Python only.
#
# Requires:
#   - Python packages: numpy, matplotlib
#   - fret package installed (pip install -e . --no-deps)
#
# Usage:
#   bash scripts/simulate_arc.sh [--output <dir>]
#
# Options:
#   --output <dir>   Directory for output files (default: /tmp/sim_output_arc)
#
# Outputs:
#   <dir>/arc_plots.png     — 3-panel arc diagnostic figure
#   <dir>/results.env       — KEY=VALUE file with simulation metrics
#
# Exit code: 0 = simulation passed (max EE error ≤ 5 mm, no fault), 1 = failed.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Arc scenario simulation"' ERR

OUTPUT_DIR="/tmp/sim_output_arc"

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

echo "=== Arc scenario simulation (SC-05: circular arc trajectory + tracking) ==="
info "Output directory: ${OUTPUT_DIR}"

if python3 "${SCRIPT_DIR}/simulate_arc_pipeline.py" \
    --output "${OUTPUT_DIR}" \
    --results-file "${OUTPUT_DIR}/results.env"; then
    ok "Simulation PASSED — max EE error ≤ 5 mm, no fault triggered"
    if [[ -f "${OUTPUT_DIR}/results.env" ]]; then
        cat "${OUTPUT_DIR}/results.env"
    fi
    exit 0
else
    fail "Simulation FAILED — arc trajectory simulation did not complete successfully"
    exit 1
fi
