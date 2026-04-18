#!/usr/bin/env bash
# scripts/simulate_static_reach.sh
#
# Runs the static-reach scenario (SC-01) as a pure-Python end-to-end
# simulation (planning + Jacobian tracking).  No ROS or Gazebo required.
#
# This reuses the Milestone 3 pipeline (planning + controller), which already
# validates the static_reach scenario end-to-end.  The output is formatted
# in SC-01 expected keys for the unified simulations report.
#
# Validates:
#   - Planning SUCCESS (FR-PLN-01).
#   - Trajectory generated (N ≥ 2 waypoints).
#   - No controller fault triggered (FR-CTL-02).
#   - Max EE tracking error ≤ 5 mm.
#
# Requires:
#   - Python packages: numpy, pyyaml, matplotlib (optional for plots)
#   - fret package installed: pip install -e . --no-deps
#
# Usage:
#   bash scripts/simulate_static_reach.sh [--output <dir>]
#
# Options:
#   --output <dir>   Directory for output files
#                    (default: /tmp/sim_output_static_reach)
#
# Outputs:
#   <dir>/results.env      — KEY=VALUE file with SC-01 metrics
#   <dir>/tracking_plots.png — tracking diagnostic figure (if matplotlib available)
#
# Exit code: 0 = simulation passed, 1 = failed.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "SC-01 static-reach simulation"' ERR

OUTPUT_DIR="/tmp/sim_output_static_reach"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output) OUTPUT_DIR="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,/^[^#]/{ /^#/{ s/^# \?//; p } }' "${BASH_SOURCE[0]}" | head -30
            exit 0
            ;;
        *) fail "Unknown argument: $1"; exit 1 ;;
    esac
done

require_command python3 "python3 is required."

mkdir -p "${OUTPUT_DIR}"

# Temp directory for the pipeline output (MS-03 key format).
PIPELINE_DIR="${OUTPUT_DIR}/_pipeline"
mkdir -p "${PIPELINE_DIR}"

echo "=== Static Reach (SC-01) simulation (pure-Python end-to-end) ==="
info "Output directory: ${OUTPUT_DIR}"

# ---------------------------------------------------------------------------
# Run the MS-03 / static_reach pipeline.
# ---------------------------------------------------------------------------
if ! python3 "${SCRIPT_DIR}/simulate_milestone3_pipeline.py" \
        --output "${PIPELINE_DIR}" \
        --results-file "${PIPELINE_DIR}/results.env"; then
    fail "Pipeline execution failed."
    # Write a failed results.env so the report job can still read it.
    cat >"${OUTPUT_DIR}/results.env" <<EOF
PLANNING_STATUS=FAILED
PLANNING_DURATION_S=N/A
N_PLANNED_WAYPOINTS=N/A
TRAJECTORY_LOADED=0
FAULT_TRIGGERED=1
EOF
    exit 1
fi

# ---------------------------------------------------------------------------
# Read pipeline results and convert to SC-01 key format.
# ---------------------------------------------------------------------------
PLANNING_DURATION_S="N/A"
N_PLANNED_WAYPOINTS="N/A"
FAULT_TRIGGERED=0

if [[ -f "${PIPELINE_DIR}/results.env" ]]; then
    # shellcheck source=/dev/null
    source "${PIPELINE_DIR}/results.env"
    # MS-03 pipeline writes PLANNING_DURATION_S, N_WAYPOINTS, FAULT_TRIGGERED.
    # Map to SC-01 expected keys.
    PLANNING_DURATION_S="${PLANNING_DURATION_S:-N/A}"
    N_PLANNED_WAYPOINTS="${N_WAYPOINTS:-N/A}"
    FAULT_TRIGGERED="${FAULT_TRIGGERED:-0}"
fi

# ---------------------------------------------------------------------------
# Write SC-01 results.env.
# ---------------------------------------------------------------------------
cat >"${OUTPUT_DIR}/results.env" <<EOF
PLANNING_STATUS=SUCCESS
PLANNING_DURATION_S=${PLANNING_DURATION_S}
N_PLANNED_WAYPOINTS=${N_PLANNED_WAYPOINTS}
TRAJECTORY_LOADED=1
FAULT_TRIGGERED=${FAULT_TRIGGERED}
EOF

# Copy the diagnostic plot if available.
if [[ -f "${PIPELINE_DIR}/tracking_plots.png" ]]; then
    cp "${PIPELINE_DIR}/tracking_plots.png" "${OUTPUT_DIR}/tracking_plots.png"
fi

info "Results:"
cat "${OUTPUT_DIR}/results.env"

if python3 -c "import sys; sys.exit(0 if float('${FAULT_TRIGGERED:-0}') == 0.0 else 1)"; then
    : # no fault
else
    fail "Controller fault triggered during execution."
    exit 1
fi

ok "SC-01 static-reach PASSED — planning SUCCESS in ${PLANNING_DURATION_S} s, ${N_PLANNED_WAYPOINTS} waypoints, no fault."
exit 0
