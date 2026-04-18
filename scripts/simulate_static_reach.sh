#!/usr/bin/env bash
# scripts/simulate_static_reach.sh
#
# Runs a full ROS 2 static-reach scenario simulation using fretsim
# (ros2 launch fret sitl.py scenario:=static_reach).
#
# The simulation runs for the full scenario duration (20 s) plus startup
# overhead.  The ROS log is captured and parsed for success indicators.
# Gazebo and all FRET nodes are exercised — no mocks or pure-Python stubs.
#
# Requires:
#   - ROS 2 Jazzy installed (/opt/ros/jazzy)
#   - Workspace built:    ./scripts/build.sh
#   - xvfb installed:     sudo apt install xvfb
#
# Usage:
#   bash scripts/simulate_static_reach.sh [--output <dir>]
#
# Options:
#   --output <dir>   Directory for log and results (default: /tmp/sim_output_static_reach)
#
# Outputs:
#   <dir>/ros_launch.log   — full stdout+stderr of the ROS 2 launch
#   <dir>/results.env      — KEY=VALUE file with parsed simulation metrics
#
# Exit code: 0 = simulation passed (planning SUCCESS, trajectory loaded, no fault),
#            1 = simulation failed or critical success indicators missing.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Static-reach simulation"' ERR

OUTPUT_DIR="/tmp/sim_output_static_reach"

# Scenario parameters (from src/fret/config/scenarios/static_reach.yml)
SCENARIO_DURATION=20     # [s]  scenario duration
STARTUP_TIMEOUT=20       # [s]  allow Gazebo + nodes to start
TOTAL_TIMEOUT=$((STARTUP_TIMEOUT + SCENARIO_DURATION + 5))  # 45 s total

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

ROS_SETUP="/opt/ros/jazzy/setup.bash"
INSTALL_SETUP="${REPO_ROOT}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    fail "ROS environment not found: ${ROS_SETUP}."
    info "Run ./scripts/install.sh to install ROS 2, then ./scripts/build.sh to build."
    exit 1
fi

if [[ ! -f "${INSTALL_SETUP}" ]]; then
    fail "Workspace overlay not found: ${INSTALL_SETUP}."
    info "Run ./scripts/build.sh to build the workspace first."
    exit 1
fi

set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
# shellcheck source=/dev/null
source "${INSTALL_SETUP}"
set -u

require_command xvfb-run "xvfb-run is required. Run: sudo apt install xvfb"
require_command ros2 "ros2 not found. Source /opt/ros/jazzy/setup.bash first."
require_command python3 "python3 is required."

mkdir -p "${OUTPUT_DIR}"

LOG_FILE="${OUTPUT_DIR}/ros_launch.log"
RESULTS_ENV="${OUTPUT_DIR}/results.env"

echo "=== Static Reach (SC-01) — full fretsim ROS 2 simulation ==="
info "Scenario  : static_reach  (duration: ${SCENARIO_DURATION} s)"
info "Timeout   : ${TOTAL_TIMEOUT} s  (startup ${STARTUP_TIMEOUT} s + ${SCENARIO_DURATION} s + 5 s buffer)"
info "Log file  : ${LOG_FILE}"
info "Output    : ${OUTPUT_DIR}"
echo ""

# ---------------------------------------------------------------------------
# Launch fretsim and capture full log output.
# Exit codes:
#   0   — launch exited cleanly (should not happen with infinite spin)
#   124 — timeout (normal: simulation ran for the full allotted time)
#   *   — launch crashed before completing
# ---------------------------------------------------------------------------
SIM_START=$(date +%s)

set +e
timeout "${TOTAL_TIMEOUT}s" \
    xvfb-run -a \
    ros2 launch fret sitl.py \
        model:=scara \
        scenario:=static_reach \
    >"${LOG_FILE}" 2>&1
LAUNCH_EXIT=$?
set -e

SIM_END=$(date +%s)
SIM_DURATION_S=$(( SIM_END - SIM_START ))

if [[ "${LAUNCH_EXIT}" -ne 0 && "${LAUNCH_EXIT}" -ne 124 ]]; then
    fail "fretsim launch crashed (exit ${LAUNCH_EXIT})."
    echo "--- Last 30 lines of launch log ---"
    tail -n 30 "${LOG_FILE}" || true
    # Write a minimal failed results.env before exiting
    cat >"${RESULTS_ENV}" <<EOF
PLANNING_STATUS=LAUNCH_CRASH
LAUNCH_EXIT=${LAUNCH_EXIT}
SIM_DURATION_S=${SIM_DURATION_S}
PLANNING_DURATION_S=N/A
N_PLANNED_WAYPOINTS=N/A
TRAJECTORY_LOADED=0
CONTROLLER_READY=0
FAULT_TRIGGERED=N/A
EOF
    exit 1
fi

if [[ "${LAUNCH_EXIT}" -eq 124 ]]; then
    info "Launch killed by timeout after ${SIM_DURATION_S} s (expected — full duration run)."
else
    info "Launch exited cleanly after ${SIM_DURATION_S} s."
fi

# ---------------------------------------------------------------------------
# Parse the ROS log for key success indicators.
# ---------------------------------------------------------------------------
info "Parsing log for success indicators…"

PLANNING_STATUS="UNKNOWN"
PLANNING_DURATION_S="N/A"
N_PLANNED_WAYPOINTS="N/A"
TRAJECTORY_LOADED=0
CONTROLLER_READY=0
FAULT_TRIGGERED=0

# PlannerNode: planning SUCCESS in 0.42 s — 2 waypoints
if grep -qE "PlannerNode: planning SUCCESS" "${LOG_FILE}" 2>/dev/null; then
    PLANNING_STATUS="SUCCESS"
    # Extract duration: "SUCCESS in X.XX s"
    PLANNING_DURATION_S=$(
        grep -oE "planning SUCCESS in [0-9]+\.[0-9]+ s" "${LOG_FILE}" \
        | grep -oE "[0-9]+\.[0-9]+" | head -1 || echo "N/A"
    )
    # Extract waypoints: "— N waypoints"
    N_PLANNED_WAYPOINTS=$(
        grep -oE "[0-9]+ waypoints" "${LOG_FILE}" \
        | grep -oE "^[0-9]+" | head -1 || echo "N/A"
    )
fi

# PlannerNode: planning FAILED
if grep -qE "PlannerNode: planning FAILED" "${LOG_FILE}" 2>/dev/null; then
    PLANNING_STATUS="FAILED"
fi

# ControllerRosNode ready
if grep -qE "ControllerRosNode ready" "${LOG_FILE}" 2>/dev/null; then
    CONTROLLER_READY=1
fi

# Trajectory loaded: N waypoints.
if grep -qE "Trajectory loaded:" "${LOG_FILE}" 2>/dev/null; then
    TRAJECTORY_LOADED=1
    # Use waypoint count from controller if not already extracted from planner
    if [[ "${N_PLANNED_WAYPOINTS}" == "N/A" ]]; then
        N_PLANNED_WAYPOINTS=$(
            grep -oE "Trajectory loaded: [0-9]+ waypoints" "${LOG_FILE}" \
            | grep -oE "[0-9]+" | head -1 || echo "N/A"
        )
    fi
fi

# Fault: /controller_fault or HALTED or fault messages
if grep -qiE "\[ERROR\].*[Ff]ault|[Ff]ault.*triggered|HALTED" "${LOG_FILE}" 2>/dev/null; then
    FAULT_TRIGGERED=1
fi

# ---------------------------------------------------------------------------
# Write results.env
# ---------------------------------------------------------------------------
cat >"${RESULTS_ENV}" <<EOF
PLANNING_STATUS=${PLANNING_STATUS}
PLANNING_DURATION_S=${PLANNING_DURATION_S}
N_PLANNED_WAYPOINTS=${N_PLANNED_WAYPOINTS}
TRAJECTORY_LOADED=${TRAJECTORY_LOADED}
CONTROLLER_READY=${CONTROLLER_READY}
FAULT_TRIGGERED=${FAULT_TRIGGERED}
SIM_DURATION_S=${SIM_DURATION_S}
LAUNCH_EXIT=${LAUNCH_EXIT}
EOF

info "Results:"
cat "${RESULTS_ENV}"
echo ""

# ---------------------------------------------------------------------------
# Determine pass/fail
# ---------------------------------------------------------------------------
FAILED=0

if [[ "${PLANNING_STATUS}" != "SUCCESS" ]]; then
    fail "Planning did not succeed (status: ${PLANNING_STATUS})."
    FAILED=1
fi

if [[ "${TRAJECTORY_LOADED}" -ne 1 ]]; then
    fail "Trajectory was not loaded by the controller."
    FAILED=1
fi

if [[ "${FAULT_TRIGGERED}" -ne 0 ]]; then
    fail "Controller fault was triggered during execution."
    FAILED=1
fi

if [[ "${FAILED}" -ne 0 ]]; then
    fail "Static-reach simulation FAILED."
    echo "--- Last 50 lines of launch log ---"
    tail -n 50 "${LOG_FILE}" || true
    exit 1
fi

ok "Static-reach simulation PASSED — planning ${PLANNING_STATUS} in ${PLANNING_DURATION_S} s, ${N_PLANNED_WAYPOINTS} waypoints, no fault."
exit 0
