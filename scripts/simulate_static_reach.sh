#!/usr/bin/env bash
# scripts/simulate_static_reach.sh
#
# Runs the static-reach scenario (SC-01) with live ROS 2 nodes:
# planner_node and controller_node communicate over ROS 2 topics for the
# full scenario duration (20 s).  No Gazebo is required — the planner
# triggers immediately via the start_configuration parameter override
# (bypassing the /joint_states wait).
#
# This is a real ROS 2 communication test:
#   1. planner_node plans a collision-free path and publishes
#      /joint_trajectory (TRANSIENT_LOCAL).
#   2. controller_node receives /joint_trajectory and runs the
#      50 Hz Jacobian tracking loop for the scenario duration.
#
# The simulation validates:
#   - Planning SUCCESS within the timeout (FR-PLN-01).
#   - Trajectory is published and received by the controller.
#   - No fault triggered by the controller.
#
# Requires:
#   - ROS 2 Jazzy installed (/opt/ros/jazzy)
#   - Workspace built:    ./scripts/build.sh
#
# Usage:
#   bash scripts/simulate_static_reach.sh [--output <dir>]
#
# Options:
#   --output <dir>   Directory for logs and results
#                    (default: /tmp/sim_output_static_reach)
#
# Outputs:
#   <dir>/planner.log      — planner_node stdout+stderr
#   <dir>/controller.log   — controller_node stdout+stderr
#   <dir>/results.env      — KEY=VALUE file with parsed simulation metrics
#
# Exit code: 0 = planning SUCCESS and trajectory loaded (no fault),
#            1 = failure.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

OUTPUT_DIR="/tmp/sim_output_static_reach"

# Scenario duration (s) — matches static_reach.yml duration: 20.0
SCENARIO_DURATION=20
# Total time to wait: scenario duration (20 s) + 15 s margin (covers the
# 10 s planning timeout plus 5 s for node startup and ROS discovery).
TOTAL_WAIT=$((SCENARIO_DURATION + 15))

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output) OUTPUT_DIR="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,/^[^#]/{ /^#/{ s/^# \?//; p } }' "${BASH_SOURCE[0]}" | head -35
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

require_command ros2 "ros2 not found. Source /opt/ros/jazzy/setup.bash first."

mkdir -p "${OUTPUT_DIR}"

PLAN_LOG="${OUTPUT_DIR}/planner.log"
CTRL_LOG="${OUTPUT_DIR}/controller.log"
RESULTS_ENV="${OUTPUT_DIR}/results.env"

echo "=== Static Reach (SC-01) — full ROS 2 fretsim simulation ==="
info "Nodes      : planner_node + controller_node  (no Gazebo required)"
info "Duration   : ${SCENARIO_DURATION} s + 15 s margin = ${TOTAL_WAIT} s total"
info "Output     : ${OUTPUT_DIR}"
echo ""

# ---------------------------------------------------------------------------
# Launch controller_node in background.
# It will spin waiting for /joint_trajectory (TRANSIENT_LOCAL) — even if
# received before /joint_states, it logs "Trajectory loaded: N waypoints."
# and begins the tracking loop.
# ---------------------------------------------------------------------------
info "Starting controller_node…"
ros2 run fret controller_node \
    >"${CTRL_LOG}" 2>&1 &
CTRL_PID=$!

# Give the controller a moment to initialise its subscriptions before the
# planner publishes /joint_trajectory (TRANSIENT_LOCAL re-delivers, but the
# controller must be subscribed first to avoid a one-step race at startup).
sleep 2

# ---------------------------------------------------------------------------
# Launch planner_node in background.
# start_configuration default [0.0, 0.0, 0.0] is non-empty so the planner
# triggers immediately without waiting for /joint_states.
# goal_configuration and planning_timeout use scenario defaults.
# ---------------------------------------------------------------------------
info "Starting planner_node…"
ros2 run fret planner_node \
    >"${PLAN_LOG}" 2>&1 &
PLAN_PID=$!

SIM_START=$(date +%s)

# ---------------------------------------------------------------------------
# Wait for the scenario duration (+ margin), then stop the nodes.
# ---------------------------------------------------------------------------
info "Running for ${TOTAL_WAIT} s…"
sleep "${TOTAL_WAIT}"

# Graceful shutdown
kill "${PLAN_PID}" "${CTRL_PID}" 2>/dev/null || true
wait "${PLAN_PID}" 2>/dev/null || true
wait "${CTRL_PID}" 2>/dev/null || true

SIM_END=$(date +%s)
SIM_DURATION_S=$(( SIM_END - SIM_START ))

info "Nodes stopped after ${SIM_DURATION_S} s."

# ---------------------------------------------------------------------------
# Parse the ROS node logs for success indicators.
# ---------------------------------------------------------------------------
info "Parsing logs for success indicators…"

PLANNING_STATUS="UNKNOWN"
PLANNING_DURATION_S="N/A"
N_PLANNED_WAYPOINTS="N/A"
TRAJECTORY_LOADED=0
CONTROLLER_READY=0
FAULT_TRIGGERED=0

# PlannerNode: planning SUCCESS in X.XX s — N waypoints
if grep -qE "PlannerNode: planning SUCCESS" "${PLAN_LOG}" 2>/dev/null; then
    PLANNING_STATUS="SUCCESS"
    PLANNING_DURATION_S=$(
        grep -oE "planning SUCCESS in [0-9]+\.[0-9]+ s" "${PLAN_LOG}" \
        | grep -oE "[0-9]+\.[0-9]+" | head -1 || echo "N/A"
    )
    N_PLANNED_WAYPOINTS=$(
        grep -oE "[0-9]+ waypoints" "${PLAN_LOG}" \
        | grep -oE "^[0-9]+" | head -1 || echo "N/A"
    )
fi

# PlannerNode: planning FAILED
if grep -qE "PlannerNode: planning FAILED" "${PLAN_LOG}" 2>/dev/null; then
    PLANNING_STATUS="FAILED"
fi

# ControllerRosNode ready (model=scara, rate=50 Hz)
if grep -qE "ControllerRosNode ready" "${CTRL_LOG}" 2>/dev/null; then
    CONTROLLER_READY=1
fi

# Trajectory loaded: N waypoints.
if grep -qE "Trajectory loaded:" "${CTRL_LOG}" 2>/dev/null; then
    TRAJECTORY_LOADED=1
    if [[ "${N_PLANNED_WAYPOINTS}" == "N/A" ]]; then
        N_PLANNED_WAYPOINTS=$(
            grep -oE "Trajectory loaded: [0-9]+ waypoints" "${CTRL_LOG}" \
            | grep -oE "[0-9]+" | head -1 || echo "N/A"
        )
    fi
fi

# Fault: controller HALTED or error messages
if grep -qiE "\[ERROR\].*[Ff]ault|[Ff]ault.*triggered|HALTED" "${CTRL_LOG}" 2>/dev/null; then
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
    echo "--- planner.log (last 30 lines) ---"
    tail -n 30 "${PLAN_LOG}" || true
    FAILED=1
fi

if [[ "${TRAJECTORY_LOADED}" -ne 1 ]]; then
    fail "Trajectory was not loaded by the controller."
    echo "--- controller.log (last 30 lines) ---"
    tail -n 30 "${CTRL_LOG}" || true
    FAILED=1
fi

if [[ "${FAULT_TRIGGERED}" -ne 0 ]]; then
    fail "Controller fault was triggered during execution."
    FAILED=1
fi

if [[ "${FAILED}" -ne 0 ]]; then
    fail "SC-01 static-reach simulation FAILED."
    exit 1
fi

ok "SC-01 static-reach PASSED — planning ${PLANNING_STATUS} in ${PLANNING_DURATION_S} s, ${N_PLANNED_WAYPOINTS} waypoints, no fault."
exit 0
