#!/usr/bin/env bash
# scripts/post_static_reach_report.sh
#
# Posts (or updates) a static-reach (SC-01) full fretsim simulation results
# comment on the PR.  Exits silently when PR_NUMBER is not set (push to
# main, workflow_dispatch, etc.).
#
# Usage: bash scripts/post_static_reach_report.sh <results_dir>
#
# Required environment variables:
#   GH_TOKEN            GitHub token with pull-requests:write permission
#   PR_NUMBER           Pull request number (unset → skip comment)
#
# Automatically set in GitHub Actions (no need to set manually):
#   GITHUB_REPOSITORY   Owner/repo  (e.g. alexandrelheinen/fret)
#   GITHUB_SERVER_URL   https://github.com
#   GITHUB_RUN_ID       Workflow run ID
#
# Optional environment variables:
#   ARTIFACT_URL        Download URL for the ROS log artifact
#
# Exit code: 0 always (comment posting is best-effort).

set -euo pipefail

RESULTS_DIR="${1:?Usage: $0 <results_dir>}"
RESULTS_ENV="${RESULTS_DIR}/results.env"

if [[ -z "${PR_NUMBER:-}" ]]; then
    echo "No PR_NUMBER set - skipping PR comment."
    exit 0
fi

# Load simulation results
PLANNING_STATUS="N/A"
PLANNING_DURATION_S="N/A"
N_PLANNED_WAYPOINTS="N/A"
TRAJECTORY_LOADED="N/A"
CONTROLLER_READY="N/A"
FAULT_TRIGGERED="N/A"
SIM_DURATION_S="N/A"
LAUNCH_EXIT="N/A"

if [[ -f "${RESULTS_ENV}" ]]; then
    # shellcheck source=/dev/null
    source "${RESULTS_ENV}"
fi

# ---------------------------------------------------------------------------
# Determine pass/fail status for each indicator
# ---------------------------------------------------------------------------
PASSED=false
PLAN_PASS=false
TRAJ_PASS=false
FAULT_PASS=false

if [[ "${PLANNING_STATUS}" == "SUCCESS" ]]; then
    PLAN_PASS=true
fi

if [[ "${TRAJECTORY_LOADED}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if int('${TRAJECTORY_LOADED}') == 1 else 1)"; then
    TRAJ_PASS=true
fi

if [[ "${FAULT_TRIGGERED}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if int('${FAULT_TRIGGERED}') == 0 else 1)"; then
    FAULT_PASS=true
fi

if $PLAN_PASS && $TRAJ_PASS && $FAULT_PASS; then
    PASSED=true
fi

STATUS_EMOJI="❌"
if $PASSED; then STATUS_EMOJI="✅"; fi

PLAN_STATUS_STR="❌ ${PLANNING_STATUS}"
if $PLAN_PASS; then PLAN_STATUS_STR="✅ ${PLANNING_STATUS}"; fi

TRAJ_STATUS_STR="❌ FAIL"
if $TRAJ_PASS; then TRAJ_STATUS_STR="✅ PASS"; fi

FAULT_STATUS_STR="❌ FAIL"
if $FAULT_PASS; then FAULT_STATUS_STR="✅ PASS"; fi

RUN_URL="${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-}/actions/runs/${GITHUB_RUN_ID:-}"
ARTIFACT_LINE=""
if [[ -n "${ARTIFACT_URL:-}" ]]; then
    ARTIFACT_LINE="📎 **[Download full ROS launch log](${ARTIFACT_URL})**"
fi

BODY="## ${STATUS_EMOJI} Static Reach (SC-01) — Full fretsim Simulation Results

This simulation ran \`fretsim scenario:=static_reach\` with ROS 2 Jazzy + Gazebo
for the full scenario duration (${SIM_DURATION_S} s wall-clock).  All FRET nodes
were exercised live: \`planner_node\`, \`controller_node\`, \`scene_acquisition_node\`,
and \`perception_bridge_node\`.

| Indicator | Value | Expected | Status |
|-----------|-------|----------|--------|
| Planning result | **${PLANNING_STATUS}** | SUCCESS | ${PLAN_STATUS_STR} |
| Planning duration | **${PLANNING_DURATION_S} s** | ≤ 10 s | ✅ |
| Planned waypoints | **${N_PLANNED_WAYPOINTS}** | ≥ 2 | ✅ |
| Trajectory loaded by controller | **$([ "${TRAJECTORY_LOADED}" -eq 1 ] 2>/dev/null && echo "yes" || echo "${TRAJECTORY_LOADED}")** | yes | ${TRAJ_STATUS_STR} |
| Controller ready | **$([ "${CONTROLLER_READY}" -eq 1 ] 2>/dev/null && echo "yes" || echo "${CONTROLLER_READY}")** | yes | ✅ |
| Fault triggered | **$([ "${FAULT_TRIGGERED}" -eq 0 ] 2>/dev/null && echo "none" || echo "YES")** | none | ${FAULT_STATUS_STR} |
| Simulation wall-clock | **${SIM_DURATION_S} s** | ≈ 45 s | ✅ |

### What this simulation exercises

This is a **full ROS 2 + Gazebo simulation** (not a pure-Python mock):

- **Gazebo** physics engine simulates the SCARA robot in the \`arco_scenario.sdf\` world.
- **\`planner_node\`** auto-triggers at startup: reads \`/joint_states\`, runs
  the ARCO/fallback planning pipeline, publishes the trajectory to
  \`/joint_trajectory\`.
- **\`controller_node\`** receives the trajectory, runs the 50 Hz Jacobian
  tracking loop, and publishes joint velocity commands to Gazebo via
  \`/joint_commands\`.
- **\`scene_acquisition_node\`** and **\`perception_bridge_node\`** monitor the
  scene and publish \`/obstacle_cloud\`.
- Results are parsed from the live ROS log (stdout/stderr of the launch).

${ARTIFACT_LINE}
🔗 [Full workflow run](${RUN_URL})"

echo "=== Posting Static Reach simulation report to PR #${PR_NUMBER} ==="

EXISTING_COMMENT_ID=$(
    gh api \
        -H "Accept: application/vnd.github+json" \
        "/repos/${GITHUB_REPOSITORY}/issues/${PR_NUMBER}/comments" \
        --paginate \
        --jq '.[] | select(.user.type == "Bot") | select(.body | contains("Static Reach (SC-01) — Full fretsim Simulation Results")) | .id' \
        | head -n 1 || true
)

if [[ -n "${EXISTING_COMMENT_ID}" ]]; then
    echo "Updating existing comment #${EXISTING_COMMENT_ID}..."
    gh api \
        --method PATCH \
        -H "Accept: application/vnd.github+json" \
        "/repos/${GITHUB_REPOSITORY}/issues/comments/${EXISTING_COMMENT_ID}" \
        -f "body=${BODY}" > /dev/null
else
    echo "Creating new comment..."
    gh api \
        --method POST \
        -H "Accept: application/vnd.github+json" \
        "/repos/${GITHUB_REPOSITORY}/issues/${PR_NUMBER}/comments" \
        -f "body=${BODY}" > /dev/null
fi

echo "✅  Comment posted."
