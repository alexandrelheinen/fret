#!/usr/bin/env bash
# scripts/post_simulations_report.sh
#
# Posts (or updates) a single PR comment with a combined simulation results
# table for ALL milestone (MS-X) and scenario (SC-X) simulations.
#
# Usage: bash scripts/post_simulations_report.sh <results_root>
#
# <results_root> must contain one sub-directory per job:
#   results_root/
#     results-ms1/results.env
#     results-ms2/results.env
#     results-ms3/results.env
#     results-ms4/results.env
#     results-sc05/results.env
#     results-sc01/results.env
#
# Required environment variables:
#   GH_TOKEN            GitHub token with pull-requests:write permission
#   PR_NUMBER           Pull request number (unset → skip comment)
#
# Optional job outcome variables (set by GitHub Actions needs context):
#   MS1_RESULT, MS2_RESULT, MS3_RESULT, MS4_RESULT, SC05_RESULT, SC01_RESULT
#   Values: "success" | "failure" | "cancelled" | "skipped"
#
# Automatically set in GitHub Actions:
#   GITHUB_REPOSITORY, GITHUB_SERVER_URL, GITHUB_RUN_ID
#
# Exit code: 0 always (comment posting is best-effort).

set -euo pipefail

RESULTS_ROOT="${1:?Usage: $0 <results_root>}"

if [[ -z "${PR_NUMBER:-}" ]]; then
    echo "No PR_NUMBER set — skipping PR comment."
    exit 0
fi

# ---------------------------------------------------------------------------
# Helper: load a results.env file and echo its content into variables.
# ---------------------------------------------------------------------------
load_env() {
    local file="$1"
    if [[ -f "${file}" ]]; then
        # shellcheck source=/dev/null
        source "${file}"
    fi
}

# ---------------------------------------------------------------------------
# Helper: render pass/fail emoji based on job outcome and value check.
# job_outcome is "success" | "failure" | "cancelled" | "skipped".
# ---------------------------------------------------------------------------
outcome_emoji() {
    local outcome="${1:-}"
    case "${outcome}" in
        success)   echo "✅" ;;
        failure)   echo "❌" ;;
        cancelled) echo "🚫" ;;
        skipped)   echo "⏭️"  ;;
        *)         echo "❓" ;;
    esac
}

bool_yes_no() {
    local val="${1:-}"
    [[ "${val}" == "1" ]] && echo "yes" || echo "no"
}

# ===========================================================================
# MS-01 — Straight-Line Tracking
# ===========================================================================
MS1_RESULT="${MS1_RESULT:-}"
MAX_ERR_MM="N/A"
MS1_PASSED="false"
load_env "${RESULTS_ROOT}/results-ms1/results.env" 2>/dev/null || true
MS1_EMOJI=$(outcome_emoji "${MS1_RESULT}")
if [[ "${MS1_PASSED}" == "true" || "${MS1_RESULT}" == "success" ]]; then
    MS1_EMOJI="✅"
elif [[ "${MS1_RESULT}" == "failure" ]]; then
    MS1_EMOJI="❌"
fi

# ===========================================================================
# MS-02 — Planning Pipeline
# ===========================================================================
MS2_RESULT="${MS2_RESULT:-}"
N_WAYPOINTS="N/A"
N_TRAJ_POINTS="N/A"
PLANNING_DURATION_S="N/A"
FINAL_EE_ERROR_MM="N/A"
load_env "${RESULTS_ROOT}/results-ms2/results.env" 2>/dev/null || true
MS2_N_WAYPOINTS="${N_WAYPOINTS}"
MS2_PLANNING_S="${PLANNING_DURATION_S}"
MS2_FINAL_ERR="${FINAL_EE_ERROR_MM}"
MS2_EMOJI=$(outcome_emoji "${MS2_RESULT}")

# ===========================================================================
# MS-03 — End-to-End Pipeline
# ===========================================================================
MS3_RESULT="${MS3_RESULT:-}"
N_WAYPOINTS="N/A"
PLANNING_DURATION_S="N/A"
MAX_EE_ERROR_MM="N/A"
RMS_EE_ERROR_MM="N/A"
FINAL_EE_ERROR_MM="N/A"
FAULT_TRIGGERED="N/A"
load_env "${RESULTS_ROOT}/results-ms3/results.env" 2>/dev/null || true
MS3_MAX_ERR="${MAX_EE_ERROR_MM}"
MS3_RMS_ERR="${RMS_EE_ERROR_MM}"
MS3_PLANNING_S="${PLANNING_DURATION_S}"
MS3_FAULT="${FAULT_TRIGGERED}"
MS3_EMOJI=$(outcome_emoji "${MS3_RESULT}")

# ===========================================================================
# MS-04 — Workspace Occupancy
# ===========================================================================
MS4_RESULT="${MS4_RESULT:-}"
N_OCCUPIED="N/A"
N_FREE="N/A"
CLEARANCE_INSIDE_M="N/A"
load_env "${RESULTS_ROOT}/results-ms4/results.env" 2>/dev/null || true
MS4_OCCUPIED="${N_OCCUPIED}"
MS4_FREE="${N_FREE}"
MS4_CLEARANCE="${CLEARANCE_INSIDE_M}"
MS4_EMOJI=$(outcome_emoji "${MS4_RESULT}")

# ===========================================================================
# SC-05 — Arc Scenario
# ===========================================================================
SC05_RESULT="${SC05_RESULT:-}"
MAX_EE_ERROR_MM="N/A"
RMS_EE_ERROR_MM="N/A"
N_WAYPOINTS="N/A"
FAULT_TRIGGERED="N/A"
load_env "${RESULTS_ROOT}/results-sc05/results.env" 2>/dev/null || true
SC05_MAX_ERR="${MAX_EE_ERROR_MM}"
SC05_RMS_ERR="${RMS_EE_ERROR_MM}"
SC05_FAULT="${FAULT_TRIGGERED}"
SC05_EMOJI=$(outcome_emoji "${SC05_RESULT}")

# ===========================================================================
# SC-01 — Static Reach (ROS 2)
# ===========================================================================
SC01_RESULT="${SC01_RESULT:-}"
PLANNING_STATUS="N/A"
PLANNING_DURATION_S="N/A"
N_PLANNED_WAYPOINTS="N/A"
TRAJECTORY_LOADED="N/A"
FAULT_TRIGGERED="N/A"
load_env "${RESULTS_ROOT}/results-sc01/results.env" 2>/dev/null || true
SC01_PLAN_STATUS="${PLANNING_STATUS}"
SC01_PLAN_S="${PLANNING_DURATION_S}"
SC01_WAYPOINTS="${N_PLANNED_WAYPOINTS}"
SC01_TRAJ_LOADED=$(bool_yes_no "${TRAJECTORY_LOADED}")
SC01_FAULT="${FAULT_TRIGGERED}"
SC01_EMOJI=$(outcome_emoji "${SC01_RESULT}")

# ---------------------------------------------------------------------------
# Overall status
# ---------------------------------------------------------------------------
ALL_PASS=true
for r in "${MS1_RESULT}" "${MS2_RESULT}" "${MS3_RESULT}" "${MS4_RESULT}" \
          "${SC05_RESULT}" "${SC01_RESULT}"; do
    if [[ "${r}" == "failure" || "${r}" == "cancelled" ]]; then
        ALL_PASS=false
        break
    fi
done

if $ALL_PASS; then
    OVERALL_EMOJI="✅"
    OVERALL_LABEL="ALL PASSED"
else
    OVERALL_EMOJI="❌"
    OVERALL_LABEL="SOME FAILURES"
fi

RUN_URL="${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-}/actions/runs/${GITHUB_RUN_ID:-}"

# ---------------------------------------------------------------------------
# Build the PR comment body
# ---------------------------------------------------------------------------
BODY="<!-- fret-simulations-report -->
${OVERALL_EMOJI} **Simulation Suite — ${OVERALL_LABEL}**

| Simulation | Status | Key Metrics |
|------------|--------|-------------|
| **MS-01** Straight-Line Tracking | ${MS1_EMOJI} | max error: **${MAX_ERR_MM} mm** |
| **MS-02** Planning Pipeline | ${MS2_EMOJI} | plan: **${MS2_PLANNING_S} s**, **${MS2_N_WAYPOINTS}** waypoints, final EE err: **${MS2_FINAL_ERR} mm** |
| **MS-03** End-to-End Pipeline | ${MS3_EMOJI} | plan: **${MS3_PLANNING_S} s**, max err: **${MS3_MAX_ERR} mm**, RMSE: **${MS3_RMS_ERR} mm**, fault: **${MS3_FAULT}** |
| **MS-04** Workspace Occupancy | ${MS4_EMOJI} | occupied: **${MS4_OCCUPIED}** voxels, free: **${MS4_FREE}** voxels, clearance: **${MS4_CLEARANCE} m** |
| **SC-05** Arc Scenario | ${SC05_EMOJI} | max err: **${SC05_MAX_ERR} mm**, RMSE: **${SC05_RMS_ERR} mm**, fault: **${SC05_FAULT}** |
| **SC-01** Static Reach *(ROS 2)* | ${SC01_EMOJI} | planning: **${SC01_PLAN_STATUS}** in **${SC01_PLAN_S} s**, **${SC01_WAYPOINTS}** wpts, traj loaded: **${SC01_TRAJ_LOADED}**, fault: **${SC01_FAULT}** |

> SC-01 runs live **ROS 2 nodes** (\`planner_node\` + \`controller_node\`) for the full
> scenario duration.  All other simulations are pure-Python (no ROS required).

🔗 [Full workflow run](${RUN_URL})"

echo "=== Posting combined simulations report to PR #${PR_NUMBER} ==="

EXISTING_COMMENT_ID=$(
    gh api \
        -H "Accept: application/vnd.github+json" \
        "/repos/${GITHUB_REPOSITORY}/issues/${PR_NUMBER}/comments" \
        --paginate \
        --jq '.[] | select(.user.type == "Bot") | select(.body | contains("<!-- fret-simulations-report -->")) | .id' \
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

echo "✅  Combined simulation report posted."
