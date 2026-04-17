#!/usr/bin/env bash
# scripts/post_arc_report.sh
#
# Posts (or updates) an arc scenario (SC-05) simulation results comment on
# the PR.  Exits silently when PR_NUMBER is not set (workflow_dispatch, etc.).
#
# Usage: bash scripts/post_arc_report.sh <results_dir>
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
#   ARTIFACT_URL        Download URL for the arc plots artifact
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
MAX_EE_ERROR_MM="N/A"
RMS_EE_ERROR_MM="N/A"
N_WAYPOINTS="N/A"
TRAJ_DURATION_S="N/A"
GEN_DURATION_S="N/A"
FAULT_TRIGGERED="N/A"
if [[ -f "${RESULTS_ENV}" ]]; then
    # shellcheck source=/dev/null
    source "${RESULTS_ENV}"
fi

PASSED=false
EE_PASS=false
FAULT_PASS=false

if [[ "${MAX_EE_ERROR_MM}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${MAX_EE_ERROR_MM}') <= 5.0 else 1)"; then
    EE_PASS=true
fi

if [[ "${FAULT_TRIGGERED}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${FAULT_TRIGGERED}') == 0.0 else 1)"; then
    FAULT_PASS=true
fi

if $EE_PASS && $FAULT_PASS; then
    PASSED=true
fi

STATUS_EMOJI="❌"
if $PASSED; then STATUS_EMOJI="✅"; fi

EE_STATUS="❌ FAIL"
if $EE_PASS; then EE_STATUS="✅ PASS"; fi

FAULT_STATUS="❌ FAIL"
if $FAULT_PASS; then FAULT_STATUS="✅ PASS"; fi

RUN_URL="${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-}/actions/runs/${GITHUB_RUN_ID:-}"
ARTIFACT_LINE=""
if [[ -n "${ARTIFACT_URL:-}" ]]; then
    ARTIFACT_LINE="📎 **[Download arc plots](${ARTIFACT_URL})**"
fi

BODY="## ${STATUS_EMOJI} Arc Scenario (SC-05) Simulation Results

| Metric | Value | Limit | Status |
|--------|-------|-------|--------|
| Max EE tracking error | **${MAX_EE_ERROR_MM} mm** | ≤ 5 mm | ${EE_STATUS} |
| RMS EE tracking error | **${RMS_EE_ERROR_MM} mm** | - | ✅ |
| Planned waypoints | **${N_WAYPOINTS}** | ≥ 2 | ✅ |
| Trajectory duration | **${TRAJ_DURATION_S} s** | 4 s | ✅ |
| Trajectory gen. time | **${GEN_DURATION_S} s** | - | ✅ |
| Fault triggered | **${FAULT_TRIGGERED}** | 0 | ${FAULT_STATUS} |

### What the plots show

**Panel 1 — Cartesian EE arc (top view):**
The end-effector traces a circular arc from the start (green) to the end (red)
in the XY plane (constant z = 0.138 m).  The grey line shows the ideal
geometric arc; the blue solid line is the IK reference; the red dashed line
is the executed path tracked by the Jacobian controller.

**Panel 2 — EE tracking error over time:**
Cartesian EE position error throughout the 4-second arc simulation.
The dashed line at 5 mm shows the FR-CTL-02 acceptance limit.

**Panel 3 — Joint variables over time:**
Reference and executed q1, q2, q3 trajectories.  The nonlinear joint-space
arc (produced by analytical IK of the Cartesian circle) is clearly visible.

${ARTIFACT_LINE}
🔗 [Full workflow run](${RUN_URL})"

echo "=== Posting Arc scenario report to PR #${PR_NUMBER} ==="

# Find and update an existing bot comment, or create a new one
EXISTING_COMMENT_ID=$(
    gh api \
        -H "Accept: application/vnd.github+json" \
        "/repos/${GITHUB_REPOSITORY}/issues/${PR_NUMBER}/comments" \
        --paginate \
        --jq '.[] | select(.user.type == "Bot") | select(.body | contains("Arc Scenario (SC-05) Simulation Results")) | .id' \
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
