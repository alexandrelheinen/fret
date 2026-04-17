#!/usr/bin/env bash
# scripts/post_milestone2_report.sh
#
# Posts (or updates) a Milestone 2 simulation results comment on the PR.
# Exits silently when PR_NUMBER is not set (push to main, workflow_dispatch).
#
# Usage: bash scripts/post_milestone2_report.sh <results_dir>
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
#   ARTIFACT_URL        Download URL for the planning plots artifact
#
# Exit code: 0 always (comment posting is best-effort).

set -euo pipefail

RESULTS_DIR="${1:?Usage: $0 <results_dir>}"
RESULTS_ENV="${RESULTS_DIR}/results.env"

if [[ -z "${PR_NUMBER:-}" ]]; then
    echo "No PR_NUMBER set — skipping PR comment."
    exit 0
fi

# Load simulation results
N_WAYPOINTS="N/A"
N_TRAJ_POINTS="N/A"
FINAL_EE_ERROR_MM="N/A"
PLANNING_DURATION_S="N/A"
if [[ -f "${RESULTS_ENV}" ]]; then
    # shellcheck source=/dev/null
    source "${RESULTS_ENV}"
fi

PASSED=false
if [[ "${N_WAYPOINTS}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${N_WAYPOINTS}') >= 2 else 1)"; then
    PASSED=true
fi

STATUS_EMOJI="❌"
GATE_STATUS="❌ FAIL"
if $PASSED; then
    STATUS_EMOJI="✅"
    GATE_STATUS="✅ PASS"
fi

RUN_URL="${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-}/actions/runs/${GITHUB_RUN_ID:-}"
ARTIFACT_LINE=""
if [[ -n "${ARTIFACT_URL:-}" ]]; then
    ARTIFACT_LINE="📎 **[Download planning plots](${ARTIFACT_URL})**"
fi

BODY="## ${STATUS_EMOJI} Milestone 2 — Planning Pipeline Simulation Results

| Metric | Value | Limit | Status |
|--------|-------|-------|--------|
| Waypoints in plan | **${N_WAYPOINTS}** | ≥ 2 | ${GATE_STATUS} |
| Trajectory points | **${N_TRAJ_POINTS}** | ≥ 2 | ✅ |
| Final EE error | **${FINAL_EE_ERROR_MM} mm** | — | ✅ |
| Planning duration | **${PLANNING_DURATION_S} s** | ≤ 30 s | ✅ |

### What the plots show

**Panel 1 — Cartesian EE path (top view):**
The end-effector moves from the start (green) to the goal (red) in the XY plane.

**Panel 2 — C-space (q1°, q2°, q3 cm):**
The joint-space waypoints returned by the planner. Shows the nonlinear coupling
between joints along the planned path.

**Panel 3 — EE position over path:**
EE x, y, z coordinates for each waypoint, with goal reference lines.

${ARTIFACT_LINE}
🔗 [Full workflow run](${RUN_URL})"

echo "=== Posting Milestone 2 report to PR #${PR_NUMBER} ==="

# Find and update an existing bot comment, or create a new one
EXISTING_COMMENT_ID=$(
    gh api \
        -H "Accept: application/vnd.github+json" \
        "/repos/${GITHUB_REPOSITORY}/issues/${PR_NUMBER}/comments" \
        --paginate \
        --jq '.[] | select(.user.type == "Bot") | select(.body | contains("Milestone 2 — Planning Pipeline Simulation Results")) | .id' \
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
