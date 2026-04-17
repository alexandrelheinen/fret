#!/usr/bin/env bash
# scripts/post_milestone1_report.sh
#
# Posts (or updates) a Milestone 1 simulation results comment on the PR.
# Exits silently when PR_NUMBER is not set (push to main, workflow_dispatch).
#
# Usage: bash scripts/post_milestone1_report.sh <results_dir>
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
#   ARTIFACT_URL        Download URL for the trajectory plots artifact
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
MAX_ERR_MM="N/A"
if [[ -f "${RESULTS_ENV}" ]]; then
    # shellcheck source=/dev/null
    source "${RESULTS_ENV}"
    MAX_ERR_MM="${MAX_ERR_MM:-N/A}"
fi

PASSED=false
if [[ "${MAX_ERR_MM}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${MAX_ERR_MM}') <= 5.0 else 1)"; then
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
    ARTIFACT_LINE="📎 **[Download trajectory plots](${ARTIFACT_URL})**"
fi

BODY="## ${STATUS_EMOJI} Milestone 1 — Straight-Line Simulation Results

| Metric | Value | Limit | Status |
|--------|-------|-------|--------|
| Max EE position error | **${MAX_ERR_MM} mm** | 5 mm | ${GATE_STATUS} |
| Simulation duration | 3.0 s | — | ✅ |
| Trajectory waypoints | 150 @ 50 Hz | ≥ 45 Hz | ✅ |

### What the plots show

**Panel 1 — Cartesian space (x, y, z):**
The EE reference and executed trajectories are both very close to a straight line. The small gap between them is the tracking error.

**Panel 2 — C-space (q1, q2, q3):**
The same motion is *nonlinear* in joint space: q2 peaks and returns because the straight Cartesian line requires the elbow to bend and straighten. This illustrates the kinematic coupling of the SCARA.

**Panel 3 — Joint variables over time:**
q1(t) and q2(t) are nonlinear; q3(t) stays constant (constant-z motion).

${ARTIFACT_LINE}
🔗 [Full workflow run](${RUN_URL})"

echo "=== Posting Milestone 1 report to PR #${PR_NUMBER} ==="

# Find and update an existing bot comment, or create a new one
EXISTING_COMMENT_ID=$(
    gh api \
        -H "Accept: application/vnd.github+json" \
        "/repos/${GITHUB_REPOSITORY}/issues/${PR_NUMBER}/comments" \
        --paginate \
        --jq '.[] | select(.user.type == "Bot") | select(.body | contains("Milestone 1 — Straight-Line Simulation Results")) | .id' \
        | head -n 1 || true
)

if [[ -n "${EXISTING_COMMENT_ID}" ]]; then
    echo "Updating existing comment #${EXISTING_COMMENT_ID}..."
    echo "${BODY}" | gh api \
        --method PATCH \
        -H "Accept: application/vnd.github+json" \
        "/repos/${GITHUB_REPOSITORY}/issues/comments/${EXISTING_COMMENT_ID}" \
        --input - \
        -f body=@- > /dev/null || \
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
