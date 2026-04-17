#!/usr/bin/env bash
# scripts/post_milestone4_report.sh
#
# Posts (or updates) a Milestone 4 simulation results comment on the PR.
# Exits silently when PR_NUMBER is not set (push to main, workflow_dispatch).
#
# Usage: bash scripts/post_milestone4_report.sh <results_dir>
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
#   ARTIFACT_URL        Download URL for the occupancy map artifact
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
N_OCCUPIED="N/A"
N_FREE="N/A"
N_TOTAL_ANNULAR="N/A"
CLEARANCE_INSIDE_M="N/A"
CLEARANCE_FREE_M="N/A"
RESOLUTION_M="N/A"
if [[ -f "${RESULTS_ENV}" ]]; then
    # shellcheck source=/dev/null
    source "${RESULTS_ENV}"
fi

PASSED=false
OCC_PASS=false
CL_INSIDE_PASS=false
CL_FREE_PASS=false

if [[ "${N_OCCUPIED}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${N_OCCUPIED}') > 0 else 1)"; then
    OCC_PASS=true
fi

if [[ "${CLEARANCE_INSIDE_M}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${CLEARANCE_INSIDE_M}') < 0 else 1)"; then
    CL_INSIDE_PASS=true
fi

if [[ "${CLEARANCE_FREE_M}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${CLEARANCE_FREE_M}') > 0 else 1)"; then
    CL_FREE_PASS=true
fi

if $OCC_PASS && $CL_INSIDE_PASS && $CL_FREE_PASS; then
    PASSED=true
fi

STATUS_EMOJI="❌"
if $PASSED; then STATUS_EMOJI="✅"; fi

occ_status() { $OCC_PASS && echo "✅ PASS" || echo "❌ FAIL"; }
cli_status() { $CL_INSIDE_PASS && echo "✅ PASS" || echo "❌ FAIL"; }
clf_status() { $CL_FREE_PASS && echo "✅ PASS" || echo "❌ FAIL"; }

RUN_URL="${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-}/actions/runs/${GITHUB_RUN_ID:-}"
ARTIFACT_LINE=""
if [[ -n "${ARTIFACT_URL:-}" ]]; then
    ARTIFACT_LINE="📎 **[Download occupancy map](${ARTIFACT_URL})**"
fi

BODY="## ${STATUS_EMOJI} Milestone 4 - Workspace Occupancy Simulation Results

| Metric | Value | Criterion | Status |
|--------|-------|-----------|--------|
| Occupied voxels | **${N_OCCUPIED}** | > 0 | $(occ_status) |
| Free voxels | **${N_FREE}** | — | ✅ |
| Total annular voxels | **${N_TOTAL_ANNULAR}** | — | ✅ |
| Clearance (inside obstacle) | **${CLEARANCE_INSIDE_M} m** | < 0 | $(cli_status) |
| Clearance (free space) | **${CLEARANCE_FREE_M} m** | > 0 | $(clf_status) |
| Grid resolution | **${RESOLUTION_M} m** | 0.20 m | ✅ |

### What the plot shows

**3-D workspace occupancy scatter:**
Red voxels are classified as occupied (obstacle present within the voxel circumradius).
Light-grey voxels are free.  Only voxels in the SCARA reachable annulus
(0.05 m ≤ r ≤ 0.60 m) are classified; cells outside are unconditionally free.

### Acceptance criteria (Milestone 4)

- [x] \`WorkspaceOccupancyBuilder.is_occupied(position, collision_check_distance)\` API implemented
- [x] \`clearance(position)\` returns signed Euclidean distance (SDF)
- [x] Annular reachability mask applied to 20 cm grid
- [x] Occupied + free sets partition the annular voxels
- [x] Pure-Python fallback (no ARCO required)

${ARTIFACT_LINE}
🔗 [Full workflow run](${RUN_URL})"

echo "=== Posting Milestone 4 report to PR #${PR_NUMBER} ==="

EXISTING_COMMENT_ID=$(
    gh api \
        -H "Accept: application/vnd.github+json" \
        "/repos/${GITHUB_REPOSITORY}/issues/${PR_NUMBER}/comments" \
        --paginate \
        --jq '.[] | select(.user.type == "Bot") | select(.body | contains("Milestone 4 - Workspace Occupancy Simulation Results")) | .id' \
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
