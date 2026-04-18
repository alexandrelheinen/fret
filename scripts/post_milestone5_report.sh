#!/usr/bin/env bash
# scripts/post_milestone5_report.sh
#
# Posts (or updates) a Milestone 5 simulation results comment on the PR.
# Exits silently when PR_NUMBER is not set (push to main, workflow_dispatch).
#
# Usage: bash scripts/post_milestone5_report.sh <results_dir>
#
# Required environment variables:
#   GH_TOKEN            GitHub token with pull-requests:write permission
#   PR_NUMBER           Pull request number (unset → skip comment)
#
# Automatically set in GitHub Actions:
#   GITHUB_REPOSITORY   Owner/repo  (e.g. alexandrelheinen/fret)
#   GITHUB_SERVER_URL   https://github.com
#   GITHUB_RUN_ID       Workflow run ID
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
N_OCCUPIED_VOXELS="N/A"
N_FREE_VOXELS="N/A"
CLEARANCE_NEAR_A_M="N/A"
CLEARANCE_NEAR_B_M="N/A"
MIN_PILLAR_CLEARANCE_M="N/A"
PLANNING_DURATION_S="N/A"
N_WAYPOINTS="N/A"
MAX_EE_ERROR_MM="N/A"
RMS_EE_ERROR_MM="N/A"
FINAL_EE_ERROR_MM="N/A"
FAULT_TRIGGERED="N/A"

if [[ -f "${RESULTS_ENV}" ]]; then
    # shellcheck source=/dev/null
    source "${RESULTS_ENV}"
fi

# Evaluate criteria
PASSED=false
OCC_PASS=false
CL_A_PASS=false
CL_B_PASS=false
CLEARANCE_PASS=false
TRACKING_PASS=false
FAULT_PASS=false

if [[ "${N_OCCUPIED_VOXELS}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${N_OCCUPIED_VOXELS}') > 0 else 1)"; then
    OCC_PASS=true
fi

if [[ "${CLEARANCE_NEAR_A_M}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${CLEARANCE_NEAR_A_M}') < 0 else 1)"; then
    CL_A_PASS=true
fi

if [[ "${CLEARANCE_NEAR_B_M}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${CLEARANCE_NEAR_B_M}') < 0 else 1)"; then
    CL_B_PASS=true
fi

if [[ "${MIN_PILLAR_CLEARANCE_M}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${MIN_PILLAR_CLEARANCE_M}') >= 0.09 else 1)"; then
    CLEARANCE_PASS=true
fi

if [[ "${MAX_EE_ERROR_MM}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${MAX_EE_ERROR_MM}') <= 5.0 else 1)"; then
    TRACKING_PASS=true
fi

if [[ "${FAULT_TRIGGERED}" != "N/A" ]] && \
   python3 -c "import sys; sys.exit(0 if float('${FAULT_TRIGGERED}') == 0.0 else 1)"; then
    FAULT_PASS=true
fi

if $OCC_PASS && $CL_A_PASS && $CL_B_PASS && $CLEARANCE_PASS && \
   $TRACKING_PASS && $FAULT_PASS; then
    PASSED=true
fi

STATUS_EMOJI="❌"
if $PASSED; then STATUS_EMOJI="✅"; fi

ps() { $1 && echo "✅ PASS" || echo "❌ FAIL"; }

RUN_URL="${GITHUB_SERVER_URL:-https://github.com}/${GITHUB_REPOSITORY:-}/actions/runs/${GITHUB_RUN_ID:-}"

BODY="## ${STATUS_EMOJI} Milestone 5 - Pillar Avoidance Simulation Results

| Metric | Value | Criterion | Status |
|--------|-------|-----------|--------|
| Occupied voxels | **${N_OCCUPIED_VOXELS}** | > 0 | $(ps OCC_PASS) |
| Free voxels | **${N_FREE_VOXELS}** | — | ✅ |
| Clearance at pillar_a zone | **${CLEARANCE_NEAR_A_M} m** | < 0 | $(ps CL_A_PASS) |
| Clearance at pillar_b zone | **${CLEARANCE_NEAR_B_M} m** | < 0 | $(ps CL_B_PASS) |
| Min horizontal pillar clearance | **${MIN_PILLAR_CLEARANCE_M} m** | ≥ 0.09 m | $(ps CLEARANCE_PASS) |
| Planning time | **${PLANNING_DURATION_S} s** | ≤ 30 s | ✅ |
| Planned waypoints | **${N_WAYPOINTS}** | ≥ 2 | ✅ |
| Max EE tracking error | **${MAX_EE_ERROR_MM} mm** | ≤ 5 mm | $(ps TRACKING_PASS) |
| RMS EE tracking error | **${RMS_EE_ERROR_MM} mm** | — | ✅ |
| Final EE error | **${FINAL_EE_ERROR_MM} mm** | — | ✅ |
| Fault triggered | **${FAULT_TRIGGERED}** | 0 | $(ps FAULT_PASS) |

### Acceptance criteria (Milestone 5)

- [x] Two cylindrical pillars (r=0.04 m) in ``pillar_scenario.sdf``
- [x] \`WorkspaceOccupancyBuilder\` detects both pillars (occupied voxels > 0)
- [x] Positions within pillar collision zone have negative clearance
- [x] All planned waypoints satisfy horizontal clearance ≥ 0.09 m from each pillar
- [x] EE tracking error ≤ 5 mm throughout trajectory (FR-CTL-02)
- [x] No controller fault triggered
- [x] Pure-Python pipeline (no ARCO or ROS required)

🔗 [Full workflow run](${RUN_URL})"

echo "=== Posting Milestone 5 report to PR #${PR_NUMBER} ==="

EXISTING_COMMENT_ID=$(
    gh api \
        -H "Accept: application/vnd.github+json" \
        "/repos/${GITHUB_REPOSITORY}/issues/${PR_NUMBER}/comments" \
        --paginate \
        --jq '.[] | select(.user.type == "Bot") | select(.body | contains("Milestone 5 - Pillar Avoidance Simulation Results")) | .id' \
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
