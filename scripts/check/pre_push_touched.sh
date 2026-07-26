#!/usr/bin/env bash
# scripts/check/pre_push_touched.sh
#
# Path → CI-shard pre-push gate. Agents must run this (or an equivalent
# stronger gate) before pushing when the working tree / branch diff touches
# code or tests — not only formatting.sh + types.sh.
#
# Usage:
#   bash scripts/check/pre_push_touched.sh
#   bash scripts/check/pre_push_touched.sh --base HEAD~1
#
# Exit code: 0 = all required gates pass, 1 = failure.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
source "${REPO_ROOT}/scripts/common.sh"

BASE_REF=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --base)
            BASE_REF="${2:-}"
            if [[ -z "${BASE_REF}" ]]; then
                fail "--base requires a git ref"
                exit 1
            fi
            shift 2
            ;;
        -h|--help)
            sed -n '2,/^[^#]/{ /^#/{ s/^# \?//; p } }' "${BASH_SOURCE[0]}" | head -18
            exit 0
            ;;
        *)
            fail "Unknown argument: $1"
            exit 1
            ;;
    esac
done

cd "${REPO_ROOT}"

if [[ -z "${BASE_REF}" ]]; then
    if git rev-parse --verify origin/main >/dev/null 2>&1; then
        BASE_REF="origin/main"
    else
        BASE_REF="main"
    fi
fi

mapfile -t CHANGED < <(
    {
        git diff --name-only "${BASE_REF}...HEAD" 2>/dev/null || true
        git diff --name-only
        git diff --name-only --cached
    } | sed '/^$/d' | sort -u
)

need_control=0
need_planning=0
need_scene=0
need_simulation=0
code_changed=0

classify() {
    local f="$1"
    case "${f}" in
        *.md|docs/*|.cursor/*|LICENSE*|AUTHORS*)
            return 0
            ;;
    esac
    code_changed=1
    case "${f}" in
        src/fret/control/*|src/fret/mjcf/*|src/fret/vision/*|src/fret/hardware/*|\
        src/fret/config/scenarios/*|src/fret/config/vision/*|\
        tests/control/*|tests/vision/*|tests/hardware/*)
            need_control=1
            ;;
        src/fret/planning/*|tests/planning/*)
            need_planning=1
            ;;
        src/fret/scene/*|tests/scene/*|tests/test_*.py)
            need_scene=1
            ;;
        src/fret/simulation/*|src/fret/scenario/*|src/fret/ros/*|\
        src/fret/config/release/*|scripts/release/*|scripts/render_mujoco.py|\
        scripts/video.sh|tests/simulation/*|tests/scenario/*|tests/ros/*|\
        tests/release/*)
            need_simulation=1
            ;;
        src/fret/*|tests/*|scripts/*|pyproject.toml|setup.cfg|.github/*|\
        src/fret/package.xml)
            # Cross-cutting / unsure → all shards (matches CI risk).
            need_control=1
            need_planning=1
            need_scene=1
            need_simulation=1
            ;;
        *)
            # Unknown path under the repo — be conservative.
            need_control=1
            need_planning=1
            need_scene=1
            need_simulation=1
            ;;
    esac
}

for f in "${CHANGED[@]:-}"; do
    [[ -n "${f}" ]] || continue
    classify "${f}"
done

echo "╔══════════════════════════════════════╗"
echo "║   FRET pre-push (touched → shards)    ║"
echo "╚══════════════════════════════════════╝"
echo "Base: ${BASE_REF}"
echo "Changed files: ${#CHANGED[@]}"
echo "Shards: control=${need_control} planning=${need_planning} scene=${need_scene} simulation=${need_simulation}"

FAILED=0
run_gate() {
    local NAME="$1"
    shift
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  Gate: ${NAME}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    if "$@"; then
        ok "${NAME} — PASSED"
    else
        fail "${NAME} — FAILED"
        FAILED=$((FAILED + 1))
    fi
}

run_gate "Formatting" bash "${REPO_ROOT}/scripts/check/formatting.sh"
run_gate "Type check" bash "${REPO_ROOT}/scripts/check/types.sh"

if [[ "${code_changed}" -eq 0 ]]; then
    warn "Docs/comment-only (or empty) diff — skipping unit shards."
else
    if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
        info "Building workspace (required for unit shards)..."
        if ! bash "${REPO_ROOT}/scripts/build.sh"; then
            FAILED=$((FAILED + 1))
        fi
    fi
    [[ "${need_control}" -eq 1 ]] && \
        run_gate "Unit shard control" bash "${REPO_ROOT}/scripts/tests/unit_shard.sh" control
    [[ "${need_planning}" -eq 1 ]] && \
        run_gate "Unit shard planning" bash "${REPO_ROOT}/scripts/tests/unit_shard.sh" planning
    [[ "${need_scene}" -eq 1 ]] && \
        run_gate "Unit shard scene" bash "${REPO_ROOT}/scripts/tests/unit_shard.sh" scene
    [[ "${need_simulation}" -eq 1 ]] && \
        run_gate "Unit shard simulation" bash "${REPO_ROOT}/scripts/tests/unit_shard.sh" simulation
fi

echo ""
if [[ "${FAILED}" -eq 0 ]]; then
    ok "pre_push_touched: all required gates PASSED"
    info "Still wait for CI green after push — local green ≠ release done."
    exit 0
fi
fail "pre_push_touched: ${FAILED} gate(s) FAILED"
exit 1
