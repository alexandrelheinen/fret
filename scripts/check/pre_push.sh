#!/usr/bin/env bash
# scripts/check/pre_push.sh
#
# Master pre-push validation — runs every required CI gate locally.
# All contributors and AI agents should run this before pushing.
#
# Gates:
#   1. scripts/check/formatting.sh   — black + isort + clang-format
#   2. scripts/check/types.sh        — mypy strict
#   3. scripts/tests/unit.sh         — pytest unit tests [requires ROS workspace]
#   4. scripts/tests/smoke.sh        — ROS 2 launch smoke tests [requires ROS + xvfb]
#   5. scripts/tests/integration.sh  — launch_testing scenarios [requires ROS + xvfb]
#
# Usage:
#   bash scripts/check/pre_push.sh [--skip-ros]
#
# Options:
#   --skip-ros   Skip gates 3–5 that require a built ROS 2 workspace.
#
# Exit code: 0 = all gates pass, 1 = at least one gate failed.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
source "${REPO_ROOT}/scripts/common.sh"

SKIP_ROS=0

for arg in "$@"; do
    case "$arg" in
        --skip-ros) SKIP_ROS=1 ;;
        -h|--help)
            sed -n '2,/^[^#]/{ /^#/{ s/^# \?//; p } }' "${BASH_SOURCE[0]}" | head -20
            exit 0
            ;;
        *) fail "Unknown argument: ${arg}"; exit 1 ;;
    esac
done

echo "╔══════════════════════════════════════╗"
echo "║        FRET pre-push validation       ║"
echo "╚══════════════════════════════════════╝"

FAILED=0

run_gate() {
    local NAME="$1"
    local SCRIPT="$2"
    shift 2
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  Gate: ${NAME}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    if bash "${SCRIPT}" "$@"; then
        ok "${NAME} — PASSED"
    else
        fail "${NAME} — FAILED"
        FAILED=$((FAILED + 1))
    fi
}

run_gate "Formatting (black + isort + clang-format)" \
    "${REPO_ROOT}/scripts/check/formatting.sh"
run_gate "Type check (mypy)" \
    "${REPO_ROOT}/scripts/check/types.sh"

if [[ "${SKIP_ROS}" -eq 0 ]]; then
    run_gate "Unit tests (pytest)" \
        "${REPO_ROOT}/scripts/tests/unit.sh"
    run_gate "Smoke tests (ROS 2 launch)" \
        "${REPO_ROOT}/scripts/tests/smoke.sh"
    run_gate "Integration tests (launch_testing)" \
        "${REPO_ROOT}/scripts/tests/integration.sh"
else
    warn "Skipping ROS-dependent gates (--skip-ros)."
    warn "Run manually when ROS 2 is available:"
    warn "  bash scripts/tests/unit.sh"
    warn "  bash scripts/tests/smoke.sh"
    warn "  bash scripts/tests/integration.sh"
fi

echo ""
echo "╔══════════════════════════════════════╗"
if [[ "${FAILED}" -eq 0 ]]; then
    echo "║  ✅  ALL GATES PASSED               ║"
else
    printf "║  ❌  %d GATE(S) FAILED               ║\n" "${FAILED}"
fi
echo "╚══════════════════════════════════════╝"

exit "${FAILED}"
