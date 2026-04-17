#!/usr/bin/env bash
# scripts/pre_push.sh
#
# Master pre-push validation — runs every required CI gate locally.
# All AI agents and contributors MUST run this before pushing.
#
# Gates (all required unless --skip-ros is passed):
#   1. check_formatting.sh    — black + isort + clang-format
#   2. check_types.sh         — mypy strict
#   3. simulate_milestone1.sh — Milestone 1 pure-Python simulation (no ROS)
#   4. run_tests.sh           — pytest unit tests + quality gates
#   5. run_smoke_tests.sh     — ROS 2 launch smoke tests
#
# Usage:
#   bash scripts/pre_push.sh [--skip-ros]
#
# Options:
#   --skip-ros   Skip gates 4 and 5 that require a built ROS 2 workspace.
#                Useful when running without a local ROS 2 install.
#
# Exit code: 0 = all gates pass, 1 = at least one gate failed.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

SKIP_ROS=0

for arg in "$@"; do
    case "$arg" in
        --skip-ros) SKIP_ROS=1 ;;
        -h|--help)
            sed -n '2,/^[^#]/{ /^#/{ s/^# \?//; p } }' "${BASH_SOURCE[0]}" | head -25
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

run_gate "Formatting (black + isort + clang-format)" "${SCRIPT_DIR}/check_formatting.sh"
run_gate "Type check (mypy)"                          "${SCRIPT_DIR}/check_types.sh"
run_gate "Milestone 1 simulation (no ROS)"            "${SCRIPT_DIR}/simulate_milestone1.sh"

if [[ "${SKIP_ROS}" -eq 0 ]]; then
    run_gate "Unit tests (pytest + coverage)"         "${SCRIPT_DIR}/run_tests.sh"
    run_gate "Smoke tests (ROS 2 launch)"             "${SCRIPT_DIR}/run_smoke_tests.sh"
else
    warn "Skipping ROS-dependent gates (--skip-ros)."
    warn "Run ./scripts/run_tests.sh and ./scripts/run_smoke_tests.sh manually once ROS 2 is available."
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
