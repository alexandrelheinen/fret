#!/usr/bin/env bash
# Combine parallel unit-test coverage shards and enforce the 90 % gate.
#
# Usage:
#   bash scripts/tests/unit_coverage.sh

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="$(dirname "${SCRIPT_DIR}")"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Unit coverage"' ERR

shopt -s nullglob
coverage_files=(.coverage.*)
shopt -u nullglob

if [[ ${#coverage_files[@]} -eq 0 ]]; then
    fail "No shard coverage files found (.coverage.*)"
    exit 1
fi

require_command coverage "coverage is required. Run: pip install coverage"

info "Combining coverage from ${#coverage_files[@]} shard file(s)"
coverage combine "${coverage_files[@]}"

if coverage report --fail-under=90; then
    ok "Coverage gate: PASSED (>= 90%)"
else
    fail "Coverage gate: FAILED (< 90%)"
fi
