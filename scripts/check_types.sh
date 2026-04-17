#!/usr/bin/env bash
# scripts/check_types.sh
#
# Runs mypy strict type checking on src/.
# Exit code: 0 = pass, 1 = any type error found.
#
# Usage: bash scripts/check_types.sh
# Run this locally before pushing any commit.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Type check"' ERR

require_command python3 "python3 is required."

echo "=== Type check (mypy) ==="
if python3 -m mypy src/ \
    --ignore-missing-imports \
    --strict \
    --exclude 'src/fret\.egg-info' \
    --exclude 'src/fret/mesh'; then
    ok "mypy: PASSED"
    exit 0
else
    fail "mypy: FAILED"
    exit 1
fi
