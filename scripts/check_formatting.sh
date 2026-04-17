#!/usr/bin/env bash
# scripts/check_formatting.sh
#
# Validates Python formatting (black + isort) and C++ formatting (clang-format).
# All checks must pass; exit code is 0 on success, 1 on any failure.
#
# Usage: bash scripts/check_formatting.sh
# Run this locally before pushing any commit.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Formatting check"' ERR

FAILED=0

echo "=== Formatting check (black + isort + clang-format) ==="

# ---------- black ----------
echo "--- black ---"
require_command python3 "python3 is required."
if python3 -m black --check --target-version py312 --line-length 79 src/; then
    ok "black: PASSED"
else
    fail "black: FAILED  (fix: python3 -m black --target-version py312 --line-length 79 src/)"
    FAILED=1
fi

# ---------- isort ----------
echo "--- isort ---"
if python3 -m isort --check-only --line-length 79 src/; then
    ok "isort: PASSED"
else
    fail "isort: FAILED  (fix: python3 -m isort --line-length 79 src/)"
    FAILED=1
fi

# ---------- clang-format ----------
echo "--- clang-format ---"
CPP_FILES=$(find src -type f \( -name '*.cpp' -o -name '*.hpp' \) 2>/dev/null || true)
if [[ -z "${CPP_FILES}" ]]; then
    info "No C++ files found — clang-format check skipped."
else
    require_command clang-format \
        "clang-format is required. Run: sudo apt install clang-format"
    if echo "${CPP_FILES}" | xargs clang-format --dry-run --Werror; then
        ok "clang-format: PASSED"
    else
        fail "clang-format: FAILED"
        FAILED=1
    fi
fi

echo "======================================"
if [[ "${FAILED}" -eq 0 ]]; then
    ok "All formatting checks PASSED"
    exit 0
else
    fail "Formatting check FAILED"
    exit 1
fi
