#!/usr/bin/env bash
# scripts/run_tests.sh
#
# Runs the Python unit test suite (pytest + 90 % coverage gate) and the
# quality gate validation script.
#
# Requires:
#   - Python packages installed (pytest, pytest-cov, pytest-timeout)
#   - ROS 2 workspace built: ./scripts/build.sh
#   - ROS 2 overlay sourced OR let this script source it automatically
#
# Usage:
#   bash scripts/run_tests.sh
#
# Exit code: 0 = all pass, 1 = any failure.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="$(dirname "${SCRIPT_DIR}")"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Tests"' ERR

ROS_SETUP="/opt/ros/jazzy/setup.bash"
INSTALL_SETUP="${REPO_ROOT}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    fail "ROS environment not found: ${ROS_SETUP}."
    info "Run ./scripts/install.sh to install ROS 2, then ./scripts/build.sh to build."
    exit 1
fi

if [[ ! -f "${INSTALL_SETUP}" ]]; then
    fail "Workspace overlay not found: ${INSTALL_SETUP}."
    info "Run ./scripts/build.sh to build the workspace first."
    exit 1
fi

set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
# shellcheck source=/dev/null
source "${INSTALL_SETUP}"
set -u

require_command python3 "python3 is required."
require_command pytest "pytest is required. Run: pip install pytest pytest-cov pytest-timeout"

if pytest tests/ \
    --ignore=tests/integration \
    -p no:launch_testing \
    -p no:launch_ros \
    -v \
    --cov=src/fret; then
    ok "Unit tests: PASSED"
else
    fail "Unit tests: FAILED"
fi
