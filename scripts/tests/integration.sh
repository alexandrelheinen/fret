#!/usr/bin/env bash
# scripts/run_integration_tests.sh
#
# Runs ROS 2 launch_testing integration tests from tests/integration/.
#
# Requires:
#   - ROS 2 Jazzy installed (/opt/ros/jazzy)
#   - Workspace built:    ./scripts/build.sh
#   - xvfb installed:     sudo apt install xvfb
#   - pytest installed:   pip install pytest
#
# Usage:
#   bash scripts/run_integration_tests.sh
#
# Exit code: 0 = all pass, 1 = any failure.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="$(dirname "${SCRIPT_DIR}")"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Integration tests"' ERR

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

require_command xvfb-run "xvfb-run is required. Run: sudo apt install xvfb"
require_command pytest "pytest is required. Run: pip install pytest"

if xvfb-run -a pytest tests/integration/ -v; then
    ok "Integration tests: PASSED"
    exit 0
else
    fail "Integration tests: FAILED"
    exit 1
fi
