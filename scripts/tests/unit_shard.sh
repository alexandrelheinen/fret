#!/usr/bin/env bash
# Run one parallel CI shard of the Python unit test suite.
#
# Usage:
#   bash scripts/tests/unit_shard.sh <shard>
#
# Shards:
#   control    — tests/control, tests/hardware
#   planning   — tests/planning
#   scene      — tests/scene and top-level tests/test_*.py
#   simulation — tests/simulation, tests/scenario, tests/ros
#
# Each shard writes a distinct coverage data file for later combination in CI.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="$(dirname "${SCRIPT_DIR}")"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Unit shard tests"' ERR

SHARD="${1:-}"
if [[ -z "${SHARD}" ]]; then
    fail "Usage: bash scripts/tests/unit_shard.sh <shard>"
    fail "Shards: control | planning | scene | simulation"
    exit 1
fi

case "${SHARD}" in
    control)
        PATHS=(
            tests/control
            tests/hardware
        )
        ;;
    planning)
        PATHS=(tests/planning)
        ;;
    scene)
        PATHS=(
            tests/scene
            tests/test_cli.py
            tests/test_config_loader.py
            tests/test_interfaces.py
            tests/test_metrics.py
            tests/test_quality_gates.py
            tests/test_sitl_config.py
        )
        ;;
    simulation)
        PATHS=(
            tests/simulation
            tests/scenario
            tests/ros
        )
        ;;
    *)
        fail "Unknown shard: ${SHARD}"
        exit 1
        ;;
esac

ROS_SETUP="/opt/ros/jazzy/setup.bash"
INSTALL_SETUP="${REPO_ROOT}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    fail "ROS environment not found: ${ROS_SETUP}."
    exit 1
fi

if [[ ! -f "${INSTALL_SETUP}" ]]; then
    fail "Workspace overlay not found: ${INSTALL_SETUP}."
    exit 1
fi

set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
# shellcheck source=/dev/null
source "${INSTALL_SETUP}"
set -u

require_command python3 "python3 is required."
require_command pytest "pytest is required."

export COVERAGE_FILE="${REPO_ROOT}/.coverage.${SHARD}"

info "Unit shard: ${SHARD}"
if pytest "${PATHS[@]}" \
    -p no:launch_testing \
    -p no:launch_ros \
    -v \
    --cov=src/fret \
    --cov-append; then
    ok "Unit shard ${SHARD}: PASSED"
else
    fail "Unit shard ${SHARD}: FAILED"
    exit 1
fi
