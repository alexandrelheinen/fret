#!/usr/bin/env bash
# scripts/run_smoke_tests.sh
#
# Runs ROS 2 launch smoke tests for all standard launch files.
# Each launch is allowed up to 20 s; exit code 124 (timeout) is accepted —
# it means the launch started cleanly and ran until killed.
#
# Requires:
#   - ROS 2 Jazzy installed (/opt/ros/jazzy)
#   - Workspace built:    ./scripts/build.sh
#   - xvfb installed:     sudo apt install xvfb
#
# Usage:
#   bash scripts/run_smoke_tests.sh
#
# Exit code: 0 = all pass, 1 = any failure.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="$(dirname "${SCRIPT_DIR}")"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/common.sh"

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
require_command ros2 "ros2 not found. Source /opt/ros/jazzy/setup.bash first."

# ---------------------------------------------------------------------------
# Helper: run a single launch and accept timeout (124) as success.
# ---------------------------------------------------------------------------
run_smoke_test() {
    local test_name="$1"
    shift
    local timeout_s=20
    if [[ "${1:-}" =~ ^[0-9]+$ ]]; then
        timeout_s="$1"
        shift
    fi
    if [[ "${1:-}" == "--" ]]; then
        shift
    fi
    local log_file="/tmp/smoke_${test_name}.log"

    info "Smoke test: ${test_name} (${timeout_s}s)"
    set +e
    timeout "${timeout_s}s" "$@" >"${log_file}" 2>&1
    local exit_code=$?
    set -e

    if [[ "${exit_code}" -ne 0 && "${exit_code}" -ne 124 ]]; then
        fail "${test_name}: FAILED (exit ${exit_code})"
        cat "${log_file}"
        return 1
    fi

    ok "${test_name}: PASSED (exit ${exit_code})"
    tail -n 10 "${log_file}" || true
    return 0
}

FAILED=0

echo "=== ROS 2 launch smoke tests ==="

# MuJoCo viewer dry-run (no ROS, no display required).
run_smoke_test "mujoco_view_dry_run" -- python3 scripts/view_mujoco.py --dry-run \
    || FAILED=1

# Release workflow CLI must parse before tag builds render showcase MP4s.
info "Showcase release CLI smoke"
if python3 -m pytest tests/simulation/test_render_mujoco.py \
    -k "release_workflow_cli_args_parse or main_accepts_release" -q; then
    ok "showcase_release_cli: PASSED"
else
    fail "showcase_release_cli: FAILED"
    FAILED=1
fi

# v1.0 V10-1: PPP warehouse MuJoCo SITL (no Gazebo dependency).
run_smoke_test "sitl_ppp_warehouse_mujoco" 30 -- \
    env MUJOCO_GL=egl PYOPENGL_PLATFORM=egl \
    xvfb-run -a ros2 launch fret sitl.py \
    scenario:=ppp_warehouse model:=ppp backend:=mujoco \
    || FAILED=1

# Gazebo-dependent launches — skip gracefully when ros_gz_sim is not installed.
if ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
    run_smoke_test "sim_launch" -- xvfb-run -a ros2 launch fret sim.py model:=scara \
        || FAILED=1
    run_smoke_test "arco_scenario_launch" -- xvfb-run -a ros2 launch fret arco_scenario.py \
        || FAILED=1
    run_smoke_test "sitl_launch" -- xvfb-run -a ros2 launch fret sitl.py \
        || FAILED=1
else
    info "ros_gz_sim not found — skipping Gazebo-dependent smoke tests (sim_launch, arco_scenario_launch, sitl_launch)."
fi

echo "======================================"
if [[ "${FAILED}" -eq 0 ]]; then
    ok "All smoke tests PASSED"
    exit 0
else
    fail "Smoke tests FAILED"
    exit 1
fi
