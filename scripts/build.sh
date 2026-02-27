#!/usr/bin/env bash

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Build"' ERR

INSTALL_DEPS=false

usage() {
  cat <<'EOF'
Usage: ./scripts/build.sh [--deps]

Builds ROS packages from ./src into ./build and ./install.

Options:
  --deps   Run rosdep install before building (slow: checks apt on every run).
           Only needed after adding new package dependencies.
EOF
}

while [[ $# -gt 0 ]]; do
  case "${1}" in
    -h|--help) usage; exit 0 ;;
    --deps) INSTALL_DEPS=true; shift ;;
    *) fail "Unknown argument: ${1}"; exit 1 ;;
  esac
done

require_not_root
require_command colcon "colcon is required but not installed. Run ./scripts/install.sh first."

ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  fail "ROS environment file not found: ${ROS_SETUP}. Run ./scripts/install.sh first."
  exit 1
fi

info "Sourcing ROS 2 Jazzy environment..."
set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
set -u

if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  info "Sourcing local overlay from install/setup.bash..."
  set +u
  # shellcheck source=/dev/null
  source "${PROJECT_ROOT}/install/setup.bash"
  set -u
fi

SRC_DIR="${PROJECT_ROOT}/src"
BUILD_DIR="${PROJECT_ROOT}/build"
INSTALL_DIR="${PROJECT_ROOT}/install"
LOG_DIR="${PROJECT_ROOT}/log"

PACKAGE_COUNT=$(find "${SRC_DIR}" -type f -name package.xml | wc -l | tr -d '[:space:]')
if [[ "${PACKAGE_COUNT}" -eq 0 ]]; then
  fail "No ROS packages found in src/."
  exit 1
fi

if [[ "${INSTALL_DEPS}" == true ]]; then
  info "Installing package dependencies with rosdep..."
  rosdep install --from-paths "${SRC_DIR}" --ignore-src -r -y
fi

JOBS=$(nproc)
info "Building packages from src/ (${JOBS} parallel jobs)..."

CCACHE_ARGS=()
if command -v ccache &>/dev/null; then
  CCACHE_ARGS=(
    -DCMAKE_C_COMPILER_LAUNCHER=ccache
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
  )
else
  warn "ccache not found — builds will be slower. Run: sudo apt install ccache"
fi

MAKEFLAGS="-j${JOBS}" colcon --log-base "${LOG_DIR}" build \
  --base-paths "${SRC_DIR}" \
  --build-base "${BUILD_DIR}" \
  --install-base "${INSTALL_DIR}" \
  --symlink-install \
  --parallel-workers "${JOBS}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    ${CCACHE_ARGS[@]+"${CCACHE_ARGS[@]}"}

ok "Build completed successfully."
info "Activate overlay with: source install/setup.bash"
