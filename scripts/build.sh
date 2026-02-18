#!/usr/bin/env bash

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Build"' ERR

usage() {
  cat <<'EOF'
Usage: ./scripts/build.sh

Builds ROS packages from ./src into ./build and ./install.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

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

info "Installing package dependencies with rosdep..."
rosdep install --from-paths "${SRC_DIR}" --ignore-src -r -y

info "Building packages from src/..."
colcon --log-base "${LOG_DIR}" build \
  --base-paths "${SRC_DIR}" \
  --build-base "${BUILD_DIR}" \
  --install-base "${INSTALL_DIR}"

ok "Build completed successfully."
info "Activate overlay with: source install/setup.bash"
