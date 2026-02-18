#!/usr/bin/env bash

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Setup"' ERR

usage() {
	cat <<'EOF'
Usage: ./scripts/setup.sh [--yes] [--install]

Options:
	-y, --yes       Run non-interactively (auto-confirm).
	-i, --install   Run dependency installation before workspace setup.
	-h, --help      Show this help message.
EOF
}

ASSUME_YES=0
RUN_INSTALL=0
for arg in "$@"; do
	case "$arg" in
		-y|--yes) ASSUME_YES=1 ;;
		-i|--install) RUN_INSTALL=1 ;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			fail "Unknown option: ${arg}"
			usage
			exit 1
			;;
	esac
done

require_not_root
require_command git "git is required but not installed. Install git manually, then rerun this script."

if [[ "${RUN_INSTALL}" -eq 1 ]]; then
	info "Running dependency installer..."
	if [[ "${ASSUME_YES}" -eq 1 ]]; then
		"${SCRIPT_DIR}/install.sh" --yes
	else
		"${SCRIPT_DIR}/install.sh"
	fi
	ok "Dependency installation stage completed."
else
	info "Skipping dependency installation stage by default. Use --install to run it."
fi

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
ok "ROS 2 Jazzy sourced for this setup session."

SRC_DIR="${PROJECT_ROOT}/src"
INSTALL_DIR="${PROJECT_ROOT}/install"

info "Ensuring source directory exists at ${SRC_DIR}..."
mkdir -p "${SRC_DIR}"
ok "Source directory is ready."

PACKAGE_COUNT=$(find "${SRC_DIR}" -type f -name package.xml | wc -l | tr -d '[:space:]')

if [[ "${PACKAGE_COUNT}" -gt 0 ]]; then
	info "Detected ${PACKAGE_COUNT} ROS package(s) in src/."

	info "Installing package dependencies with rosdep..."
	rosdep install --from-paths "${SRC_DIR}" --ignore-src -r -y
	ok "Workspace dependencies resolved."
	info "Setup stage complete. Use ./scripts/build.sh to build packages."
else
	warn "No ROS packages found in src/. Skipping rosdep stage."
fi

echo
ok "Setup completed successfully."
info "Workspace root: ${PROJECT_ROOT}"
if [[ -f "${INSTALL_DIR}/setup.bash" ]]; then
	info "Activate overlay with: source install/setup.bash"
else
	info "Add ROS packages to src/, run setup.sh, then run ./scripts/build.sh to generate overlay."
fi

