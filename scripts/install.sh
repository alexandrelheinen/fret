#!/usr/bin/env bash

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Installation"' ERR

usage() {
  cat <<'EOF'
Usage: ./scripts/install.sh [--yes]

Options:
  -y, --yes   Run non-interactively (auto-confirm).
  -h, --help  Show this help message.
EOF
}

ASSUME_YES=0
for arg in "$@"; do
  case "$arg" in
    -y|--yes) ASSUME_YES=1 ;;
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

if [[ ! -f /etc/os-release ]]; then
  fail "Cannot detect OS: /etc/os-release not found."
  exit 1
fi

source /etc/os-release
OS_ID="${ID:-unknown}"
DETECTED_OS_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}"
TARGET_UBUNTU_CODENAME='noble'
ROS_DISTRO='jazzy'

require_command apt "apt is required for installation but was not found on this system."

info "Detected OS: ${PRETTY_NAME}"
if [[ "${OS_ID}" == "ubuntu" && "${DETECTED_OS_CODENAME}" == "${TARGET_UBUNTU_CODENAME}" ]]; then
  ok "Validated platform: Ubuntu 24.04 (${TARGET_UBUNTU_CODENAME})."
else
  warn "This project is validated only on Ubuntu 24.04 (${TARGET_UBUNTU_CODENAME})."
  warn "Detected: ${OS_ID}/${DETECTED_OS_CODENAME}. Attempting installation with Ubuntu 24.04 + ROS 2 Jazzy packages anyway."
fi
info "Selected ROS 2 distribution: ${ROS_DISTRO}"
echo

if [[ "${ASSUME_YES}" -ne 1 ]]; then
  read -r -p "Proceed with installation? [y/N] " answer
  if [[ ! "${answer}" =~ ^[Yy]$ ]]; then
    warn "Installation aborted by user."
    exit 0
  fi
fi

info "Requesting sudo access..."
sudo -v
ok "Sudo access granted."

info "Installing base dependencies..."
sudo apt update
sudo apt install -y \
  software-properties-common \
  curl \
  gnupg2 \
  lsb-release \
  build-essential \
  cmake \
  black \
  isort
ok "Base dependencies installed."

info "Enabling Ubuntu universe repository..."
sudo add-apt-repository -y universe
ok "Universe repository enabled."

info "Configuring ROS 2 APT repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${TARGET_UBUNTU_CODENAME} main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
ok "ROS 2 repository configured."

info "Configuring Gazebo APT repository..."
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg | sudo gpg --dearmor --yes -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${TARGET_UBUNTU_CODENAME} main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
ok "Gazebo repository configured."

info "Installing ROS 2 core and development tools..."
sudo apt update
sudo apt install -y \
  "ros-${ROS_DISTRO}-desktop" \
  ros-dev-tools \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-vcstool
ok "ROS 2 core and development tools installed."

info "Installing simulation stack..."
if apt-cache show gz-harmonic >/dev/null 2>&1; then
  GZ_BASE_PACKAGE="gz-harmonic"
elif apt-cache show gz-sim >/dev/null 2>&1; then
  warn "Package gz-harmonic is unavailable; falling back to gz-sim."
  GZ_BASE_PACKAGE="gz-sim"
else
  fail "Could not find Gazebo Harmonic packages (gz-harmonic/gz-sim) in configured apt sources."
  exit 1
fi

sudo apt install -y \
  "${GZ_BASE_PACKAGE}" \
  "ros-${ROS_DISTRO}-ros-gz" \
  "ros-${ROS_DISTRO}-ros-gz-sim" \
  "ros-${ROS_DISTRO}-xacro" \
  "ros-${ROS_DISTRO}-joint-state-publisher-gui" \
  "ros-${ROS_DISTRO}-robot-state-publisher" \
  "ros-${ROS_DISTRO}-rviz2"
ok "Simulation stack installed."

info "Initializing rosdep..."
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
else
  warn "rosdep already initialized."
fi
rosdep update
ok "rosdep is ready."

echo
ok "Dependency installation completed successfully."
info "This script installs dependencies only (no workspace creation/build)."
info "For a session-based ROS environment: source /opt/ros/${ROS_DISTRO}/setup.bash"
info "Recommended project ROS source path: ./src"
info "Recommended build output paths: ./build ./install ./log"
info "Verify with: ros2 --version"
info "Optional checks: gz sim, rviz2"
