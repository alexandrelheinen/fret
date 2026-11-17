#!/usr/bin/env bash
# Idempotent first install of FRET on Ubuntu Server 24.04 arm64 (FR-HW-55).
set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

TAG=""
SKIP_APT=0
DRY_RUN=0
REPO_URL="${FRET_REPO_URL}"

usage() {
  cat <<'EOF'
Usage: rpi5_install.sh --tag <vX.Y.Z> [--repo-url URL] [--skip-apt] [--dry-run]

Install ROS 2 Jazzy ros-base, system deps, a PEP 668 venv with system site
packages, the ARCO aarch64 wheel, and build FRET into /opt/fret.
Idempotent: a second run exits 0 and reports no changes when already current.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tag) TAG="${2:-}"; shift 2 ;;
    --repo-url) REPO_URL="${2:-}"; shift 2 ;;
    --skip-apt) SKIP_APT=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown argument: $1"; usage; exit 1 ;;
  esac
done

if [[ -z "${TAG}" ]]; then
  fail "--tag is required"
  usage
  exit 1
fi

CHANGES=0
mark_change() {
  CHANGES=$((CHANGES + 1))
  info "$*"
}

ensure_user() {
  if id -u "${FRET_SERVICE_USER}" >/dev/null 2>&1; then
    return 0
  fi
  mark_change "creating service user ${FRET_SERVICE_USER}"
  [[ "${DRY_RUN}" -eq 1 ]] && return 0
  run_root useradd --system --create-home --home-dir /var/lib/fret \
    --shell /usr/sbin/nologin --groups dialout,video "${FRET_SERVICE_USER}" || true
  run_root usermod -aG dialout,video "${FRET_SERVICE_USER}" || true
}

install_apt() {
  if [[ "${SKIP_APT}" -eq 1 ]]; then
    info "skipping apt (--skip-apt)"
    return 0
  fi
  mark_change "ensuring apt packages (ROS Jazzy ros-base, zenoh, build deps)"
  [[ "${DRY_RUN}" -eq 1 ]] && return 0
  run_root apt-get update
  run_root apt-get install -y --no-install-recommends \
    curl gnupg lsb-release ca-certificates git \
    python3-pip python3-venv python3-colcon-common-extensions \
    v4l-utils systemd-zram-generator \
    ros-${ROS_DISTRO:-jazzy}-ros-base \
    ros-${ROS_DISTRO:-jazzy}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO:-jazzy}-usb-cam || \
    run_root apt-get install -y --no-install-recommends \
      curl gnupg lsb-release ca-certificates git \
      python3-pip python3-venv python3-colcon-common-extensions \
      v4l-utils systemd-zram-generator
}

clone_or_fetch() {
  local src
  src="$(fret_src_dir)"
  if [[ -d "${src}/.git" ]]; then
    info "fetching tags in ${src}"
    [[ "${DRY_RUN}" -eq 1 ]] && return 0
    git -C "${src}" fetch --tags --force origin
    if [[ "$(git -C "${src}" rev-parse HEAD)" != "$(git -C "${src}" rev-parse "${TAG}^{commit}" 2>/dev/null || true)" ]]; then
      mark_change "checking out ${TAG}"
      git -C "${src}" checkout --detach "${TAG}"
    fi
  else
    mark_change "cloning ${REPO_URL} into ${src}"
    [[ "${DRY_RUN}" -eq 1 ]] && return 0
    run_root mkdir -p "${FRET_ROOT}"
    run_root git clone --branch "${TAG}" --depth 1 "${REPO_URL}" "${src}" \
      || { run_root git clone "${REPO_URL}" "${src}"; git -C "${src}" checkout --detach "${TAG}"; }
  fi
}

setup_venv_and_arco() {
  local venv
  venv="$(fret_venv_dir)"
  if [[ ! -d "${venv}" ]]; then
    mark_change "creating venv with --system-site-packages at ${venv}"
    [[ "${DRY_RUN}" -eq 1 ]] && return 0
    run_root python3 -m venv --system-site-packages "${venv}"
  fi
  [[ "${DRY_RUN}" -eq 1 ]] && return 0
  local wheel
  wheel="$(download_arco_aarch64_wheel "${FRET_ROOT}/cache")"
  "${venv}/bin/pip" install --upgrade pip
  "${venv}/bin/pip" install "${wheel}"
  "${venv}/bin/pip" install -e "$(fret_src_dir)[hw]"
}

build_release() {
  local release_dir
  release_dir="$(fret_releases_dir)/${TAG}"
  if [[ -d "${release_dir}/install" ]]; then
    info "release ${TAG} already built"
  else
    mark_change "building release ${TAG} into ${release_dir}"
    [[ "${DRY_RUN}" -eq 1 ]] && return 0
    run_root mkdir -p "${release_dir}"
    # Prefer colcon when available; otherwise stage a venv-linked tree.
    if command -v colcon >/dev/null 2>&1 && [[ -f /opt/ros/jazzy/setup.bash ]]; then
      set +u
      # shellcheck source=/dev/null
      source /opt/ros/jazzy/setup.bash
      set -u
      (
        cd "$(fret_src_dir)"
        colcon build --install-base "${release_dir}/install" \
          --build-base "${release_dir}/build" \
          --cmake-args -DCMAKE_BUILD_TYPE=Release
      )
    else
      run_root mkdir -p "${release_dir}/install"
      run_root ln -sfn "$(fret_venv_dir)" "${release_dir}/venv"
      printf '%s\n' "${TAG}" >"${release_dir}/VERSION"
    fi
  fi
  local current
  current="$(fret_current_link)"
  if [[ ! -L "${current}" ]] || [[ "$(readlink -f "${current}")" != "$(readlink -f "${release_dir}")" ]]; then
    mark_change "pointing current -> ${TAG}"
    [[ "${DRY_RUN}" -eq 1 ]] && return 0
    run_root ln -sfn "${release_dir}" "${current}"
  fi
}

apply_storage_policy() {
  local repo_deploy
  repo_deploy="$(fret_src_dir)/deploy"
  if [[ ! -d "${repo_deploy}" ]]; then
    repo_deploy="${SCRIPT_DIR}/../../deploy"
  fi
  mark_change "applying microSD storage policy (FR-HW-72)"
  [[ "${DRY_RUN}" -eq 1 ]] && return 0
  run_root install -D -m 0644 "${repo_deploy}/journald/fret.conf" \
    /etc/systemd/journald.conf.d/fret.conf
  run_root install -D -m 0644 "${repo_deploy}/zram/zram-generator.conf" \
    /etc/systemd/zram-generator.conf
  run_root mkdir -p /etc/fret /var/log/fret /var/lib/fret/diag /run/fret-logs
  if ! grep -q 'fret-logs' /etc/fstab 2>/dev/null; then
    render_template "${repo_deploy}/fstab.d/fret-tmpfs.in" /etc/fstab.d/fret-tmpfs
    # Append tmpfs lines if fstab.d is unused on this image.
    if [[ ! -d /etc/fstab.d ]]; then
      cat "${repo_deploy}/fstab.d/fret-tmpfs.in" | run_root tee -a /etc/fstab >/dev/null
    fi
  fi
  # Ensure no swap file on the card.
  if [[ -f /swapfile ]]; then
    warn "found /swapfile; not removing automatically — prefer zram only"
  fi
}

apply_network_and_units() {
  local repo_deploy
  repo_deploy="$(fret_src_dir)/deploy"
  if [[ ! -d "${repo_deploy}" ]]; then
    repo_deploy="${SCRIPT_DIR}/../../deploy"
  fi
  mark_change "installing ros.env, udev template, systemd units (FR-HW-57/63)"
  [[ "${DRY_RUN}" -eq 1 ]] && return 0
  render_template "${repo_deploy}/ros.env.in" /etc/fret/ros.env
  render_template "${repo_deploy}/dds/cyclonedds.xml.in" /etc/fret/cyclonedds.xml
  run_root install -D -m 0644 "${repo_deploy}/udev/99-fret.rules.in" \
    /etc/udev/rules.d/99-fret.rules.in
  run_root install -D -m 0755 "${repo_deploy}/bin/fret-hardware-launch" \
    "${FRET_ROOT}/current/bin/fret-hardware-launch" 2>/dev/null || \
    run_root install -D -m 0755 "${repo_deploy}/bin/fret-hardware-launch" \
      "${FRET_ROOT}/bin/fret-hardware-launch"
  for unit in fret-hardware fret-planner fret-vision fret-zenohd; do
    run_root install -D -m 0644 "${repo_deploy}/systemd/${unit}.service" \
      "/etc/systemd/system/${unit}.service"
  done
  if command -v systemctl >/dev/null 2>&1; then
    run_root systemctl daemon-reload || true
  fi
}

ensure_user
install_apt
clone_or_fetch
setup_venv_and_arco
build_release
apply_storage_policy
apply_network_and_units

if [[ "${CHANGES}" -eq 0 ]]; then
  ok "rpi5_install: no changes (idempotent) for ${TAG}"
else
  ok "rpi5_install: applied ${CHANGES} change(s) for ${TAG}"
fi
exit 0
