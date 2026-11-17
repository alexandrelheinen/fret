#!/usr/bin/env bash
# Shared helpers for Raspberry Pi 5 deploy scripts (FR-HW-55..FR-HW-74).
# shellcheck shell=bash

set -Eeuo pipefail
IFS=$'\n\t'

: "${FRET_ROOT:=/opt/fret}"
: "${FRET_REPO_URL:=https://github.com/alexandrelheinen/fret.git}"
: "${FRET_SERVICE_USER:=fret}"
: "${ARCO_AARCH64_WHEEL_URL:=https://github.com/alexandrelheinen/arco/releases/download/v0.5.0/arco-0.5.0-cp310-abi3-manylinux_2_17_aarch64.manylinux2014_aarch64.whl}"
: "${ARCO_AARCH64_WHEEL_SHA256:=d279384fb15e3ea455cc3aa19a9fc5214accbc0a9a57e7e9f847a358640e8a51}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Prefer repo-relative common.sh when running from a checkout.
if [[ -f "${SCRIPT_DIR}/../common.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/../common.sh"
elif [[ -f "${FRET_ROOT}/src/scripts/common.sh" ]]; then
  # shellcheck source=/dev/null
  source "${FRET_ROOT}/src/scripts/common.sh"
else
  info() { echo "[INFO] $*"; }
  ok() { echo "[ OK ] $*"; }
  warn() { echo "[WARN] $*"; }
  fail() { echo "[FAIL] $*"; }
fi

fret_releases_dir() { echo "${FRET_ROOT}/releases"; }
fret_current_link() { echo "${FRET_ROOT}/current"; }
fret_src_dir() { echo "${FRET_ROOT}/src"; }
fret_venv_dir() { echo "${FRET_ROOT}/venv"; }

require_root_or_sudo() {
  if [[ "${EUID}" -eq 0 ]]; then
    return 0
  fi
  if command -v sudo >/dev/null 2>&1; then
    return 0
  fi
  fail "root or sudo is required for Pi deploy scripts"
  exit 1
}

run_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

verify_arco_wheel_sha() {
  local wheel_path="$1"
  local expected="${ARCO_AARCH64_WHEEL_SHA256}"
  local actual
  actual="$(sha256sum "${wheel_path}" | awk '{print $1}')"
  if [[ "${actual}" != "${expected}" ]]; then
    fail "ARCO aarch64 wheel SHA-256 mismatch"
    fail "expected ${expected}"
    fail "actual   ${actual}"
    exit 1
  fi
}

download_arco_aarch64_wheel() {
  local dest_dir="$1"
  local wheel_name
  wheel_name="$(basename "${ARCO_AARCH64_WHEEL_URL}")"
  local dest="${dest_dir}/${wheel_name}"
  mkdir -p "${dest_dir}"
  if [[ -f "${dest}" ]]; then
    if verify_arco_wheel_sha "${dest}"; then
      echo "${dest}"
      return 0
    fi
    rm -f "${dest}"
  fi
  info "Downloading ARCO aarch64 wheel"
  if ! curl -fsSL -o "${dest}" "${ARCO_AARCH64_WHEEL_URL}"; then
    fail "Failed to download ARCO aarch64 wheel from ${ARCO_AARCH64_WHEEL_URL}"
    fail "Refusing to fall back to a source build on the Pi"
    exit 1
  fi
  verify_arco_wheel_sha "${dest}"
  echo "${dest}"
}

render_template() {
  local src="$1"
  local dest="$2"
  local tmp
  tmp="$(mktemp)"
  sed \
    -e "s|@FRET_ROOT@|${FRET_ROOT}|g" \
    -e "s|@FRET_SERVICE_USER@|${FRET_SERVICE_USER}|g" \
    -e "s|@ROS_DOMAIN_ID@|${ROS_DOMAIN_ID:-42}|g" \
    -e "s|@PI_HOSTNAME@|${PI_HOSTNAME:-fret-pi}|g" \
    "${src}" >"${tmp}"
  run_root install -D -m 0644 "${tmp}" "${dest}"
  rm -f "${tmp}"
}
