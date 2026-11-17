#!/usr/bin/env bash
# Write /etc/udev/rules.d/99-fret.rules from live udevadm info (FR-HW-58).
set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

ROLE=""
DEVICE=""
OUT_RULES="${UDEV_RULES_OUT:-/etc/udev/rules.d/99-fret.rules}"
TEMPLATE="${SCRIPT_DIR}/../../deploy/udev/99-fret.rules.in"

usage() {
  cat <<'EOF'
Usage: udev_capture.sh --follower|/dev/ttyUSB0
       udev_capture.sh --camera|/dev/video0
       udev_capture.sh --leader|/dev/ttyUSB1   # reserved, optional
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --follower) ROLE=follower; DEVICE="${2:-}"; shift 2 ;;
    --camera) ROLE=camera; DEVICE="${2:-}"; shift 2 ;;
    --leader) ROLE=leader; DEVICE="${2:-}"; shift 2 ;;
    --output) OUT_RULES="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown argument: $1"; exit 1 ;;
  esac
done

if [[ -z "${ROLE}" || -z "${DEVICE}" ]]; then
  usage
  exit 1
fi

require_command udevadm "udevadm is required"

props="$(udevadm info -q property -n "${DEVICE}")"
vendor="$(printf '%s\n' "${props}" | awk -F= '/ID_VENDOR_ID=/{print tolower($2)}')"
product="$(printf '%s\n' "${props}" | awk -F= '/ID_MODEL_ID=/{print tolower($2)}')"
serial="$(printf '%s\n' "${props}" | awk -F= '/ID_SERIAL_SHORT=/{print $2}')"
path_id="$(printf '%s\n' "${props}" | awk -F= '/ID_PATH=/{print $2}')"

run_root mkdir -p "$(dirname "${OUT_RULES}")"
if [[ ! -f "${OUT_RULES}" ]]; then
  run_root cp "${TEMPLATE}" "${OUT_RULES}"
fi

case "${ROLE}" in
  follower)
    if [[ -n "${serial}" ]]; then
      run_root sed -i "s/FOLLOWER_SERIAL/${serial}/g" "${OUT_RULES}"
    else
      warn "no serial; falling back to ID_PATH=${path_id}"
      run_root sed -i "s/ATTRS{serial}==\"FOLLOWER_SERIAL\"/ENV{ID_PATH}==\"${path_id}\"/" "${OUT_RULES}"
    fi
    ;;
  leader)
    run_root sed -i 's/^# SUBSYSTEM=="tty".*LEADER/SUBSYSTEM=="tty"/' "${OUT_RULES}" || true
    if [[ -n "${serial}" ]]; then
      run_root sed -i "s/LEADER_SERIAL/${serial}/g" "${OUT_RULES}"
    fi
    ;;
  camera)
    run_root sed -i "s/CAMERA_VENDOR/${vendor}/g; s/CAMERA_PRODUCT/${product}/g" "${OUT_RULES}"
    ;;
esac

ok "updated ${OUT_RULES} for ${ROLE} (${DEVICE})"
