#!/usr/bin/env bash
# Build a tagged release and switch current only after smoke (FR-HW-56).
set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

TAG="${1:-}"
if [[ -z "${TAG}" ]]; then
  fail "Usage: rpi5_update.sh <vX.Y.Z>"
  exit 1
fi

SRC="$(fret_src_dir)"
RELEASES="$(fret_releases_dir)"
CURRENT="$(fret_current_link)"
PREVIOUS="${FRET_ROOT}/previous"
LOG_DIR="${FRET_ROOT}/log"
mkdir -p "${LOG_DIR}"
LOG="${LOG_DIR}/update-${TAG}-$(date -u +%Y%m%dT%H%M%SZ).log"

info "Updating to ${TAG} (log: ${LOG})"
{
  git -C "${SRC}" fetch --tags --force origin
  git -C "${SRC}" checkout --detach "${TAG}"
  RELEASE="${RELEASES}/${TAG}"
  rm -rf "${RELEASE}"
  mkdir -p "${RELEASE}"
  if command -v colcon >/dev/null 2>&1 && [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    set -u
    (
      cd "${SRC}"
      colcon build --install-base "${RELEASE}/install" \
        --build-base "${RELEASE}/build"
    )
  else
    mkdir -p "${RELEASE}/install"
    printf '%s\n' "${TAG}" >"${RELEASE}/VERSION"
  fi
  # Smoke: import fret from the new tree.
  PYTHONPATH="${SRC}/src:${PYTHONPATH:-}" python3 -c "import fret; print('smoke_ok', fret.__version__ if hasattr(fret,'__version__') else 'ok')"
} >>"${LOG}" 2>&1 || {
  fail "update build/smoke failed; current untouched. log: ${LOG}"
  exit 1
}

python3 - <<PY
from pathlib import Path
from fret.deploy.release_switch import activate_release

activate_release(
    Path("${RELEASES}"),
    Path("${CURRENT}"),
    "${TAG}",
    previous_link=Path("${PREVIOUS}"),
)
print("activated ${TAG}")
PY

ok "updated current -> ${TAG}"
