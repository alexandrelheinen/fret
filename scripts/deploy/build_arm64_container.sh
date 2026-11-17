#!/usr/bin/env bash
# Build and smoke-test the arm64 Pi reproducibility image (FR-HW-60).
set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
# shellcheck source=/dev/null
source "${REPO_ROOT}/scripts/common.sh"

IMAGE_TAG="${FRET_ARM64_IMAGE:-fret-rpi5:local}"
DOCKERFILE="${REPO_ROOT}/docker/rpi5.Dockerfile"
LOG_DIR="${REPO_ROOT}/.fret-artifacts/arm64-container"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/build-$(date -u +%Y%m%dT%H%M%SZ).log"

require_command docker "docker is required for the arm64 container check"

info "Ensuring arm64 QEMU / binfmt support"
docker run --privileged --rm tonistiigi/binfmt --install arm64 \
  >>"${LOG_FILE}" 2>&1 || warn "binfmt install returned non-zero (may already be set)"

info "Building ${IMAGE_TAG} (linux/arm64); log: ${LOG_FILE}"
START_TS="$(date +%s)"
set +e
docker buildx build \
  --platform linux/arm64 \
  --load \
  -f "${DOCKERFILE}" \
  -t "${IMAGE_TAG}" \
  "${REPO_ROOT}" >>"${LOG_FILE}" 2>&1
BUILD_RC=$?
set -e
END_TS="$(date +%s)"
WALL=$((END_TS - START_TS))
echo "wall_time_s=${WALL}" | tee -a "${LOG_FILE}"

if [[ "${BUILD_RC}" -ne 0 ]]; then
  fail "arm64 container build failed (exit ${BUILD_RC}); see ${LOG_FILE}"
  exit "${BUILD_RC}"
fi

info "Running MuJoCo-free unit shards inside the arm64 image"
set +e
docker run --rm --platform linux/arm64 "${IMAGE_TAG}" bash -lc '
  set -Eeuo pipefail
  # ROS setup.bash references optional unset vars (AMENT_TRACE_SETUP_FILES).
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  uname -m | tee /tmp/arch.txt
  test "$(cat /tmp/arch.txt)" = "aarch64"
  python3 - <<'PY'
import arco, importlib.metadata, platform
assert platform.machine() == "aarch64"
ver = getattr(arco, "__version__", None) or importlib.metadata.version("arco")
print("arco", ver)
import fret  # noqa: F401
print("fret_import_ok")
PY
  # Prefer apt usb_cam evidence for the PR driver choice.
  apt-cache policy ros-jazzy-usb-cam ros-jazzy-v4l2-camera 2>/dev/null \
    | tee /tmp/camera-drivers.txt || true
  python3 -m pytest tests/test_import_graph_no_mujoco.py tests/deploy \
    tests/hardware/test_diag_pi.py tests/hardware/test_diag_cli.py \
    tests/hardware/test_diag_load.py tests/hardware/test_sd_notify_cadence.py \
    tests/integration/test_so101_hw_scenario_mock.py \
    -p no:launch_testing -p no:launch_ros -q
'
SMOKE_RC=$?
set -e
echo "smoke_exit=${SMOKE_RC}" | tee -a "${LOG_FILE}"

if [[ "${SMOKE_RC}" -ne 0 ]]; then
  fail "arm64 container smoke failed (exit ${SMOKE_RC}); see ${LOG_FILE}"
  exit "${SMOKE_RC}"
fi

ok "arm64 container build + smoke passed in ${WALL}s"
echo "log=${LOG_FILE}"
