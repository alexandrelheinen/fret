#!/usr/bin/env bash
# V115-03: physics showcase dry-run within release job timeouts.
#
# Runs headless physics-mode showcase renders (PPP 30 min, Dubins 45 min caps)
# using scenario-duration clip subsampling so CI/release dry-runs finish in time.
#
# Usage:
#   bash scripts/tests/physics_showcase_dry_run.sh
#
# Requires: pip install -e ".[sim]", ffmpeg, EGL (MUJOCO_GL=egl).

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

source "${SCRIPT_DIR}/../common.sh"

export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-egl}"

OUTPUT_DIR="${OUTPUT_DIR:-/tmp/fret_physics_showcase_dry_run}"
mkdir -p "${OUTPUT_DIR}"

PPP_TIMEOUT_S="${PPP_TIMEOUT_S:-1800}"
DUBINS_TIMEOUT_S="${DUBINS_TIMEOUT_S:-2700}"

FAILED=0

info "V115-03 physics showcase dry-run (output: ${OUTPUT_DIR})"

info "PPP physics render (timeout ${PPP_TIMEOUT_S}s)"
if timeout "${PPP_TIMEOUT_S}s" ./scripts/video.sh \
    --model ppp \
    --scenario ppp_warehouse \
    --all-cameras \
    --collision-backend mujoco \
    --planner-algorithm rrt_star \
    --full-duration \
    --physics-mode \
    --output-dir "${OUTPUT_DIR}/ppp" \
    --timing-json "${OUTPUT_DIR}/ppp/timing.json" \
    --fps 30 \
    --width 640 \
    --height 360; then
    ok "ppp_physics_showcase: PASSED"
else
    code=$?
    fail "ppp_physics_showcase: FAILED (exit ${code})"
    FAILED=1
fi

info "Dubins physics render (timeout ${DUBINS_TIMEOUT_S}s)"
if timeout "${DUBINS_TIMEOUT_S}s" ./scripts/video.sh \
    --model dubins \
    --scenario dubins_race \
    --all-cameras \
    --collision-backend mujoco \
    --planner-algorithm sst \
    --full-duration \
    --physics-mode \
    --output-dir "${OUTPUT_DIR}/dubins" \
    --timing-json "${OUTPUT_DIR}/dubins/timing.json" \
    --fps 30 \
    --width 640 \
    --height 360; then
    ok "dubins_physics_showcase: PASSED"
else
    code=$?
    fail "dubins_physics_showcase: FAILED (exit ${code})"
    FAILED=1
fi

if [[ "${FAILED}" -eq 0 ]]; then
    ok "Physics showcase dry-run PASSED"
    exit 0
fi

fail "Physics showcase dry-run FAILED"
exit 1
