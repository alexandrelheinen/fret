#!/usr/bin/env bash
# Roll /opt/fret/current back to the previous release (FR-HW-56).
set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/lib.sh"

CURRENT="$(fret_current_link)"
PREVIOUS="${FRET_ROOT}/previous"

python3 - <<PY
from pathlib import Path
from fret.deploy.release_switch import rollback_release

path = rollback_release(Path("${CURRENT}"), Path("${PREVIOUS}"))
print(f"rolled back to {path}")
PY

ok "rollback complete"
