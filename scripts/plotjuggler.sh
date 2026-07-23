#!/usr/bin/env bash
# Open a FRET telemetry CSV in PlotJuggler with the matching scenario layout.
#
# Usage:
#   bash scripts/plotjuggler.sh --csv /tmp/fret_telemetry/.../telemetry.csv \
#       --scenario dubins_race
#   bash scripts/plotjuggler.sh --csv path.csv --layout src/fret/telemetry/layouts/omx_arm.xml
#   bash scripts/plotjuggler.sh --list
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAYOUT_DIR="${ROOT}/src/fret/telemetry/layouts"
INDEX="${LAYOUT_DIR}/index.yaml"

CSV=""
SCENARIO=""
LAYOUT=""
LIST=0

usage() {
  sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --csv) CSV="${2:-}"; shift 2 ;;
    --scenario) SCENARIO="${2:-}"; shift 2 ;;
    --layout) LAYOUT="${2:-}"; shift 2 ;;
    --list) LIST=1; shift ;;
    -h|--help) usage 0 ;;
    *) echo "unknown arg: $1" >&2; usage 1 ;;
  esac
done

if [[ "${LIST}" -eq 1 ]]; then
  echo "scenario_id → layout (from ${INDEX})"
  python3 - "${INDEX}" <<'PY'
import sys
from pathlib import Path
import yaml
index = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
for k in sorted(index):
    print(f"  {k:20s}  {index[k]}")
PY
  exit 0
fi

if [[ -z "${CSV}" ]]; then
  echo "error: --csv is required" >&2
  usage 1
fi
if [[ ! -f "${CSV}" ]]; then
  echo "error: CSV not found: ${CSV}" >&2
  exit 1
fi

if [[ -z "${LAYOUT}" ]]; then
  if [[ -z "${SCENARIO}" ]]; then
    echo "error: pass --scenario <id> or --layout <file.xml>" >&2
    usage 1
  fi
  LAYOUT="$(
    python3 - "${SCENARIO}" "${ROOT}" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[2]) / "src"))
from fret.telemetry.layout_paths import layout_path_for_scenario
print(layout_path_for_scenario(sys.argv[1]))
PY
  )"
fi

if [[ ! -f "${LAYOUT}" ]]; then
  echo "error: layout not found: ${LAYOUT}" >&2
  exit 1
fi

if ! command -v plotjuggler >/dev/null 2>&1; then
  echo "error: plotjuggler not on PATH" >&2
  echo "install: https://github.com/facontidavide/PlotJuggler" >&2
  echo "manual: plotjuggler -d ${CSV} -l ${LAYOUT}" >&2
  exit 127
fi

echo "PlotJuggler: csv=${CSV}"
echo "PlotJuggler: layout=${LAYOUT}"
echo "Hint: choose time axis column 't' if prompted."
exec plotjuggler -d "${CSV}" -l "${LAYOUT}"
