#!/usr/bin/env bash
# FR-HW-55: two consecutive dry-ish install runs exit 0.
# Full apt/colcon path is exercised in the arm64 container; this PC-side
# check covers argument parsing and idempotent reporting with --dry-run.
set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL="${SCRIPT_DIR}/../rpi5_install.sh"

bash "${INSTALL}" --help >/dev/null
bash "${INSTALL}" --tag v0.0.0-test --dry-run --skip-apt
bash "${INSTALL}" --tag v0.0.0-test --dry-run --skip-apt
echo "idempotent_dry_run_ok"
