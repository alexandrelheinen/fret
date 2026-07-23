#!/usr/bin/env bash
# Unit-style checks for scripts/download_showcase.sh (mock aws, no R2 credentials).
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MOCK_BIN="$(mktemp -d)"

cleanup() {
  rm -rf "${MOCK_BIN}"
}
trap cleanup EXIT

cat >"${MOCK_BIN}/aws" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

if [[ "$1" == "s3" && "$2" == "ls" ]]; then
  # Support both recursive and non-recursive listings.
  target=""
  for arg in "$@"; do
    if [[ "${arg}" == s3://* ]]; then
      target="${arg}"
    fi
  done
  prefix="${target#s3://fret-renders/}"
  prefix="${prefix%/}/"
  case "${prefix}" in
    latest/)
      echo "2026-01-01 00:00:00    1200000 latest/dubins_race/dubins_race_overview.mp4"
      echo "2026-01-01 00:00:00    1050000 latest/dubins_race/dubins_race_follow.mp4"
      echo "2026-01-01 00:00:00      50000 latest/dubins_race/dubins_race_overview.csv"
      echo "2026-01-01 00:00:00       2000 latest/dubins_race/dubins_race_overview.json"
      echo "2026-01-01 00:00:00     900000 latest/dubins_race/dubins_race.mp4"
      ;;
    releases/v1.1.0/)
      # Legacy flat layout fallback
      echo "2026-01-01 00:00:00    1200000 releases/v1.1.0/dubins_race_overview.mp4"
      echo "2026-01-01 00:00:00    1050000 releases/v1.1.0/dubins_race_follow.mp4"
      ;;
    *)
      echo "An error occurred (NoSuchKey)" >&2
      exit 1
      ;;
  esac
  exit 0
fi

if [[ "$1" == "s3" && "$2" == "cp" ]]; then
  dest=""
  skip_next=0
  for arg in "$@"; do
    if ((skip_next)); then
      skip_next=0
      continue
    fi
    if [[ "${arg}" == --endpoint-url ]]; then
      skip_next=1
      continue
    fi
    if [[ "${arg}" != s3 && "${arg}" != cp && "${arg}" != s3://* ]]; then
      dest="${arg}"
    fi
  done
  mkdir -p "$(dirname "${dest}")"
  echo "mock video" >"${dest}"
  exit 0
fi

echo "unsupported aws invocation: $*" >&2
exit 1
EOF
chmod +x "${MOCK_BIN}/aws"

export PATH="${MOCK_BIN}:${PATH}"
export R2_ACCESS_KEY_ID=test
export R2_SECRET_ACCESS_KEY=test
export R2_ACCOUNT_ID=testaccount
export R2_BUCKET=fret-renders

DOWNLOAD="${REPO_ROOT}/scripts/download_showcase.sh"
OUT_DIR="$(mktemp -d)"

assert_contains() {
  local haystack="$1"
  local needle="$2"
  local label="$3"
  if [[ "${haystack}" != *"${needle}"* ]]; then
    echo "FAIL: ${label} — expected output to contain '${needle}'"
    echo "${haystack}"
    exit 1
  fi
  echo "OK: ${label}"
}

assert_file() {
  local path="$1"
  local label="$2"
  if [[ ! -f "${path}" ]]; then
    echo "FAIL: ${label} — missing ${path}"
    exit 1
  fi
  echo "OK: ${label}"
}

list_out="$("${DOWNLOAD}" --list)"
assert_contains "${list_out}" "dubins_race/dubins_race_overview.mp4" "--list latest nested"
assert_contains "${list_out}" "dubins_race/dubins_race_overview.csv" "--list telemetry"

single_out="${OUT_DIR}/dubins_overview.mp4"
"${DOWNLOAD}" --scenario dubins_race --camera overview -o "${single_out}"
assert_file "${single_out}" "single overview download"

"${DOWNLOAD}" --scenario dubins_race --camera overview --with-telemetry -o "${OUT_DIR}/with_tele.mp4"
assert_file "${OUT_DIR}/with_tele.mp4" "overview with telemetry video"
assert_file "${OUT_DIR}/dubins_race_overview.csv" "paired telemetry csv"
assert_file "${OUT_DIR}/dubins_race_overview.json" "paired telemetry manifest"

tag_out="$("${DOWNLOAD}" --tag v1.1.0 --list)"
assert_contains "${tag_out}" "dubins_race_overview.mp4" "--list legacy flat tag"

echo "All download_showcase checks passed."
