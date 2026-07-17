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
  prefix="${3#s3://fret-renders/}"
  prefix="${prefix%/}/"
  case "${prefix}" in
    latest/)
      echo "2026-01-01 00:00:00    1200000 dubins_race_overview.mp4"
      echo "2026-01-01 00:00:00    1050000 dubins_race_follow.mp4"
      echo "2026-01-01 00:00:00     900000 dubins_race.mp4"
      ;;
    releases/v1.1.0/)
      echo "2026-01-01 00:00:00    1200000 dubins_race_overview.mp4"
      echo "2026-01-01 00:00:00    1050000 dubins_race_follow.mp4"
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
}

assert_file_count() {
  local dir="$1"
  local expected="$2"
  local label="$3"
  local actual
  actual="$(find "${dir}" -maxdepth 1 -name '*.mp4' | wc -l | tr -d ' ')"
  if [[ "${actual}" != "${expected}" ]]; then
    echo "FAIL: ${label} — expected ${expected} mp4 files, got ${actual}"
    ls -la "${dir}"
    exit 1
  fi
}

echo "=== download_showcase.sh mock tests ==="

list_out="$("${DOWNLOAD}" --list 2>&1)"
assert_contains "${list_out}" "dubins_race_overview.mp4" "--list latest"
assert_contains "${list_out}" "dubins_race.mp4" "--list includes legacy alias"

all_out_dir="${OUT_DIR}/all_latest"
"${DOWNLOAD}" --all -o "${all_out_dir}" >/dev/null
assert_file_count "${all_out_dir}" 3 " --all on latest downloads only listed objects"

v11_out_dir="${OUT_DIR}/all_v1_1"
"${DOWNLOAD}" --all --tag v1.1.0 -o "${v11_out_dir}" >/dev/null
assert_file_count "${v11_out_dir}" 2 " --all on v1.1.0 downloads every listed clip"

single_out="${OUT_DIR}/dubins_overview.mp4"
"${DOWNLOAD}" --scenario dubins_race --camera overview -o "${single_out}" >/dev/null
if [[ ! -s "${single_out}" ]]; then
  echo "FAIL: single download did not create output file"
  exit 1
fi

if "${DOWNLOAD}" --tag v9.9.9 --list >/dev/null 2>&1; then
  echo "FAIL: expected missing prefix to fail"
  exit 1
fi

echo "PASS: download_showcase.sh mock tests"
