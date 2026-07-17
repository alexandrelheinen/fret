#!/usr/bin/env bash
# Download FRET showcase MP4s from a private Cloudflare R2 bucket.
#
# Release CI uploads two POVs per scenario: overview (whole scene) + follow.
# This script lists objects under latest/ or releases/<tag>/ and downloads
# only files that actually exist in the bucket (no hard-coded manifest).
#
# Credentials (pick one):
#   1. Copy .env.example → .env at repo root (recommended; .env is gitignored)
#   2. Export R2_ACCESS_KEY_ID, R2_SECRET_ACCESS_KEY, R2_ACCOUNT_ID, R2_BUCKET
#
# Requires: awscli  (sudo apt install awscli)
#
# Examples:
#   ./scripts/download_showcase.sh
#   ./scripts/download_showcase.sh --list
#   ./scripts/download_showcase.sh --scenario dubins_race
#   ./scripts/download_showcase.sh --tag v1.1.0 --scenario dubins_race --camera follow
#   ./scripts/download_showcase.sh --all --tag v1.1.0
#   ./scripts/download_showcase.sh -o /tmp/fret_demo.mp4
#
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=scripts/common.sh
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage: download_showcase.sh [OPTIONS]

Download MuJoCo showcase videos uploaded by the Release GitHub Actions workflow.
Objects are discovered by listing the R2 prefix; missing files are skipped with
--all or reported with a clear error for a single-file download.

Options:
  --latest              Download from latest/ (default)
  --tag VERSION         Download from releases/VERSION/
  --list                List available MP4 objects and exit
  --scenario NAME       Filter by scenario prefix (e.g. dubins_race)
  --camera NAME         POV clip: overview or follow (default: overview)
  --all                 Download every MP4 in the prefix (optionally filtered)
  -o PATH               Output file path (single download) or directory (--all)
  -h, --help            Show this help

Environment (or .env at repo root):
  R2_ACCESS_KEY_ID      R2 API token access key
  R2_SECRET_ACCESS_KEY  R2 API token secret
  R2_ACCOUNT_ID         Cloudflare account ID
  R2_BUCKET             Bucket name (default: fret-renders)
EOF
}

load_dotenv() {
  local env_file="${REPO_ROOT}/.env"
  if [[ ! -f "${env_file}" ]]; then
    return 0
  fi
  info "Loading credentials from ${env_file}"
  set -a
  # shellcheck source=/dev/null
  source "${env_file}"
  set +a
}

require_r2_env() {
  local missing=()
  [[ -n "${R2_ACCESS_KEY_ID:-}" ]] || missing+=("R2_ACCESS_KEY_ID")
  [[ -n "${R2_SECRET_ACCESS_KEY:-}" ]] || missing+=("R2_SECRET_ACCESS_KEY")
  [[ -n "${R2_ACCOUNT_ID:-}" ]] || missing+=("R2_ACCOUNT_ID")
  [[ -n "${R2_BUCKET:-}" ]] || R2_BUCKET="fret-renders"

  if ((${#missing[@]} > 0)); then
    fail "Missing R2 credentials: ${missing[*]}"
    info "Copy .env.example to .env and fill in values, or export the variables."
    exit 1
  fi
}

download_object() {
  local object_key="$1"
  local output_path="$2"
  mkdir -p "$(dirname "${output_path}")"
  info "Downloading s3://${R2_BUCKET}/${object_key}"
  if ! aws s3 cp \
    "s3://${R2_BUCKET}/${object_key}" \
    "${output_path}" \
    --endpoint-url "${R2_ENDPOINT}"; then
    fail "Download failed for ${object_key}"
    return 1
  fi
  ok "Saved ${output_path} ($(du -h "${output_path}" | cut -f1))"
}

legacy_primary_object() {
  local scenario="$1"
  case "${scenario}" in
    dubins_race) echo "dubins_race.mp4" ;;
    *)
      fail "Unknown scenario: ${scenario}"
      exit 1
      ;;
  esac
}

list_r2_mp4_objects() {
  local prefix="$1"
  local listing=""
  if ! listing="$(
    aws s3 ls "s3://${R2_BUCKET}/${prefix}/" --endpoint-url "${R2_ENDPOINT}" 2>&1
  )"; then
    fail "Cannot list s3://${R2_BUCKET}/${prefix}/"
    echo "${listing}" >&2
    return 1
  fi

  local object_name=""
  local -a objects=()
  while IFS= read -r line; do
    [[ -z "${line}" ]] && continue
    object_name="${line##* }"
    if [[ "${object_name}" == *.mp4 ]]; then
      objects+=("${object_name}")
    fi
  done <<<"${listing}"

  if ((${#objects[@]} == 0)); then
    fail "No MP4 objects found under s3://${R2_BUCKET}/${prefix}/"
    return 1
  fi

  printf '%s\n' "${objects[@]}"
}

object_matches_scenario() {
  local object_name="$1"
  local scenario="$2"
  [[ "${object_name}" == "${scenario}.mp4" || "${object_name}" == "${scenario}_"* ]]
}

object_matches_camera() {
  local object_name="$1"
  local scenario="$2"
  local camera="$3"
  if [[ "${object_name}" == "${scenario}_${camera}.mp4" ]]; then
    return 0
  fi
  if [[ "${camera}" == "overview" && "${object_name}" == "$(legacy_primary_object "${scenario}")" ]]; then
    return 0
  fi
  return 1
}

filter_objects() {
  local scenario="${1:-}"
  local camera="${2:-}"
  shift 2 || true
  local objects=("$@")
  local filtered=()
  local object_name=""

  for object_name in "${objects[@]}"; do
    if [[ -n "${scenario}" ]] && ! object_matches_scenario "${object_name}" "${scenario}"; then
      continue
    fi
    if [[ -n "${camera}" ]] && ! object_matches_camera "${object_name}" "${scenario}" "${camera}"; then
      continue
    fi
    filtered+=("${object_name}")
  done

  if ((${#filtered[@]} > 0)); then
    printf '%s\n' "${filtered[@]}"
  fi
}

# Populated by load_collected_objects on success.
COLLECTED_OBJECTS=()

load_collected_objects() {
  local prefix="$1"
  local scenario="${2:-}"
  local camera="${3:-}"
  local -a objects=()
  local listed=""

  COLLECTED_OBJECTS=()
  if ! listed="$(list_r2_mp4_objects "${prefix}")"; then
    return 1
  fi
  mapfile -t objects <<<"${listed}"

  if [[ -n "${scenario}" || -n "${camera}" ]]; then
    mapfile -t COLLECTED_OBJECTS < <(
      filter_objects "${scenario}" "${camera}" "${objects[@]}"
    )
    if ((${#COLLECTED_OBJECTS[@]} == 0)); then
      fail "No matching MP4 objects under s3://${R2_BUCKET}/${prefix}/"
      info "Available objects:"
      printf '  %s\n' "${objects[@]}"
      return 1
    fi
    return 0
  fi

  COLLECTED_OBJECTS=("${objects[@]}")
  return 0
}

print_object_list() {
  local prefix="$1"
  local scenario="${2:-}"

  if ! load_collected_objects "${prefix}" "${scenario}" ""; then
    exit 1
  fi
  info "Objects under s3://${R2_BUCKET}/${prefix}/:"
  printf '  %s\n' "${COLLECTED_OBJECTS[@]}"
}

select_single_object() {
  local prefix="$1"
  local scenario="$2"
  local camera="$3"
  local canonical="${scenario}_${camera}.mp4"
  local object_name=""

  if ! load_collected_objects "${prefix}" "${scenario}" "${camera}"; then
    return 1
  fi

  for object_name in "${COLLECTED_OBJECTS[@]}"; do
    if [[ "${object_name}" == "${canonical}" ]]; then
      echo "${object_name}"
      return 0
    fi
  done

  if ((${#COLLECTED_OBJECTS[@]} > 1)); then
    warn "Multiple matches for ${scenario}/${camera}; using ${COLLECTED_OBJECTS[0]}"
  fi
  echo "${COLLECTED_OBJECTS[0]}"
}

MODE="latest"
TAG=""
SCENARIO=""
SCENARIO_EXPLICIT=0
CAMERA="overview"
DOWNLOAD_ALL=0
LIST_ONLY=0
OUTPUT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --latest)
      MODE="latest"
      shift
      ;;
    --tag)
      MODE="tag"
      TAG="${2:?--tag requires a version such as v1.1.0}"
      shift 2
      ;;
    --list)
      LIST_ONLY=1
      shift
      ;;
    --scenario)
      SCENARIO="${2:?--scenario requires a scenario name such as dubins_race}"
      SCENARIO_EXPLICIT=1
      shift 2
      ;;
    --camera)
      CAMERA="${2:?--camera requires overview or follow}"
      shift 2
      ;;
    --all)
      DOWNLOAD_ALL=1
      shift
      ;;
    -o)
      OUTPUT="${2:?-o requires a path}"
      shift 2
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      fail "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

load_dotenv
require_r2_env
require_command aws "Install awscli: sudo apt install awscli"

R2_ENDPOINT="https://${R2_ACCOUNT_ID}.r2.cloudflarestorage.com"
export AWS_ACCESS_KEY_ID="${R2_ACCESS_KEY_ID}"
export AWS_SECRET_ACCESS_KEY="${R2_SECRET_ACCESS_KEY}"
export AWS_DEFAULT_REGION="auto"

prefix="latest"
if [[ "${MODE}" == "tag" ]]; then
  prefix="releases/${TAG}"
fi

if [[ -z "${SCENARIO}" ]]; then
  SCENARIO="dubins_race"
fi

if [[ "${LIST_ONLY}" -eq 1 ]]; then
  if [[ "${SCENARIO_EXPLICIT}" -eq 1 ]]; then
    print_object_list "${prefix}" "${SCENARIO}"
  else
    print_object_list "${prefix}"
  fi
  exit 0
fi

if [[ "${DOWNLOAD_ALL}" -eq 1 ]]; then
  out_dir="${OUTPUT:-${REPO_ROOT}/artifacts/r2/${prefix##*/}}"
  mkdir -p "${out_dir}"
  if [[ "${SCENARIO_EXPLICIT}" -eq 1 ]]; then
    load_collected_objects "${prefix}" "${SCENARIO}" "" || exit 1
  else
    load_collected_objects "${prefix}" "" "" || exit 1
  fi
  for object_name in "${COLLECTED_OBJECTS[@]}"; do
    download_object "${prefix}/${object_name}" "${out_dir}/${object_name}"
  done
  ok "Downloaded ${#COLLECTED_OBJECTS[@]} file(s) to ${out_dir}"
  exit 0
fi

object_name="$(select_single_object "${prefix}" "${SCENARIO}" "${CAMERA}")"
if [[ -z "${object_name}" ]]; then
  fail "No MP4 found for scenario=${SCENARIO} camera=${CAMERA} under ${prefix}/"
  print_object_list "${prefix}" "${SCENARIO}" || print_object_list "${prefix}"
  exit 1
fi

if [[ "${MODE}" == "latest" && "${CAMERA}" == "overview" && -z "${OUTPUT}" ]]; then
  OUTPUT="${REPO_ROOT}/artifacts/r2/${SCENARIO}_latest.mp4"
fi

OUTPUT="${OUTPUT:-${REPO_ROOT}/artifacts/r2/${object_name}}"
download_object "${prefix}/${object_name}" "${OUTPUT}"
