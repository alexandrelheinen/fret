#!/usr/bin/env bash
# Download FRET showcase MP4s from a private Cloudflare R2 bucket.
#
# Release CI uploads two POVs per scenario: overview (whole scene) + follow.
#
# Credentials (pick one):
#   1. Copy .env.example → .env at repo root (recommended; .env is gitignored)
#   2. Export R2_ACCESS_KEY_ID, R2_SECRET_ACCESS_KEY, R2_ACCOUNT_ID, R2_BUCKET
#
# Requires: awscli  (sudo apt install awscli)
#
# Examples:
#   ./scripts/download_showcase.sh
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

RELEASE_SCENARIOS=(ppp_warehouse dubins_race)
RELEASE_CAMERAS=(overview follow)

usage() {
  cat <<'EOF'
Usage: download_showcase.sh [OPTIONS]

Download MuJoCo showcase videos uploaded by the Release GitHub Actions workflow.

Options:
  --latest              Download latest/<scenario>_overview.mp4 (default)
  --tag VERSION         Download from releases/VERSION/
  --scenario NAME       ppp_warehouse (default) or dubins_race
  --camera NAME         POV clip: overview or follow (default: overview)
  --all                 Download every release POV (both scenarios, or one with --scenario)
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
    ppp_warehouse) echo "ppp_warehouse.mp4" ;;
    dubins_race) echo "dubins_race.mp4" ;;
    *)
      fail "Unknown scenario: ${scenario}"
      exit 1
      ;;
  esac
}

MODE="latest"
TAG=""
SCENARIO=""
SCENARIO_EXPLICIT=0
CAMERA="overview"
DOWNLOAD_ALL=0
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
    --scenario)
      SCENARIO="${2:?--scenario requires ppp_warehouse or dubins_race}"
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
  SCENARIO="ppp_warehouse"
fi

if [[ "${DOWNLOAD_ALL}" -eq 1 ]]; then
  if [[ "${SCENARIO_EXPLICIT}" -eq 1 ]]; then
    scenarios=("${SCENARIO}")
  else
    scenarios=("${RELEASE_SCENARIOS[@]}")
  fi
  out_dir="${OUTPUT:-${REPO_ROOT}/artifacts/r2/${prefix##*/}}"
  mkdir -p "${out_dir}"
  for scenario in "${scenarios[@]}"; do
    for cam in "${RELEASE_CAMERAS[@]}"; do
      file_name="${scenario}_${cam}.mp4"
      download_object "${prefix}/${file_name}" "${out_dir}/${file_name}"
    done
  done
  exit 0
fi

file_name="${SCENARIO}_${CAMERA}.mp4"
if [[ "${MODE}" == "latest" && "${CAMERA}" == "overview" && -z "${OUTPUT}" ]]; then
  default_out="${REPO_ROOT}/artifacts/r2/${SCENARIO}_latest.mp4"
  if download_object "latest/${file_name}" "${default_out}"; then
    exit 0
  fi
  legacy="$(legacy_primary_object "${SCENARIO}")"
  warn "Falling back to legacy object latest/${legacy}"
  download_object "latest/${legacy}" "${default_out}"
  exit 0
fi

OUTPUT="${OUTPUT:-${REPO_ROOT}/artifacts/r2/${file_name}}"
download_object "${prefix}/${file_name}" "${OUTPUT}"
