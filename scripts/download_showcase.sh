#!/usr/bin/env bash
# Download FRET showcase MP4s from a private Cloudflare R2 bucket.
#
# Credentials (pick one):
#   1. Copy .env.example → .env at repo root (recommended; .env is gitignored)
#   2. Export R2_ACCESS_KEY_ID, R2_SECRET_ACCESS_KEY, R2_ACCOUNT_ID, R2_BUCKET
#
# Requires: awscli  (sudo apt install awscli)
#
# Examples:
#   ./scripts/download_showcase.sh
#   ./scripts/download_showcase.sh --tag v0.2.0
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

Options:
  --latest          Download latest/ppp_warehouse.mp4 (default)
  --tag VERSION     Download releases/VERSION/ppp_warehouse_showcase.mp4
  -o PATH           Output file path (default: artifacts/r2/<name>.mp4)
  -h, --help        Show this help

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

MODE="latest"
TAG=""
OUTPUT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --latest)
      MODE="latest"
      shift
      ;;
    --tag)
      MODE="tag"
      TAG="${2:?--tag requires a version such as v0.2.0}"
      shift 2
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

if [[ "${MODE}" == "latest" ]]; then
  OBJECT_KEY="latest/ppp_warehouse.mp4"
  DEFAULT_NAME="ppp_warehouse_latest.mp4"
else
  OBJECT_KEY="releases/${TAG}/ppp_warehouse_showcase.mp4"
  DEFAULT_NAME="ppp_warehouse_${TAG}.mp4"
fi

if [[ -z "${OUTPUT}" ]]; then
  OUTPUT="${REPO_ROOT}/artifacts/r2/${DEFAULT_NAME}"
fi

mkdir -p "$(dirname "${OUTPUT}")"

info "Downloading s3://${R2_BUCKET}/${OBJECT_KEY}"
if ! aws s3 cp \
  "s3://${R2_BUCKET}/${OBJECT_KEY}" \
  "${OUTPUT}" \
  --endpoint-url "${R2_ENDPOINT}"; then
  fail "Download failed. Check credentials, bucket name, and object path."
  exit 1
fi

ok "Saved ${OUTPUT} ($(du -h "${OUTPUT}" | cut -f1))"
