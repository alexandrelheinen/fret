#!/usr/bin/env bash
# Upload one scenario's showcase MP4s + matching telemetry to Cloudflare R2.
#
# Layout (per scenario folder; video and log share the same basename):
#   releases/${TAG}/${scenario}/${scenario}_overview.mp4
#   releases/${TAG}/${scenario}/${scenario}_overview.csv
#   releases/${TAG}/${scenario}/${scenario}_overview.json
#   releases/${TAG}/${scenario}/${scenario}_follow.mp4
#   latest/${scenario}/...   (skipped for *-dev* tags / SKIP_LATEST=1)
#
# Usage:
#   upload_scenario_r2.sh <scenario_prefix> <latest_alias_basename>
#
# Example:
#   upload_scenario_r2.sh dubins_race dubins_race.mp4
#
# Requires: awscli, R2_ENDPOINT, R2_BUCKET, AWS_ACCESS_KEY_ID,
# AWS_SECRET_ACCESS_KEY, TAG
set -Eeuo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 <scenario_prefix> <latest_alias_basename>" >&2
  exit 2
fi

scenario_prefix="$1"
latest_alias="$2"
renders_dir="${RENDERS_DIR:-showcase_renders}"

if [[ -z "${TAG:-}" ]]; then
  echo "TAG env var is required" >&2
  exit 2
fi
if [[ -z "${R2_ENDPOINT:-}" || -z "${R2_BUCKET:-}" ]]; then
  echo "R2_ENDPOINT and R2_BUCKET env vars are required" >&2
  exit 2
fi

# Only promote clean semver tags (vX.Y.Z) to latest/. Suffixes like
# -dev / -agent-probe stay under releases/<tag>/ only.
update_latest=1
if [[ "${SKIP_LATEST:-0}" == "1" || ! "${TAG}" =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  update_latest=0
  echo "Skipping latest/ update for TAG=${TAG}"
fi

shopt -s nullglob
matches=("${renders_dir}/${scenario_prefix}"*.mp4)
if [[ ${#matches[@]} -eq 0 ]]; then
  echo "No MP4 files matching ${renders_dir}/${scenario_prefix}*.mp4" >&2
  exit 1
fi

python3 -m pip install --break-system-packages awscli >/dev/null

upload_file() {
  local src="$1"
  local dest_key="$2"
  local content_type="$3"
  echo "Uploading $(basename "${src}") → ${dest_key}"
  aws s3 cp "${src}" \
    "s3://${R2_BUCKET}/${dest_key}" \
    --endpoint-url "${R2_ENDPOINT}" \
    --content-type "${content_type}"
}

release_prefix="releases/${TAG}/${scenario_prefix}"
latest_prefix="latest/${scenario_prefix}"

for mp4 in "${matches[@]}"; do
  base="$(basename "${mp4}")"
  upload_file "${mp4}" "${release_prefix}/${base}" "video/mp4"
  if [[ "${update_latest}" -eq 1 ]]; then
    upload_file "${mp4}" "${latest_prefix}/${base}" "video/mp4"
  fi

  # Matching telemetry (same basename as the video stem).
  stem="${base%.mp4}"
  csv="${renders_dir}/${stem}.csv"
  manifest="${renders_dir}/${stem}.json"
  if [[ -f "${csv}" ]]; then
    upload_file "${csv}" "${release_prefix}/${stem}.csv" "text/csv"
    if [[ "${update_latest}" -eq 1 ]]; then
      upload_file "${csv}" "${latest_prefix}/${stem}.csv" "text/csv"
    fi
  fi
  if [[ -f "${manifest}" ]]; then
    upload_file "${manifest}" "${release_prefix}/${stem}.json" "application/json"
    if [[ "${update_latest}" -eq 1 ]]; then
      upload_file "${manifest}" "${latest_prefix}/${stem}.json" "application/json"
    fi
  fi
done

primary="${renders_dir}/${scenario_prefix}_overview.mp4"
if [[ -f "${primary}" && "${update_latest}" -eq 1 ]]; then
  echo "Updating ${latest_prefix}/${latest_alias} from ${primary##*/}"
  upload_file "${primary}" "${latest_prefix}/${latest_alias}" "video/mp4"
fi
