#!/usr/bin/env bash
# Upload one scenario's showcase MP4s to Cloudflare R2.
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

shopt -s nullglob
matches=("${renders_dir}/${scenario_prefix}"*.mp4)
if [[ ${#matches[@]} -eq 0 ]]; then
  echo "No MP4 files matching ${renders_dir}/${scenario_prefix}*.mp4" >&2
  exit 1
fi

python3 -m pip install --break-system-packages awscli >/dev/null

for mp4 in "${matches[@]}"; do
  base="$(basename "${mp4}")"
  echo "Uploading ${base} → releases/${TAG}/ and latest/"
  aws s3 cp "${mp4}" \
    "s3://${R2_BUCKET}/releases/${TAG}/${base}" \
    --endpoint-url "${R2_ENDPOINT}" \
    --content-type video/mp4
  aws s3 cp "${mp4}" \
    "s3://${R2_BUCKET}/latest/${base}" \
    --endpoint-url "${R2_ENDPOINT}" \
    --content-type video/mp4
done

primary="${renders_dir}/${scenario_prefix}_overview.mp4"
if [[ -f "${primary}" ]]; then
  echo "Updating latest/${latest_alias} from ${primary##*/}"
  aws s3 cp "${primary}" \
    "s3://${R2_BUCKET}/latest/${latest_alias}" \
    --endpoint-url "${R2_ENDPOINT}" \
    --content-type video/mp4
fi
