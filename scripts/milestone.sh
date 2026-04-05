#!/usr/bin/env bash

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/common.sh"

trap 'on_error $LINENO "Milestone publisher"' ERR

usage() {
  cat <<'EOF'
Usage:
  ./scripts/milestone.sh --path <docs_folder> [--repo <owner/name>] [--dry-run]
  ./scripts/milestone.sh <docs_folder>

Description:
  Creates one GitHub milestone from milestone.md and creates one GitHub issue
  per issue-*.md file in the target folder, associating all issues to the
  milestone.

Required:
  --path <docs_folder>   Folder containing milestone.md and issue-*.md files.
                         Path can be absolute or relative to repository root.

Optional:
  --repo <owner/name>    Target repository (default: current repo via gh).
  --dry-run              Show planned actions without creating milestone/issues.
  -h, --help             Show this help message.

Examples:
  ./scripts/milestone.sh --path docs/arco
  ./scripts/milestone.sh docs/arco --repo alexandrelheinen/fret
  ./scripts/milestone.sh --path docs/arco --dry-run
EOF
}

DOCS_PATH=""
REPO=""
DRY_RUN=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --path)
      [[ $# -ge 2 ]] || { fail "Missing value for --path"; usage; exit 1; }
      DOCS_PATH="$2"
      shift 2
      ;;
    --repo)
      [[ $# -ge 2 ]] || { fail "Missing value for --repo"; usage; exit 1; }
      REPO="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ -z "${DOCS_PATH}" ]]; then
        DOCS_PATH="$1"
        shift
      else
        fail "Unknown argument: $1"
        usage
        exit 1
      fi
      ;;
  esac
done

if [[ -z "${DOCS_PATH}" ]]; then
  fail "Path to docs folder is required."
  usage
  exit 1
fi

require_command sed "sed is required but not installed."
require_command find "find is required but not installed."
require_command sort "sort is required but not installed."
require_command git "git is required but not installed."

cd "${PROJECT_ROOT}"

if [[ "${DOCS_PATH}" != /* ]]; then
  DOCS_PATH="${PROJECT_ROOT}/${DOCS_PATH}"
fi

if [[ ! -d "${DOCS_PATH}" ]]; then
  fail "Folder not found: ${DOCS_PATH}"
  exit 1
fi

MILESTONE_FILE="${DOCS_PATH}/milestone.md"
if [[ ! -f "${MILESTONE_FILE}" ]]; then
  fail "Required file not found: ${MILESTONE_FILE}"
  exit 1
fi

resolve_repo_from_remote() {
  local remote_url
  remote_url="$(git config --get remote.origin.url || true)"

  if [[ -z "${remote_url}" ]]; then
    return 1
  fi

  # Supports both forms:
  # - git@github.com:owner/repo.git
  # - https://github.com/owner/repo.git
  remote_url="${remote_url%.git}"
  remote_url="${remote_url#git@github.com:}"
  remote_url="${remote_url#https://github.com/}"
  remote_url="${remote_url#http://github.com/}"

  if [[ "${remote_url}" == */* ]]; then
    echo "${remote_url}"
    return 0
  fi

  return 1
}

if [[ -z "${REPO}" ]]; then
  if command -v gh >/dev/null 2>&1; then
    REPO="$(gh repo view --json nameWithOwner --jq '.nameWithOwner' 2>/dev/null || true)"
  fi

  if [[ -z "${REPO}" ]]; then
    REPO="$(resolve_repo_from_remote || true)"
  fi
fi

if [[ -z "${REPO}" ]]; then
  fail "Could not resolve target repository. Use --repo <owner/name>."
  exit 1
fi

if [[ "${DRY_RUN}" -eq 0 ]]; then
  require_command gh "GitHub CLI (gh) is required but not installed."
  gh auth status >/dev/null
fi

MILESTONE_TITLE="$(sed -n '1s/^# //p' "${MILESTONE_FILE}")"
if [[ -z "${MILESTONE_TITLE}" ]]; then
  fail "Could not parse milestone title from first heading in ${MILESTONE_FILE}"
  exit 1
fi

info "Target repository: ${REPO}"
info "Docs folder: ${DOCS_PATH}"
info "Milestone title: ${MILESTONE_TITLE}"

MILESTONE_NUMBER=""
if [[ "${DRY_RUN}" -eq 0 ]]; then
  MILESTONE_NUMBER="$(gh api "repos/${REPO}/milestones?state=all&per_page=100" --jq ".[] | select(.title == \"${MILESTONE_TITLE}\") | .number" | head -n 1 || true)"

  if [[ -z "${MILESTONE_NUMBER}" ]]; then
    info "Creating milestone..."
    MILESTONE_BODY="$(cat "${MILESTONE_FILE}")"
    gh api -X POST "repos/${REPO}/milestones" \
      -f title="${MILESTONE_TITLE}" \
      -f description="${MILESTONE_BODY}" >/dev/null

    MILESTONE_NUMBER="$(gh api "repos/${REPO}/milestones?state=all&per_page=100" --jq ".[] | select(.title == \"${MILESTONE_TITLE}\") | .number" | head -n 1 || true)"
  else
    info "Milestone already exists (#${MILESTONE_NUMBER})."
  fi

  if [[ -z "${MILESTONE_NUMBER}" ]]; then
    fail "Failed to resolve milestone number for '${MILESTONE_TITLE}'."
    exit 1
  fi
else
  info "[DRY-RUN] Would ensure milestone exists: ${MILESTONE_TITLE}"
fi

mapfile -t ISSUE_FILES < <(find "${DOCS_PATH}" -maxdepth 1 -type f -name 'issue-*.md' | sort)

if [[ "${#ISSUE_FILES[@]}" -eq 0 ]]; then
  warn "No issue files found in ${DOCS_PATH} matching issue-*.md"
  exit 0
fi

CREATED_COUNT=0
SKIPPED_COUNT=0

for issue_file in "${ISSUE_FILES[@]}"; do
  issue_title="$(sed -n '1s/^# //p' "${issue_file}")"

  if [[ -z "${issue_title}" ]]; then
    warn "Skipping ${issue_file}: could not parse title from first heading"
    SKIPPED_COUNT=$((SKIPPED_COUNT + 1))
    continue
  fi

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    info "[DRY-RUN] Would create issue: ${issue_title}"
    CREATED_COUNT=$((CREATED_COUNT + 1))
    continue
  fi

  existing_issue_number="$(gh issue list --repo "${REPO}" --state all --limit 200 --search "\"${issue_title}\" in:title" --json title,number --jq ".[] | select(.title == \"${issue_title}\") | .number" | head -n 1 || true)"

  if [[ -n "${existing_issue_number}" ]]; then
    info "Skipping existing issue #${existing_issue_number}: ${issue_title}"
    SKIPPED_COUNT=$((SKIPPED_COUNT + 1))
    continue
  fi

  body_tmp="$(mktemp)"
  tail -n +2 "${issue_file}" > "${body_tmp}"

  info "Creating issue: ${issue_title}"
  gh issue create \
    --repo "${REPO}" \
    --title "${issue_title}" \
    --body-file "${body_tmp}" \
    --milestone "${MILESTONE_TITLE}" >/dev/null

  rm -f "${body_tmp}"
  CREATED_COUNT=$((CREATED_COUNT + 1))
done

echo
ok "Milestone publishing completed."
info "Issues processed: ${#ISSUE_FILES[@]}"
info "Created: ${CREATED_COUNT}"
info "Skipped: ${SKIPPED_COUNT}"
