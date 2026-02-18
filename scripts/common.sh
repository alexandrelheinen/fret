#!/usr/bin/env bash

if [[ -t 1 ]]; then
  C_RESET='\033[0m'
  C_BLUE='\033[1;34m'
  C_GREEN='\033[1;32m'
  C_YELLOW='\033[1;33m'
  C_RED='\033[1;31m'
else
  C_RESET=''
  C_BLUE=''
  C_GREEN=''
  C_YELLOW=''
  C_RED=''
fi

info() { echo -e "${C_BLUE}[INFO]${C_RESET} $*"; }
ok() { echo -e "${C_GREEN}[ OK ]${C_RESET} $*"; }
warn() { echo -e "${C_YELLOW}[WARN]${C_RESET} $*"; }
fail() { echo -e "${C_RED}[FAIL]${C_RESET} $*"; }

on_error() {
  local line="$1"
  local context="${2:-Operation}"
  fail "${context} failed at line ${line}."
}

require_not_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    fail "Do not run this script as root. Run it as your normal user (sudo will be used when needed)."
    exit 1
  fi
}

require_command() {
  local command_name="$1"
  local message="${2:-${command_name} is required but not installed.}"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    fail "${message}"
    exit 1
  fi
}
