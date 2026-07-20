#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROVIDER="${GO2_ODOM_TIME_OFFSET_PROVIDER:-${SCRIPT_DIR}/run_go2_xt16_mouth_mapping_save_to_nav.sh}"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

is_number() {
  [[ "$1" =~ ^[+-]?(([0-9]+([.][0-9]*)?)|([.][0-9]+))([eE][+-]?[0-9]+)?$ ]]
}

[[ -x "${PROVIDER}" ]] || die "Odom time-sync provider is not executable: ${PROVIDER}"

report=""
set +e
report="$("${PROVIDER}" --measure-odom-only 2>&1)"
rc=$?
set -e

printf '%s\n' "${report}" >&2
(( rc == 0 )) || die "Odom/XT16 time-offset preflight failed (exit ${rc})."

offset="$(awk -F= '$1 == "CONFIRMED_ODOM_TIME_OFFSET_SEC" {print $2}' <<<"${report}" | tail -n 1)"
is_number "${offset}" || \
  die "Odom time-sync provider did not return a finite CONFIRMED_ODOM_TIME_OFFSET_SEC value."

printf '%s\n' "${offset}"
