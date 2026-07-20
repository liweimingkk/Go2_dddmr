#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"

BAGS_DIR_VALUE="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
ROUTE_ID_VALUE="${ROUTE_ID:-flat_route_01}"
ROUTE_FILE_VALUE="${ROUTE_FILE:-${BAGS_DIR_VALUE}/routes/${ROUTE_ID_VALUE}.json}"
ROUTE_MAP_DIR_VALUE="${ROUTE_MAP_DIR:-${BAGS_DIR_VALUE}/go2_xt16_mouth_mapping_20260714_153136_map_2026_07_14_07_31_36}"
RECORDER="${SCRIPT_DIR}/record_go2_xt16_recorded_route.sh"

if [[ ! -d "${ROUTE_MAP_DIR_VALUE}" ]]; then
  printf 'ERROR: route map directory does not exist: %s\n' "${ROUTE_MAP_DIR_VALUE}" >&2
  exit 1
fi
if [[ ! -x "${RECORDER}" ]]; then
  printf 'ERROR: route recorder is missing or not executable: %s\n' "${RECORDER}" >&2
  exit 1
fi

export ROUTE_ID="${ROUTE_ID_VALUE}"
export ROUTE_FILE="${ROUTE_FILE_VALUE}"
export ROUTE_MAP_DIR="${ROUTE_MAP_DIR_VALUE}"

printf 'ROUTE_ID=%s\n' "${ROUTE_ID}"
printf 'ROUTE_FILE=%s\n' "${ROUTE_FILE}"
printf 'ROUTE_MAP_DIR=%s\n' "${ROUTE_MAP_DIR}"
printf '%s\n' 'Recording requires the dry-run stack to be READY/DISABLED and localization to be TRACKING.'

exec "${RECORDER}" "$@"
