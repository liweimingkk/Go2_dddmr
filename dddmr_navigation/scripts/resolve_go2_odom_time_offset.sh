#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROVIDER="${GO2_ODOM_TIME_OFFSET_PROVIDER:-${SCRIPT_DIR}/run_go2_xt16_mouth_mapping_save_to_nav.sh}"
DDS_BUFFER_CHECKER="${GO2_DDS_BUFFER_CHECKER:-${SCRIPT_DIR}/check_go2_dds_receive_buffers.sh}"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

is_number() {
  [[ "$1" =~ ^[+-]?(([0-9]+([.][0-9]*)?)|([.][0-9]+))([eE][+-]?[0-9]+)?$ ]]
}

[[ -x "${PROVIDER}" ]] || die "Odom time-sync provider is not executable: ${PROVIDER}"

# The short sensor preflight can pass with the stock JetPack 5 receive-buffer
# cap and still lose tens of thousands of DDS datagrams once the dense static
# layer consumes all CPU cores.  Reject that host state before the provider
# starts any ROS process.  Other platforms retain their existing wrapper-level
# policy.
if [[ "${DDDMR_PLATFORM:-x64}" == "orin-jp5" ]]; then
  [[ -x "${DDS_BUFFER_CHECKER}" ]] || \
    die "DDS receive-buffer checker is not executable: ${DDS_BUFFER_CHECKER}"
  "${DDS_BUFFER_CHECKER}" >&2 || \
    die "Orin DDS receive-buffer check failed before odom/XT16 preflight."
fi

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
