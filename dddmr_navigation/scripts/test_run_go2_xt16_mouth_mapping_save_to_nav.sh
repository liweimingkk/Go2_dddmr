#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${SCRIPT_DIR}/run_go2_xt16_mouth_mapping_save_to_nav.sh"

source "${TARGET}"

assert_contains() {
  local output="$1"
  local expected="$2"
  if [[ "${output}" != *"${expected}"* ]]; then
    printf 'Expected output to contain: %s\nActual output:\n%s\n' \
      "${expected}" "${output}" >&2
    exit 1
  fi
}

confirmation_output="$(
  printf '\nnot-save\nSAVE\n' | wait_for_save_confirmation
)"
assert_contains \
  "${confirmation_output}" \
  "Ignored input that was not exactly SAVE; mapping continues."

set +e
closed_input_output="$(wait_for_save_confirmation </dev/null)"
closed_input_rc=$?
set -e
if (( closed_input_rc == 0 )); then
  echo "Closed input unexpectedly confirmed a save." >&2
  exit 1
fi
assert_contains "${closed_input_output}" "Control input closed before explicit SAVE."

help_output="$("${TARGET}" --help)"
assert_contains "${help_output}" "--start-only"
assert_contains "${help_output}" "--save-existing CONTAINER"
assert_contains "${help_output}" "If empty, require SAVE."

set +e
invalid_name_output="$("${TARGET}" --save-existing '../bad' 2>&1)"
invalid_name_rc=$?
set -e
if (( invalid_name_rc != 2 )); then
  printf 'Invalid container name returned %s, expected 2.\n%s\n' \
    "${invalid_name_rc}" "${invalid_name_output}" >&2
  exit 1
fi
assert_contains "${invalid_name_output}" "invalid Docker container name"

(
  COMMAND_MODE="start-only"
  OWN_CONTAINER="true"
  CONTAINER_NAME="mapping_start_only_test"
  start_called="false"
  leave_called="false"
  save_called="false"

  require_file() { :; }
  require_docker_image() { :; }
  stop_live_navigation_before_mapping() { :; }
  check_no_live_navigation() { :; }
  cleanup_stale_mapping_runtime() { :; }
  measure_mouth_time_offset() { :; }
  measure_odom_time_offset() { :; }
  start_mapping_container() { start_called="true"; }
  wait_for_service() { return 0; }
  wait_for_topic() { return 0; }
  require_mouth_ground_sample() { :; }
  start_map_result_rviz() { :; }
  leave_mapping_running() { leave_called="true"; }
  docker_ros() {
    save_called="true"
    return 0
  }
  docker() { return 0; }

  main

  [[ "${start_called}" == "true" ]] || {
    echo "--start-only did not start the mapping container." >&2
    exit 1
  }
  [[ "${leave_called}" == "true" ]] || {
    echo "--start-only did not leave the mapping container running." >&2
    exit 1
  }
  [[ "${save_called}" == "false" ]] || {
    echo "--start-only unexpectedly entered the save path." >&2
    exit 1
  }
)

echo "Mapping save workflow shell tests passed."
