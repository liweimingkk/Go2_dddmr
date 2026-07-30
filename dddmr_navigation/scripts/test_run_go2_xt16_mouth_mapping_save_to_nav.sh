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

assert_file_contains() {
  local path="$1"
  local expected="$2"
  grep -Fq -- "${expected}" "${path}" || {
    printf 'Expected %s to contain: %s\n' "${path}" "${expected}" >&2
    exit 1
  }
}

CONFIG_FILE="${SCRIPT_DIR}/../src/dddmr_lego_loam/lego_loam_bor/config/loam_go2_xt16_mouth_config.yaml"
LAUNCH_FILE="${SCRIPT_DIR}/../src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_go2_xt16_mouth.launch"

assert_file_contains "${CONFIG_FILE}" 'mouth_ground_mode: "connected_surface"'
assert_file_contains "${CONFIG_FILE}" "distance_between_key_frame: 0.5"
assert_file_contains "${CONFIG_FILE}" "axis_split_factor_enabled: false"
assert_file_contains "${CONFIG_FILE}" "external_odom_factor_enabled: false"
assert_file_contains "${CONFIG_FILE}" "planar_constraint_enabled: false"
assert_file_contains "${LAUNCH_FILE}" '<arg name="mouth_ground_mode" default="connected_surface"/>'
assert_file_contains \
  "${LAUNCH_FILE}" \
  '<arg name="feature_odom_time_offset_sec" default="0.0"/>'
assert_file_contains \
  "${LAUNCH_FILE}" \
  '<param name="featureAssociation.odom_time_offset_sec" value="$(var feature_odom_time_offset_sec)"/>'
assert_file_contains \
  "${TARGET}" \
  'MOUTH_GROUND_MODE_VALUE="${MOUTH_GROUND_MODE:-connected_surface}"'
assert_file_contains "${TARGET}" "profile=normal_6dof_ramp"
assert_file_contains "${TARGET}" "feature_odom_time_offset_sec:=0.0"

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

set +e
signal_output="$(
  (
    CONTAINER_NAME="mapping_signal_test"
    CONTROLLED_MAPPING_CONTAINER="true"
    MAPPING_SIGNAL_HANDLED="false"
    INTERRUPT_STOP_TIMEOUT_SEC_VALUE="3"
    mock_running="true"

    docker() {
      case "${1:-}" in
        ps)
          if [[ "${mock_running}" == "true" ]]; then
            printf '%s\n' "${CONTAINER_NAME}"
          fi
          ;;
        stop)
          mock_running="false"
          ;;
        kill)
          mock_running="false"
          ;;
        *)
          return 2
          ;;
      esac
    }

    handle_mapping_signal INT
  )
)"
signal_rc=$?
set -e
if (( signal_rc != 130 )); then
  printf 'SIGINT handler returned %s, expected 130.\n%s\n' \
    "${signal_rc}" "${signal_output}" >&2
  exit 1
fi
assert_contains "${signal_output}" "Received INT; shutting down the mapping workflow."
assert_contains \
  "${signal_output}" \
  "Mapping container stopped; container and logs were retained: mapping_signal_test"

echo "Mapping save workflow shell tests passed."
