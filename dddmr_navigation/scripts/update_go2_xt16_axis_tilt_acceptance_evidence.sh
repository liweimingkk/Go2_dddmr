#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/update_go2_xt16_axis_tilt_acceptance_evidence.sh \
    --evidence FILE --update KEY_VALUE_FILE [--dry-run]
  scripts/update_go2_xt16_axis_tilt_acceptance_evidence.sh --self-test

Safely apply KEY=VALUE updates to the Go2 XT16 axis/tilt acceptance evidence
file after a manual or supervised gate has produced artifacts.

The update file is parsed as data and is not sourced. Unknown keys, duplicate
keys, and malformed KEY=VALUE lines are rejected. The resulting evidence file is
checked with the local acceptance verifier before it is written.

This script does not source ROS, inspect live robot topics, start Docker,
publish /cmd_vel, publish /dddmr_go2/dry_run_cmd_vel, publish /api/sport/request,
or publish /lowcmd.
EOF
}

evidence_file=""
update_file=""
dry_run=false
self_test=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --evidence)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --evidence requires a file" >&2
        exit 2
      }
      evidence_file="$2"
      shift 2
      ;;
    --update)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --update requires a file" >&2
        exit 2
      }
      update_file="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    --self-test)
      self_test=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERIFIER="${WS_ROOT}/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh"

die() {
  echo "ERROR: $*" >&2
  exit 2
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "${value}"
}

known_key() {
  case "$1" in
    REPORT_PATH|\
    RVIZ_BASE_LINK_FRONT_OBJECT_STATUS|RVIZ_FIXED_FRAME|RVIZ_FRONT_OBJECT_DIRECTION|SCREENSHOT|\
    TF_HEALTH_MAP_ODOM_STATUS|TF_HEALTH_MAP_BASE_STATUS|TF_HEALTH_STATUS|TF_HEALTH_LOG|\
    RAW_ODOM_AXIS_STATUS|STANDARD_ODOM_AXIS_STATUS|RAW_STANDARD_ODOM_MATERIAL_MATCH|\
    ODOM_AXIS_ROTATION_CHECK|RAW_ODOM_AXIS_LOG|STANDARD_ODOM_AXIS_LOG|\
    GO2_XT16_SPORT_LIVE_READINESS|SPORT_LIVE_READINESS_LOG|SPORT_PROBE_RESULT|\
    SPORT_PROBE_SUMMARY|SPORT_PROBE_HAS_MOVE_1008|SPORT_PROBE_HAS_STOPMOVE_1003|\
    LIVE_NAV_SHORT_GOAL_STATUS|LIVE_NAV_SUMMARY|LIVE_NAV_LOG|LIVE_NAV_MAX_X_MPS|\
    NO_RESIDUAL_RUNTIME_STATUS|NO_RESIDUAL_RUNTIME_LOG)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

declare -A updates=()

parse_update_file() {
  local path="$1"
  local raw
  local line
  local key
  local value
  local line_no=0

  [[ -f "${path}" ]] || die "missing update file: ${path}"
  while IFS= read -r raw || [[ -n "${raw}" ]]; do
    line_no=$((line_no + 1))
    line="${raw%$'\r'}"
    [[ -z "$(trim "${line}")" ]] && continue
    [[ "$(trim "${line}")" == \#* ]] && continue
    [[ "${line}" == *"="* ]] || die "malformed update line ${line_no}: missing '='"

    key="$(trim "${line%%=*}")"
    value="${line#*=}"
    [[ "${key}" =~ ^[A-Z0-9_]+$ ]] || die "invalid key at line ${line_no}: ${key}"
    known_key "${key}" || die "unknown evidence key at line ${line_no}: ${key}"
    if [[ -v "updates[${key}]" ]]; then
      die "duplicate update key at line ${line_no}: ${key}"
    fi
    updates["${key}"]="${value}"
  done <"${path}"

  [[ ${#updates[@]} -gt 0 ]] || die "update file has no KEY=VALUE entries"
}

apply_updates_to_temp() {
  local source_file="$1"
  local dest_file="$2"
  local raw
  local line
  local key
  declare -A applied=()

  [[ -f "${source_file}" ]] || die "missing evidence file: ${source_file}"
  while IFS= read -r raw || [[ -n "${raw}" ]]; do
    line="${raw%$'\r'}"
    if [[ "${line}" == *"="* ]]; then
      key="$(trim "${line%%=*}")"
      if [[ -v "updates[${key}]" ]]; then
        printf '%s=%s\n' "${key}" "${updates[${key}]}"
        applied["${key}"]=1
        continue
      fi
    fi
    printf '%s\n' "${line}"
  done <"${source_file}" >"${dest_file}"

  for key in "${!updates[@]}"; do
    if [[ ! -v "applied[${key}]" ]]; then
      die "evidence file does not contain key: ${key}"
    fi
  done
}

run_self_test() {
  local self_path
  local tmp_dir
  local evidence
  local update
  local bad_update
  local out
  local status

  self_path="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN
  evidence="${tmp_dir}/evidence.env"
  update="${tmp_dir}/update.env"
  bad_update="${tmp_dir}/bad_update.env"

  cp "${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env.example" "${evidence}"
  cat >"${update}" <<'EOF'
RVIZ_FIXED_FRAME=base_link
RVIZ_FRONT_OBJECT_DIRECTION=+X
EOF

  out="$("${self_path}" --evidence "${evidence}" --update "${update}" --dry-run)"
  [[ "${out}" == *"GO2_XT16_EVIDENCE_UPDATE_DRY_RUN=PASS"* ]] || {
    echo "${out}" >&2
    echo "ERROR: dry-run update did not pass" >&2
    return 1
  }
  grep -Fxq "RVIZ_FIXED_FRAME=TODO" "${evidence}" || {
    echo "ERROR: dry-run modified evidence file" >&2
    return 1
  }
  echo "GO2_XT16_EVIDENCE_UPDATER_SELFTEST_DRY_RUN=PASS"

  "${self_path}" --evidence "${evidence}" --update "${update}" >/dev/null
  grep -Fxq "RVIZ_FIXED_FRAME=base_link" "${evidence}" || {
    echo "ERROR: update did not change RVIZ_FIXED_FRAME" >&2
    return 1
  }
  grep -Fxq "RVIZ_FRONT_OBJECT_DIRECTION=+X" "${evidence}" || {
    echo "ERROR: update did not change RVIZ_FRONT_OBJECT_DIRECTION" >&2
    return 1
  }
  echo "GO2_XT16_EVIDENCE_UPDATER_SELFTEST_APPLY=PASS"

  cat >"${bad_update}" <<'EOF'
UNKNOWN_GO2_FIELD=value
EOF
  set +e
  out="$("${self_path}" --evidence "${evidence}" --update "${bad_update}" 2>&1)"
  status=$?
  set -e
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: unknown update key unexpectedly passed" >&2
    return 1
  }
  [[ "${out}" == *"unknown evidence key"* ]] || {
    echo "${out}" >&2
    echo "ERROR: unknown key rejection reason not reported" >&2
    return 1
  }
  echo "GO2_XT16_EVIDENCE_UPDATER_SELFTEST_UNKNOWN_KEY_REJECT=PASS"
  echo "GO2_XT16_EVIDENCE_UPDATER_SELFTEST_STATUS=PASS"
}

if [[ "${self_test}" == "true" ]]; then
  if [[ -n "${evidence_file}" || -n "${update_file}" || "${dry_run}" == "true" ]]; then
    die "--self-test cannot be combined with --evidence, --update, or --dry-run"
  fi
  run_self_test
  exit 0
fi

[[ -n "${evidence_file}" ]] || die "--evidence is required"
[[ -n "${update_file}" ]] || die "--update is required"
[[ -x "${VERIFIER}" ]] || die "missing executable verifier: ${VERIFIER}"

parse_update_file "${update_file}"

tmp_file="$(mktemp)"
cleanup() {
  rm -f "${tmp_file:-}"
}
trap cleanup EXIT

apply_updates_to_temp "${evidence_file}" "${tmp_file}"
"${VERIFIER}" --evidence "${tmp_file}" --allow-incomplete >/dev/null

updated_keys="$(printf '%s\n' "${!updates[@]}" | sort | paste -sd, -)"
if [[ "${dry_run}" == "true" ]]; then
  echo "GO2_XT16_EVIDENCE_UPDATE_DRY_RUN=PASS"
  echo "GO2_XT16_EVIDENCE_UPDATE_KEYS=${updated_keys}"
  exit 0
fi

backup_file="${evidence_file}.bak.$(date +%Y%m%d_%H%M%S)"
cp "${evidence_file}" "${backup_file}"
mv "${tmp_file}" "${evidence_file}"
tmp_file=""

echo "GO2_XT16_EVIDENCE_UPDATE_STATUS=PASS"
echo "GO2_XT16_EVIDENCE_UPDATE_FILE=${evidence_file}"
echo "GO2_XT16_EVIDENCE_UPDATE_BACKUP=${backup_file}"
echo "GO2_XT16_EVIDENCE_UPDATE_KEYS=${updated_keys}"
