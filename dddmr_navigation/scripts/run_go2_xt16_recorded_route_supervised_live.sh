#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_recorded_route_supervised_live.sh [--check|--live]

Dedicated supervised launcher for a recorded Go2 XT16 route.

Modes:
  --check  Default. Validate the route, map fingerprint, selected navigation
           map, and low-speed policy without Docker, live ROS, or motion.
  --live   Start the fail-closed recorded-route Docker source, attach the host
           Sport adapter, and require an interactive route-specific arm phrase.
           This mode can publish /api/sport/request and move the physical Go2.

Required for --live:
  GO2_RECORDED_ROUTE_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_RECORDED_ROUTE
  GO2_SPORT_PROBE_SUMMARY=/tmp/go2_sport_adapter_live_..._summary.env

Route environment:
  ROUTE_FILE=<repo-root>/bags/routes/my_route_a.json
  ROUTE_MAP_DIR=<repo-root>/bags/<pose-graph-map-directory>
  GO2_RECORDED_ROUTE_EXPECTED_ID=my_route_a
  GO2_RECORDED_ROUTE_MAX_LENGTH_M=5.0
  DDDMR_BAGS_DIR=<repo-root>/bags
  NAV_CONFIG_FILE=<workspace>/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml

Low-speed live policy:
  GO2_SPORT_MAX_X=0.10             Hard cap: 0.10 m/s
  GO2_SPORT_MAX_YAW=0.20           Hard cap: 0.25 rad/s
  GO2_SPORT_PUBLISH_RATE_HZ=50.0
  GO2_SPORT_CMD_TIMEOUT_SEC=0.20
  GO2_SPORT_ZERO_EPSILON=0.001
  GO2_SPORT_STOP_KEEPALIVE_HZ=2.0
  GO2_DECISION_TIMEOUT_SEC=0.30
  GO2_RECORDED_ROUTE_MAX_RUNTIME_SEC=120
  GO2_RECORDED_ROUTE_STALL_TIMEOUT_SEC=15
  GO2_RECORDED_ROUTE_ALIGN_TIMEOUT_SEC=20
  GO2_RECORDED_ROUTE_ARM_TIMEOUT_SEC=60
  GO2_RECORDED_ROUTE_START_MAX_XY_ERROR=0.25
  GO2_RECORDED_ROUTE_START_MAX_Z_ERROR=0.20
  GO2_RECORDED_ROUTE_START_MAX_YAW_ERROR=0.35
  GO2_SPORT_PROBE_MAX_AGE_SEC=3600

Other environment:
  GO2_SETUP=<repo>/.unitree_msg_ws/install/setup.bash
  GO2_RECORDED_ROUTE_LOG_DIR=/tmp
  RVIZ=true
  PUBLISH_STATIC_TF=true

Safety contract:
  * The default mode never starts Docker or publishes ROS messages.
  * --live first launches the existing recorded-route stack DISABLED.
  * A matching live Move/StopMove adapter probe is mandatory.
  * Motion decisions are restricted to recorded-route tracking/alignment states.
  * Lateral velocity and the yaw-arc shim are disabled.
  * The operator must type `ENABLE <route_id>` after all readiness checks.
  * Ctrl-C, controller fault, progress stall, or timeout disables the controller
    and sends a StopMove burst before the runtime is removed.
EOF
}

mode="check"
if (( $# > 1 )); then
  usage >&2
  exit 2
fi
case "${1:---check}" in
  --check) mode="check" ;;
  --live) mode="live" ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    echo "Unknown command: $1" >&2
    usage >&2
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"

ROUTE_RUNNER="${SCRIPT_DIR}/run_go2_xt16_recorded_route_dry_run.sh"
ADAPTER="${WS_ROOT}/src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py"
ROUTE_LAUNCH="${WS_ROOT}/src/dddmr_beginner_guide/launch/go2_xt16_recorded_route_navigation.launch"
ROUTE_CONFIG="${WS_ROOT}/src/dddmr_route_navigation/config/go2_xt16_recorded_route.yaml"
GO2_SETUP="${GO2_SETUP:-${WS_ROOT}/.unitree_msg_ws/install/setup.bash}"
BAGS_DIR="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
DEFAULT_MAP_NAME="go2_xt16_mouth_mapping_20260714_153136_map_2026_07_14_07_31_36"
MAP_DIR="${ROUTE_MAP_DIR:-${BAGS_DIR}/${DEFAULT_MAP_NAME}}"
ROUTE_FILE_VALUE="${ROUTE_FILE:-${BAGS_DIR}/routes/my_route_a.json}"
NAV_CONFIG_FILE_VALUE="${NAV_CONFIG_FILE:-${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml}"
EXPECTED_ROUTE_ID="${GO2_RECORDED_ROUTE_EXPECTED_ID:-my_route_a}"
MAX_ROUTE_LENGTH_M="${GO2_RECORDED_ROUTE_MAX_LENGTH_M:-5.0}"

LIVE_CONFIRM_PHRASE="I_AM_SUPERVISING_GO2_RECORDED_ROUTE"
REAL_REQUEST_TOPIC="/api/sport/request"
CMD_TOPIC="/dddmr_go2/safe_cmd_vel"
DECISION_TOPIC="/dddmr_go2/p2p_decision"
MOTION_ALLOWED_DECISIONS="d_controlling,d_align_heading,d_align_goal_heading"

sport_publish_rate_hz="${GO2_SPORT_PUBLISH_RATE_HZ:-50.0}"
sport_cmd_timeout_sec="${GO2_SPORT_CMD_TIMEOUT_SEC:-0.20}"
sport_max_x="${GO2_SPORT_MAX_X:-0.10}"
sport_max_y="0.0"
sport_max_yaw="${GO2_SPORT_MAX_YAW:-0.20}"
sport_zero_epsilon="${GO2_SPORT_ZERO_EPSILON:-0.001}"
sport_stop_keepalive_hz="${GO2_SPORT_STOP_KEEPALIVE_HZ:-2.0}"
decision_timeout_sec="${GO2_DECISION_TIMEOUT_SEC:-0.30}"
max_runtime_sec="${GO2_RECORDED_ROUTE_MAX_RUNTIME_SEC:-120}"
stall_timeout_sec="${GO2_RECORDED_ROUTE_STALL_TIMEOUT_SEC:-15}"
align_timeout_sec="${GO2_RECORDED_ROUTE_ALIGN_TIMEOUT_SEC:-20}"
arm_timeout_sec="${GO2_RECORDED_ROUTE_ARM_TIMEOUT_SEC:-60}"
start_max_xy_error="${GO2_RECORDED_ROUTE_START_MAX_XY_ERROR:-0.25}"
start_max_z_error="${GO2_RECORDED_ROUTE_START_MAX_Z_ERROR:-0.20}"
start_max_yaw_error="${GO2_RECORDED_ROUTE_START_MAX_YAW_ERROR:-0.35}"
probe_max_age_sec="${GO2_SPORT_PROBE_MAX_AGE_SEC:-3600}"
probe_summary="${GO2_SPORT_PROBE_SUMMARY:-}"

rviz="${RVIZ:-true}"
publish_static_tf="${PUBLISH_STATIC_TF:-true}"
install_base="${DDDMR_INSTALL_BASE:-.docker_go2_xt16_install}"
log_dir="${GO2_RECORDED_ROUTE_LOG_DIR:-/tmp}"
stamp="$(date +%Y%m%d_%H%M%S)"
request_id_base="$(date +%s)"
container_name="go2_xt16_recorded_route_live_${stamp}"
source_start_log="${log_dir}/${container_name}_source_start.log"
container_log="${log_dir}/${container_name}_docker.log"
adapter_log="${log_dir}/${container_name}_adapter.log"
request_echo_log="${log_dir}/${container_name}_request_echo.log"
summary_log="${log_dir}/${container_name}_summary.env"

route_id=""
route_length_m=""
route_start_x=""
route_start_y=""
route_start_z=""
route_start_yaw=""
adapter_pid=""
echo_pid=""
source_attempted="false"
live_output_attempted="false"
cleanup_started="false"
final_result="NOT_STARTED"

log() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

die() {
  final_result="FAILED"
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

validate_bool() {
  local name="$1"
  local value="$2"
  case "${value}" in
    true|false) ;;
    *) die "${name} must be true or false; got '${value}'." ;;
  esac
}

summary_value() {
  local path="$1"
  local key="$2"
  awk -v key="${key}" '
    index($0, key "=") == 1 {
      print substr($0, length(key) + 2)
      found += 1
    }
    END {
      if (found != 1) {
        exit 1
      }
    }
  ' "${path}"
}

validate_low_speed_policy() {
  /usr/bin/python3 - \
    "${sport_max_x}" \
    "${sport_max_yaw}" \
    "${sport_publish_rate_hz}" \
    "${sport_cmd_timeout_sec}" \
    "${sport_zero_epsilon}" \
    "${sport_stop_keepalive_hz}" \
    "${decision_timeout_sec}" \
    "${max_runtime_sec}" \
    "${stall_timeout_sec}" \
    "${align_timeout_sec}" \
    "${arm_timeout_sec}" \
    "${start_max_xy_error}" \
    "${start_max_z_error}" \
    "${start_max_yaw_error}" \
    "${probe_max_age_sec}" \
    "${MAX_ROUTE_LENGTH_M}" <<'PY'
import math
import sys

names = [
    "GO2_SPORT_MAX_X",
    "GO2_SPORT_MAX_YAW",
    "GO2_SPORT_PUBLISH_RATE_HZ",
    "GO2_SPORT_CMD_TIMEOUT_SEC",
    "GO2_SPORT_ZERO_EPSILON",
    "GO2_SPORT_STOP_KEEPALIVE_HZ",
    "GO2_DECISION_TIMEOUT_SEC",
    "GO2_RECORDED_ROUTE_MAX_RUNTIME_SEC",
    "GO2_RECORDED_ROUTE_STALL_TIMEOUT_SEC",
    "GO2_RECORDED_ROUTE_ALIGN_TIMEOUT_SEC",
    "GO2_RECORDED_ROUTE_ARM_TIMEOUT_SEC",
    "GO2_RECORDED_ROUTE_START_MAX_XY_ERROR",
    "GO2_RECORDED_ROUTE_START_MAX_Z_ERROR",
    "GO2_RECORDED_ROUTE_START_MAX_YAW_ERROR",
    "GO2_SPORT_PROBE_MAX_AGE_SEC",
    "GO2_RECORDED_ROUTE_MAX_LENGTH_M",
]

try:
    values = [float(raw) for raw in sys.argv[1:]]
except ValueError as exc:
    raise SystemExit(f"live policy values must be numeric: {exc}")
if not all(math.isfinite(value) for value in values):
    raise SystemExit("live policy values must be finite")

settings = dict(zip(names, values))

def bounded(name, low, high, *, low_inclusive=False):
    value = settings[name]
    lower_ok = value >= low if low_inclusive else value > low
    if not lower_ok or value > high:
        bracket = "[" if low_inclusive else "("
        raise SystemExit(f"{name}={value} must be within {bracket}{low}, {high}]")

bounded("GO2_SPORT_MAX_X", 0.0, 0.10)
bounded("GO2_SPORT_MAX_YAW", 0.0, 0.25)
bounded("GO2_SPORT_PUBLISH_RATE_HZ", 10.0, 100.0, low_inclusive=True)
bounded("GO2_SPORT_CMD_TIMEOUT_SEC", 0.05, 0.30, low_inclusive=True)
bounded("GO2_SPORT_ZERO_EPSILON", 0.0, 0.01, low_inclusive=True)
bounded("GO2_SPORT_STOP_KEEPALIVE_HZ", 1.0, 10.0, low_inclusive=True)
bounded("GO2_DECISION_TIMEOUT_SEC", 0.10, 0.50, low_inclusive=True)
bounded("GO2_RECORDED_ROUTE_MAX_RUNTIME_SEC", 10.0, 300.0, low_inclusive=True)
bounded("GO2_RECORDED_ROUTE_STALL_TIMEOUT_SEC", 5.0, 30.0, low_inclusive=True)
bounded("GO2_RECORDED_ROUTE_ALIGN_TIMEOUT_SEC", 5.0, 30.0, low_inclusive=True)
bounded("GO2_RECORDED_ROUTE_ARM_TIMEOUT_SEC", 10.0, 300.0, low_inclusive=True)
bounded("GO2_RECORDED_ROUTE_START_MAX_XY_ERROR", 0.05, 0.60, low_inclusive=True)
bounded("GO2_RECORDED_ROUTE_START_MAX_Z_ERROR", 0.05, 0.35, low_inclusive=True)
bounded("GO2_RECORDED_ROUTE_START_MAX_YAW_ERROR", 0.05, 0.80, low_inclusive=True)
bounded("GO2_SPORT_PROBE_MAX_AGE_SEC", 60.0, 86400.0, low_inclusive=True)
bounded("GO2_RECORDED_ROUTE_MAX_LENGTH_M", 0.1, 20.0, low_inclusive=True)
PY
}

validate_route_and_map() {
  [[ -x "${ROUTE_RUNNER}" ]] || die "Missing route runner: ${ROUTE_RUNNER}"
  [[ -x "${ADAPTER}" ]] || die "Missing Sport adapter: ${ADAPTER}"
  [[ -f "${ROUTE_FILE_VALUE}" ]] || die "Missing recorded route: ${ROUTE_FILE_VALUE}"
  [[ -f "${MAP_DIR}/poses.pcd" ]] || die "Missing route-map poses.pcd: ${MAP_DIR}/poses.pcd"
  [[ -f "${NAV_CONFIG_FILE_VALUE}" ]] || die "Missing navigation config: ${NAV_CONFIG_FILE_VALUE}"
  [[ "${EXPECTED_ROUTE_ID}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || \
    die "GO2_RECORDED_ROUTE_EXPECTED_ID contains invalid characters."

  local bags_real route_real map_real map_relative container_map_dir
  bags_real="$(realpath -m -- "${BAGS_DIR}")"
  route_real="$(realpath -m -- "${ROUTE_FILE_VALUE}")"
  map_real="$(realpath -m -- "${MAP_DIR}")"
  [[ "${route_real}" == "${bags_real}"/* ]] || \
    die "ROUTE_FILE must stay under DDDMR_BAGS_DIR."
  [[ "${map_real}" == "${bags_real}"/* ]] || \
    die "ROUTE_MAP_DIR must stay under DDDMR_BAGS_DIR."
  map_relative="${map_real#"${bags_real}"/}"
  container_map_dir="/root/dddmr_bags/${map_relative}"
  grep -Fq "pose_graph_dir: \"${container_map_dir}\"" "${NAV_CONFIG_FILE_VALUE}" || \
    die "Navigation config does not select the route map: ${container_map_dir}"
  grep -Eq '^[[:space:]]*num_horizontal_scans:[[:space:]]*2000([[:space:]]|$)' \
    "${NAV_CONFIG_FILE_VALUE}" || \
    die "Navigation config does not match the active XT16 2000-points/ring contract."

  local route_metadata
  route_metadata="$(/usr/bin/python3 - "${ROUTE_FILE_VALUE}" <<'PY'
import json
import math
import re
import sys

path = sys.argv[1]
with open(path, encoding="utf-8") as stream:
    document = json.load(stream)

route_id = document.get("route_id")
length = document.get("route_length_3d_m")
points = document.get("points")
if not isinstance(route_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", route_id):
    raise SystemExit("route JSON has an invalid route_id")
if not isinstance(length, (int, float)) or not math.isfinite(length) or length <= 0.0:
    raise SystemExit("route JSON has an invalid route_length_3d_m")
if not isinstance(points, list) or not points or not isinstance(points[0], dict):
    raise SystemExit("route JSON has no start pose")
start = points[0]
names = ("x", "y", "z", "qx", "qy", "qz", "qw")
try:
    x, y, z, qx, qy, qz, qw = (float(start[name]) for name in names)
except (KeyError, TypeError, ValueError) as exc:
    raise SystemExit(f"route start pose is invalid: {exc}")
if not all(math.isfinite(value) for value in (x, y, z, qx, qy, qz, qw)):
    raise SystemExit("route start pose contains a non-finite value")
yaw = math.atan2(
    2.0 * (qw * qz + qx * qy),
    1.0 - 2.0 * (qy * qy + qz * qz),
)
print(route_id)
print(f"{length:.17g}")
print(f"{x:.17g}")
print(f"{y:.17g}")
print(f"{z:.17g}")
print(f"{yaw:.17g}")
PY
)"
  route_id="$(sed -n '1p' <<<"${route_metadata}")"
  route_length_m="$(sed -n '2p' <<<"${route_metadata}")"
  route_start_x="$(sed -n '3p' <<<"${route_metadata}")"
  route_start_y="$(sed -n '4p' <<<"${route_metadata}")"
  route_start_z="$(sed -n '5p' <<<"${route_metadata}")"
  route_start_yaw="$(sed -n '6p' <<<"${route_metadata}")"

  [[ "${route_id}" == "${EXPECTED_ROUTE_ID}" ]] || \
    die "Route id '${route_id}' does not match expected '${EXPECTED_ROUTE_ID}'."
  /usr/bin/python3 - "${route_length_m}" "${MAX_ROUTE_LENGTH_M}" <<'PY'
import sys

length = float(sys.argv[1])
limit = float(sys.argv[2])
if length > limit:
    raise SystemExit(
        f"route length {length:.3f} m exceeds supervised limit {limit:.3f} m"
    )
PY

  log "Validating route JSON and map fingerprint without ROS..."
  ROUTE_FILE="${ROUTE_FILE_VALUE}" \
  ROUTE_MAP_DIR="${MAP_DIR}" \
  DDDMR_BAGS_DIR="${BAGS_DIR}" \
  NAV_CONFIG_FILE="${NAV_CONFIG_FILE_VALUE}" \
  REGENERATE_ROUTE=false \
    "${ROUTE_RUNNER}" --prepare-only

  log "Validated route '${route_id}' (${route_length_m} m), start=(${route_start_x}, ${route_start_y}, ${route_start_z}, yaw=${route_start_yaw})."
}

validate_fail_closed_source() {
  [[ -f "${ROUTE_LAUNCH}" ]] || die "Missing recorded-route launch: ${ROUTE_LAUNCH}"
  [[ -f "${ROUTE_CONFIG}" ]] || die "Missing recorded-route config: ${ROUTE_CONFIG}"

  local required
  for required in \
    '<arg name="start_move_base" value="false"/>' \
    '<arg name="start_go2_sport_adapter" value="false"/>' \
    '<arg name="go2_sport_enable_output" value="false"/>' \
    '<arg name="go2_sport_allow_real_request_topic" value="false"/>' \
    '<remap from="/cmd_vel" to="/dddmr_go2/dry_run_cmd_vel"/>'
  do
    grep -Fq "${required}" "${ROUTE_LAUNCH}" || \
      die "Recorded-route launch lost fail-closed invariant: ${required}"
  done
  for required in \
    'allow_nearest_start: false' \
    'allow_reverse: false' \
    'max_linear_y: 0.0'
  do
    grep -Fq "${required}" "${ROUTE_CONFIG}" || \
      die "Recorded-route config lost fail-closed invariant: ${required}"
  done
  log "Recorded-route launch/config fail-closed invariants are present."
}

validate_probe_summary() {
  local summary="$1"
  [[ -n "${summary}" ]] || die "--live requires GO2_SPORT_PROBE_SUMMARY."
  [[ -f "${summary}" ]] || die "Missing GO2_SPORT_PROBE_SUMMARY: ${summary}"

  local summary_mode summary_result summary_request_base summary_echo
  local summary_adapter_sha current_adapter_sha
  local summary_max_x summary_max_y summary_max_yaw summary_publish_rate
  local summary_cmd_timeout summary_zero summary_keepalive summary_decision_topic
  local summary_decision_timeout summary_require_decision summary_allowed summary_probe_decision
  local summary_yaw_arc_enabled summary_yaw_arc_mode
  summary_mode="$(summary_value "${summary}" MODE)" || die "Probe summary must contain one MODE entry."
  summary_result="$(summary_value "${summary}" RESULT)" || die "Probe summary must contain one RESULT entry."
  summary_adapter_sha="$(summary_value "${summary}" ADAPTER_SHA256)" || die "Probe summary is missing ADAPTER_SHA256."
  summary_request_base="$(summary_value "${summary}" REQUEST_ID_BASE)" || die "Probe summary is missing REQUEST_ID_BASE."
  summary_echo="$(summary_value "${summary}" REQUEST_ECHO_LOG)" || die "Probe summary is missing REQUEST_ECHO_LOG."
  summary_max_x="$(summary_value "${summary}" MAX_X)" || die "Probe summary is missing MAX_X."
  summary_max_y="$(summary_value "${summary}" MAX_Y)" || die "Probe summary is missing MAX_Y."
  summary_max_yaw="$(summary_value "${summary}" MAX_YAW)" || die "Probe summary is missing MAX_YAW."
  summary_publish_rate="$(summary_value "${summary}" PUBLISH_RATE_HZ)" || die "Probe summary is missing PUBLISH_RATE_HZ."
  summary_cmd_timeout="$(summary_value "${summary}" CMD_TIMEOUT_SEC)" || die "Probe summary is missing CMD_TIMEOUT_SEC."
  summary_zero="$(summary_value "${summary}" ZERO_EPSILON)" || die "Probe summary is missing ZERO_EPSILON."
  summary_keepalive="$(summary_value "${summary}" STOP_KEEPALIVE_HZ)" || die "Probe summary is missing STOP_KEEPALIVE_HZ."
  summary_decision_topic="$(summary_value "${summary}" DECISION_TOPIC)" || die "Probe summary is missing DECISION_TOPIC."
  summary_decision_timeout="$(summary_value "${summary}" DECISION_TIMEOUT_SEC)" || die "Probe summary is missing DECISION_TIMEOUT_SEC."
  summary_require_decision="$(summary_value "${summary}" REQUIRE_MOTION_DECISION)" || die "Probe summary is missing REQUIRE_MOTION_DECISION."
  summary_allowed="$(summary_value "${summary}" MOTION_ALLOWED_DECISIONS)" || die "Probe summary is missing MOTION_ALLOWED_DECISIONS."
  summary_probe_decision="$(summary_value "${summary}" PROBE_DECISION)" || die "Probe summary is missing PROBE_DECISION."
  summary_yaw_arc_enabled="$(summary_value "${summary}" ENABLE_YAW_ARC_SHIM)" || die "Probe summary is missing ENABLE_YAW_ARC_SHIM."
  summary_yaw_arc_mode="$(summary_value "${summary}" YAW_ARC_SHIM_MODE)" || die "Probe summary is missing YAW_ARC_SHIM_MODE."

  [[ "${summary_mode}" == "live" ]] || die "Probe summary MODE is not live."
  [[ "${summary_result}" == "GO2_SPORT_ADAPTER_live_COMPLETE" ]] || \
    die "Probe summary did not complete a live adapter probe."
  [[ "${summary_adapter_sha}" =~ ^[0-9a-f]{64}$ ]] || \
    die "Probe summary has an invalid ADAPTER_SHA256."
  current_adapter_sha="$(sha256sum -- "${ADAPTER}" | awk '{print $1}')"
  [[ "${summary_adapter_sha}" == "${current_adapter_sha}" ]] || \
    die "Sport adapter changed after the live probe; run a new supervised probe."
  [[ "${summary_require_decision}" == "true" ]] || \
    die "Probe did not exercise the mandatory motion-decision gate."
  [[ "${summary_decision_topic}" == "${DECISION_TOPIC}" ]] || \
    die "Probe decision topic differs from ${DECISION_TOPIC}."
  [[ "${summary_allowed}" == "${MOTION_ALLOWED_DECISIONS}" ]] || \
    die "Probe motion decisions differ from the recorded-route live policy."
  [[ "${summary_probe_decision}" == "d_controlling" ]] || \
    die "Probe must use GO2_SPORT_PROBE_DECISION=d_controlling."
  [[ "${summary_yaw_arc_enabled}" == "false" && "${summary_yaw_arc_mode}" == "off" ]] || \
    die "Probe must keep the yaw-arc shim disabled/off."
  [[ -f "${summary_echo}" ]] || die "Probe request echo log is missing: ${summary_echo}"

  /usr/bin/python3 - \
    "${summary}" \
    "${summary_echo}" \
    "${probe_max_age_sec}" \
    "${summary_max_x}" "${sport_max_x}" \
    "${summary_max_y}" "${sport_max_y}" \
    "${summary_max_yaw}" "${sport_max_yaw}" \
    "${summary_publish_rate}" "${sport_publish_rate_hz}" \
    "${summary_cmd_timeout}" "${sport_cmd_timeout_sec}" \
    "${summary_zero}" "${sport_zero_epsilon}" \
    "${summary_keepalive}" "${sport_stop_keepalive_hz}" \
    "${summary_decision_timeout}" "${decision_timeout_sec}" <<'PY'
import math
import os
import sys
import time

summary, echo_log, max_age_raw, *pairs = sys.argv[1:]
max_age = float(max_age_raw)
now = time.time()
for name, path in (("probe summary", summary), ("probe echo log", echo_log)):
    age = now - os.path.getmtime(path)
    if age < -5.0 or age > max_age:
        raise SystemExit(f"{name} age {age:.1f}s is outside [0, {max_age:.1f}]s")

labels = [
    "MAX_X",
    "MAX_Y",
    "MAX_YAW",
    "PUBLISH_RATE_HZ",
    "CMD_TIMEOUT_SEC",
    "ZERO_EPSILON",
    "STOP_KEEPALIVE_HZ",
    "DECISION_TIMEOUT_SEC",
]
for label, index in zip(labels, range(0, len(pairs), 2)):
    observed = float(pairs[index])
    expected = float(pairs[index + 1])
    if not math.isclose(observed, expected, rel_tol=0.0, abs_tol=1e-9):
        raise SystemExit(
            f"probe {label}={observed} does not match live policy {expected}"
        )
PY

  /usr/bin/python3 - "${summary_echo}" "${summary_request_base}" <<'PY'
import re
import sys

path = sys.argv[1]
base_id = int(sys.argv[2])
with open(path, encoding="utf-8") as stream:
    text = stream.read()

seen = []
for block in re.split(r"\n---\s*\n", text):
    id_match = re.search(r"^\s*id:\s*(-?\d+)\s*$", block, re.MULTILINE)
    api_match = re.search(r"^\s*api_id:\s*(-?\d+)\s*$", block, re.MULTILINE)
    if not id_match or not api_match:
        continue
    request_id = int(id_match.group(1))
    api_id = int(api_match.group(1))
    if request_id > base_id:
        seen.append((request_id, api_id))

if not any(api_id == 1008 for _, api_id in seen):
    raise SystemExit(f"probe did not prove Move api_id=1008; seen={seen}")
if not any(api_id == 1003 for _, api_id in seen):
    raise SystemExit(f"probe did not prove StopMove api_id=1003; seen={seen}")
print(f"Validated live probe request ids: {seen}")
PY

  log "Validated matching live Sport probe: ${summary}"
}

assert_no_conflicting_runtime() {
  if ! "${SCRIPT_DIR}/check_go2_xt16_no_motion_runtime_clean.sh"; then
    die "Stop every existing Go2/DDDMR navigation runtime before --live."
  fi
}

source_host_go2_ros() {
  [[ -f "${GO2_SETUP}" ]] || die "Missing Go2 ROS setup: ${GO2_SETUP}"
  [[ -f "${SCRIPT_DIR}/setup_go2_dds_env.sh" ]] || die "Missing Go2 DDS setup script."
  unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
  set +u
  # shellcheck disable=SC1090
  source "${GO2_SETUP}"
  # shellcheck disable=SC1090
  source "${SCRIPT_DIR}/setup_go2_dds_env.sh"
  set -u
}

topic_info() {
  local topic="$1"
  timeout 10 ros2 topic info "${topic}" 2>&1
}

require_topic_type() {
  local topic="$1"
  local expected="$2"
  local info
  info="$(topic_info "${topic}")" || {
    printf '%s\n' "${info}" >&2
    die "Topic ${topic} is not visible."
  }
  printf '%s\n' "=== ${topic}" "${info}"
  [[ "${info}" == *"Type: ${expected}"* ]] || \
    die "Topic ${topic} is not ${expected}."
}

require_topic_sample() {
  local topic="$1"
  local type="$2"
  local output
  output="$(timeout 10 ros2 topic echo --no-daemon --spin-time 5 \
    "${topic}" "${type}" --once 2>&1)" || {
    printf '%s\n' "${output}" >&2
    die "Did not receive a read-only sample from ${topic}."
  }
  [[ -n "${output}" ]] || die "Received an empty sample from ${topic}."
  log "Received a read-only sample from ${topic}."
}

publisher_count() {
  local topic="$1"
  local info
  info="$(topic_info "${topic}")" || return 1
  awk '/^Publisher count:/ {print $3; found = 1} END {if (!found) exit 1}' <<<"${info}"
}

require_publisher_count() {
  local topic="$1"
  local expected="$2"
  local deadline=$((SECONDS + 15))
  local count=""
  while (( SECONDS < deadline )); do
    count="$(publisher_count "${topic}" 2>/dev/null || true)"
    if [[ "${count}" == "${expected}" ]]; then
      log "${topic} publisher count is ${expected}."
      return 0
    fi
    sleep 1
  done
  die "${topic} publisher count is '${count:-unknown}', expected ${expected}."
}

docker_ros() {
  local command="$1"
  docker exec "${container_name}" bash -lc "set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/${install_base}/setup.bash
set -u
${command}"
}

controller_status() {
  docker_ros \
    "timeout 6 ros2 topic echo --no-daemon --spin-time 3 /recorded_route_controller/status std_msgs/msg/String --once" \
    2>/dev/null
}

localization_status() {
  docker_ros \
    "timeout 6 ros2 topic echo --no-daemon --spin-time 3 /localization_status std_msgs/msg/String --once" \
    2>/dev/null
}

route_ready() {
  docker_ros \
    "timeout 6 ros2 topic echo --no-daemon --spin-time 3 /recorded_route_controller/route_ready std_msgs/msg/Bool --once" \
    2>/dev/null
}

route_progress() {
  docker_ros \
    "timeout 6 ros2 topic echo --no-daemon --spin-time 3 /recorded_route_controller/progress std_msgs/msg/Float64 --once" \
    2>/dev/null
}

start_route_source() {
  mkdir -p "${log_dir}"
  source_attempted="true"
  log "Starting recorded-route Docker source DISABLED: ${container_name}"
  ROUTE_FILE="${ROUTE_FILE_VALUE}" \
  ROUTE_MAP_DIR="${MAP_DIR}" \
  DDDMR_BAGS_DIR="${BAGS_DIR}" \
  NAV_CONFIG_FILE="${NAV_CONFIG_FILE_VALUE}" \
  RECORDED_ROUTE_CONTAINER_NAME="${container_name}" \
  RUN_LOG_DIR="${log_dir}" \
  RVIZ="${rviz}" \
  PUBLISH_STATIC_TF="${publish_static_tf}" \
    "${ROUTE_RUNNER}" --start 2>&1 | tee "${source_start_log}"
}

wait_for_source_readiness() {
  local deadline=$((SECONDS + 60))
  local status_output=""
  local localization_output=""
  local ready_output=""

  while (( SECONDS < deadline )); do
    docker inspect -f '{{.State.Running}}' "${container_name}" 2>/dev/null | \
      grep -Fxq true || die "Recorded-route Docker source stopped during readiness checks."
    status_output="$(controller_status || true)"
    localization_output="$(localization_status || true)"
    ready_output="$(route_ready || true)"
    if grep -Fq "READY: route ready:" <<<"${status_output}" && \
       grep -Eq "^[[:space:]]*data:[[:space:]]*['\"]?TRACKING['\"]?[[:space:]]*$" <<<"${localization_output}" && \
       grep -Fq "data: true" <<<"${ready_output}"; then
      printf '%s\n' "${status_output}" "${ready_output}" "${localization_output}"
      return 0
    fi
    sleep 1
  done

  printf '%s\n' "${status_output}" "${ready_output}" "${localization_output}" >&2
  die "Route source did not reach READY + route_ready=true + localization TRACKING."
}

check_source_shape() {
  local nodes service_type
  nodes="$(docker_ros "timeout 8 ros2 node list" 2>&1)" || {
    printf '%s\n' "${nodes}" >&2
    die "Could not inspect recorded-route ROS nodes."
  }
  if grep -Eq '(^|/)(p2p_move_base(_node)?|global_planner(_node)?|clicked2goal|go2_sport_cmd_vel_adapter)([^[:alnum:]_]|$)' <<<"${nodes}"; then
    printf '%s\n' "${nodes}" >&2
    die "Recorded-route source contains a forbidden P2P/global/live-adapter node."
  fi
  grep -Eq '(^|/)recorded_route_controller$' <<<"${nodes}" || \
    die "recorded_route_controller node is missing."

  service_type="$(docker_ros \
    "timeout 8 ros2 service type /recorded_route_controller/set_enabled" 2>&1)" || {
    printf '%s\n' "${service_type}" >&2
    die "Recorded-route enable service is missing."
  }
  grep -Fq 'std_srvs/srv/SetBool' <<<"${service_type}" || \
    die "Recorded-route enable service has the wrong type."
}

require_tf() {
  local parent="$1"
  local child="$2"
  local output
  output="$(docker_ros \
    "timeout 4 ros2 run tf2_ros tf2_echo '${parent}' '${child}' -p 6" 2>&1 || true)"
  if ! grep -Fq 'Translation:' <<<"${output}" || ! grep -Fq 'Rotation:' <<<"${output}"; then
    printf '%s\n' "${output}" >&2
    die "TF ${parent} -> ${child} is unavailable."
  fi
  log "TF ${parent} -> ${child} is available."
}

require_route_start_envelope() {
  local output current_pose current_x current_y current_z current_yaw
  output="$(docker_ros \
    "timeout 4 ros2 run tf2_ros tf2_echo map base_link -p 6" 2>&1 || true)"
  current_pose="$(/usr/bin/python3 - "${output}" <<'PY'
import re
import sys

text = sys.argv[1]
translation_match = re.search(r"- Translation: \[([^]]+)\]", text)
rpy_match = re.search(r"- Rotation: in RPY \(radian\) \[([^]]+)\]", text)
if not translation_match or not rpy_match:
    raise SystemExit("could not parse map -> base_link translation/RPY")
translation = [float(item.strip()) for item in translation_match.group(1).split(",")]
rpy = [float(item.strip()) for item in rpy_match.group(1).split(",")]
if len(translation) != 3 or len(rpy) != 3:
    raise SystemExit("map -> base_link transform has the wrong dimensions")
print(*(translation + [rpy[2]]))
PY
)" || {
    printf '%s\n' "${output}" >&2
    die "Could not read the current map -> base_link start pose."
  }
  read -r current_x current_y current_z current_yaw <<<"${current_pose}"

  if ! /usr/bin/python3 - \
    "${route_start_x}" "${route_start_y}" "${route_start_z}" "${route_start_yaw}" \
    "${current_x}" "${current_y}" "${current_z}" "${current_yaw}" \
    "${start_max_xy_error}" "${start_max_z_error}" "${start_max_yaw_error}" <<'PY'
import math
import sys

(
    route_x,
    route_y,
    route_z,
    route_yaw,
    current_x,
    current_y,
    current_z,
    current_yaw,
    max_xy,
    max_z,
    max_yaw,
) = map(float, sys.argv[1:])

xy_error = math.hypot(current_x - route_x, current_y - route_y)
z_error = abs(current_z - route_z)
yaw_error = abs(math.atan2(
    math.sin(current_yaw - route_yaw),
    math.cos(current_yaw - route_yaw),
))
print(
    "ROUTE_START_ERROR="
    f"xy:{xy_error:.3f}/{max_xy:.3f},"
    f"z:{z_error:.3f}/{max_z:.3f},"
    f"yaw:{yaw_error:.3f}/{max_yaw:.3f}"
)
failures = []
if xy_error > max_xy:
    failures.append("xy")
if z_error > max_z:
    failures.append("z")
if yaw_error > max_yaw:
    failures.append("yaw")
if failures:
    raise SystemExit("robot is outside the supervised route-start envelope: " + ",".join(failures))
PY
  then
    die "Robot is outside the supervised route-start envelope."
  fi
  log "Robot is inside the supervised route-start envelope."
}

start_request_echo() {
  log "Recording real Sport requests: ${request_echo_log}"
  ros2 topic echo "${REAL_REQUEST_TOPIC}" unitree_api/msg/Request \
    >"${request_echo_log}" 2>&1 &
  echo_pid="$!"
}

start_live_adapter() {
  log "Starting low-speed host Sport adapter: ${adapter_log}"
  live_output_attempted="true"
  /usr/bin/python3 "${ADAPTER}" \
    --ros-args \
    -p cmd_vel_topic:="${CMD_TOPIC}" \
    -p request_topic:="${REAL_REQUEST_TOPIC}" \
    -p enable_sport_output:=true \
    -p allow_real_request_topic:=true \
    -p axis_mode:="'standard'" \
    -p max_x:="${sport_max_x}" \
    -p max_y:="${sport_max_y}" \
    -p max_yaw:="${sport_max_yaw}" \
    -p publish_rate_hz:="${sport_publish_rate_hz}" \
    -p cmd_timeout_sec:="${sport_cmd_timeout_sec}" \
    -p zero_epsilon:="${sport_zero_epsilon}" \
    -p stop_keepalive_hz:="${sport_stop_keepalive_hz}" \
    -p stop_on_stale:=true \
    -p request_id_base:="${request_id_base}" \
    -p log_period_sec:=0.10 \
    -p enable_yaw_arc_shim:=false \
    -p yaw_arc_shim_mode:="'off'" \
    -p decision_topic:="${DECISION_TOPIC}" \
    -p decision_timeout_sec:="${decision_timeout_sec}" \
    -p require_motion_decision:=true \
    -p motion_allowed_decisions:="'${MOTION_ALLOWED_DECISIONS}'" \
    >"${adapter_log}" 2>&1 &
  adapter_pid="$!"
  sleep 1
  if ! kill -0 "${adapter_pid}" >/dev/null 2>&1; then
    cat "${adapter_log}" >&2 || true
    die "Host Sport adapter exited during startup."
  fi
}

prompt_for_route_arm() {
  [[ -t 0 && -t 1 ]] || \
    die "--live requires an interactive terminal for the final route arm phrase."

  printf '\n'
  printf 'LIVE RECORDED-ROUTE OUTPUT IS CONNECTED, BUT THE CONTROLLER IS DISABLED.\n'
  printf 'Route: %s (%s m)\n' "${route_id}" "${route_length_m}"
  printf 'Limits: x<=%s m/s, y=0, |yaw|<=%s rad/s, runtime<=%ss\n' \
    "${sport_max_x}" "${sport_max_yaw}" "${max_runtime_sec}"
  printf 'Start envelope: xy<=%sm, z<=%sm, |yaw|<=%srad\n' \
    "${start_max_xy_error}" "${start_max_z_error}" "${start_max_yaw_error}"
  printf 'Before arming, verify the Go2 is at the route start, the full corridor is clear,\n'
  printf 'the original remote/physical stop is held by a second person, and this terminal\n'
  printf 'has focus so Ctrl-C can be pressed immediately.\n'
  printf 'Type exactly: ENABLE %s\n' "${route_id}"

  local response
  if ! IFS= read -r -t "${arm_timeout_sec}" response; then
    die "Route arm phrase was not entered within ${arm_timeout_sec}s."
  fi
  [[ "${response}" == "ENABLE ${route_id}" ]] || die "Route arm phrase did not match."
}

enable_route_controller() {
  local status_output localization_output response
  status_output="$(controller_status)" || die "Could not re-read controller status before arming."
  localization_output="$(localization_status)" || die "Could not re-read localization before arming."
  grep -Fq "READY: route ready:" <<<"${status_output}" || {
    printf '%s\n' "${status_output}" >&2
    die "Controller is no longer READY."
  }
  grep -Eq "^[[:space:]]*data:[[:space:]]*['\"]?TRACKING['\"]?[[:space:]]*$" <<<"${localization_output}" || {
    printf '%s\n' "${localization_output}" >&2
    die "Localization is no longer TRACKING."
  }

  response="$(docker_ros \
    "ros2 service call /recorded_route_controller/set_enabled std_srvs/srv/SetBool '{data: true}'")"
  printf '%s\n' "${response}"
  grep -Eq 'success=(True|true)|success: true' <<<"${response}" || \
    die "Controller refused enable; verify the route-start envelope and localization."
  final_result="RUNNING"
  log "LIVE route '${route_id}' enabled. Press Ctrl-C to disable and StopMove."
}

status_state() {
  sed -nE "s/^[[:space:]]*data:[[:space:]]*['\"]?([A-Z_]+):.*$/\1/p" <<<"$1" | head -n 1
}

progress_value() {
  sed -nE 's/^[[:space:]]*data:[[:space:]]*([-+0-9.eE]+).*$/\1/p' <<<"$1" | head -n 1
}

monitor_route() {
  local start_seconds="${SECONDS}"
  local state_enter_seconds="${SECONDS}"
  local progress_mark_seconds="${SECONDS}"
  local graph_check_seconds="${SECONDS}"
  local last_state=""
  local last_progress="0.0"
  local status_output progress_output state progress advanced
  local request_publishers cmd_publishers decision_publishers

  while true; do
    kill -0 "${adapter_pid}" >/dev/null 2>&1 || die "Host Sport adapter stopped unexpectedly."
    docker inspect -f '{{.State.Running}}' "${container_name}" 2>/dev/null | \
      grep -Fxq true || die "Recorded-route Docker source stopped unexpectedly."

    status_output="$(controller_status)" || die "Controller status became unavailable."
    progress_output="$(route_progress)" || die "Controller progress became unavailable."
    state="$(status_state "${status_output}")"
    progress="$(progress_value "${progress_output}")"
    [[ -n "${state}" ]] || die "Could not parse controller state: ${status_output}"
    [[ -n "${progress}" ]] || die "Could not parse controller progress: ${progress_output}"

    if (( SECONDS - graph_check_seconds >= 5 )); then
      request_publishers="$(publisher_count "${REAL_REQUEST_TOPIC}" 2>/dev/null || true)"
      cmd_publishers="$(publisher_count "${CMD_TOPIC}" 2>/dev/null || true)"
      decision_publishers="$(publisher_count "${DECISION_TOPIC}" 2>/dev/null || true)"
      [[ "${request_publishers}" == "1" ]] || \
        die "Real Sport publisher count changed to '${request_publishers:-unknown}'."
      [[ "${cmd_publishers}" == "1" ]] || \
        die "Safe velocity publisher count changed to '${cmd_publishers:-unknown}'."
      [[ "${decision_publishers}" == "1" ]] || \
        die "Route decision publisher count changed to '${decision_publishers:-unknown}'."
      graph_check_seconds="${SECONDS}"
    fi

    if [[ "${state}" != "${last_state}" ]]; then
      log "Controller state=${state}, progress=${progress}"
      last_state="${state}"
      state_enter_seconds="${SECONDS}"
      if [[ "${state}" == "TRACKING" ]]; then
        last_progress="${progress}"
        progress_mark_seconds="${SECONDS}"
      fi
    fi

    case "${state}" in
      COMPLETED)
        final_result="COMPLETED"
        log "Recorded route completed."
        return 0
        ;;
      FAULT)
        printf '%s\n' "${status_output}" >&2
        final_result="CONTROLLER_FAULT"
        return 1
        ;;
      DISABLED|READY)
        printf '%s\n' "${status_output}" >&2
        final_result="CONTROLLER_DISABLED"
        return 1
        ;;
      ALIGNING_INITIAL_HEADING|ALIGNING_GOAL_HEADING)
        if (( SECONDS - state_enter_seconds > ${align_timeout_sec%.*} )); then
          die "Heading alignment exceeded ${align_timeout_sec}s."
        fi
        ;;
      TRACKING)
        advanced="$(awk -v current="${progress}" -v previous="${last_progress}" \
          'BEGIN {print (current >= previous + 0.002) ? "true" : "false"}')"
        if [[ "${advanced}" == "true" ]]; then
          last_progress="${progress}"
          progress_mark_seconds="${SECONDS}"
        elif (( SECONDS - progress_mark_seconds > ${stall_timeout_sec%.*} )); then
          die "Route progress stalled for more than ${stall_timeout_sec}s."
        fi
        ;;
      BLOCKED)
        # The controller publishes zero and enters FAULT after its own 5 s
        # blocked timeout if no safe corridor trajectory returns.
        ;;
      *)
        die "Unexpected active controller state: ${state}"
        ;;
    esac

    if (( SECONDS - start_seconds > ${max_runtime_sec%.*} )); then
      die "Route runtime exceeded ${max_runtime_sec}s."
    fi
    sleep 1
  done
}

disable_route_controller() {
  docker_ros \
    "timeout 8 ros2 service call /recorded_route_controller/set_enabled std_srvs/srv/SetBool '{data: false}'" \
    >/dev/null 2>&1 || true
}

publish_stopmove_burst() {
  local reason="$1"
  /usr/bin/python3 - "${REAL_REQUEST_TOPIC}" "$((request_id_base + 900000))" "${reason}" <<'PY'
import sys
import time

import rclpy
from unitree_api.msg import Request

topic = sys.argv[1]
base_id = int(sys.argv[2])
reason = sys.argv[3]

rclpy.init()
node = rclpy.create_node("go2_xt16_recorded_route_stopmove_cleanup")
publisher = node.create_publisher(Request, topic, 10)
time.sleep(0.2)
for index in range(3):
    request = Request()
    request.header.identity.id = base_id + index + 1
    request.header.identity.api_id = 1003
    request.parameter = ""
    publisher.publish(request)
    node.get_logger().warn(f"{reason}: published {topic} api_id=1003 StopMove")
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
}

write_summary() {
  cat >"${summary_log}" <<EOF
MODE=${mode}
RESULT=${final_result}
ROUTE_ID=${route_id}
ROUTE_LENGTH_3D_M=${route_length_m}
ROUTE_START_X=${route_start_x}
ROUTE_START_Y=${route_start_y}
ROUTE_START_Z=${route_start_z}
ROUTE_START_YAW=${route_start_yaw}
ROUTE_FILE=${ROUTE_FILE_VALUE}
ROUTE_MAP_DIR=${MAP_DIR}
PROBE_SUMMARY=${probe_summary}
ADAPTER_SHA256=$(sha256sum -- "${ADAPTER}" | awk '{print $1}')
REQUEST_ID_BASE=${request_id_base}
CONTAINER_NAME=${container_name}
SOURCE_START_LOG=${source_start_log}
DOCKER_LOG=${container_log}
ADAPTER_LOG=${adapter_log}
REQUEST_ECHO_LOG=${request_echo_log}
CMD_TOPIC=${CMD_TOPIC}
DECISION_TOPIC=${DECISION_TOPIC}
REQUEST_TOPIC=${REAL_REQUEST_TOPIC}
SPORT_MAX_X=${sport_max_x}
SPORT_MAX_Y=${sport_max_y}
SPORT_MAX_YAW=${sport_max_yaw}
SPORT_CMD_TIMEOUT_SEC=${sport_cmd_timeout_sec}
MAX_RUNTIME_SEC=${max_runtime_sec}
STALL_TIMEOUT_SEC=${stall_timeout_sec}
ALIGN_TIMEOUT_SEC=${align_timeout_sec}
ARM_TIMEOUT_SEC=${arm_timeout_sec}
START_MAX_XY_ERROR=${start_max_xy_error}
START_MAX_Z_ERROR=${start_max_z_error}
START_MAX_YAW_ERROR=${start_max_yaw_error}
EOF
  printf 'SUMMARY_LOG=%s\n' "${summary_log}"
}

cleanup() {
  local status=$?
  if [[ "${cleanup_started}" == "true" ]]; then
    return
  fi
  cleanup_started="true"
  set +e

  if docker inspect "${container_name}" >/dev/null 2>&1; then
    disable_route_controller
  fi
  if [[ "${live_output_attempted}" == "true" ]]; then
    publish_stopmove_burst "recorded-route supervisor cleanup" || true
  fi
  if [[ -n "${adapter_pid}" ]]; then
    kill -INT "${adapter_pid}" >/dev/null 2>&1 || true
    wait "${adapter_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${echo_pid}" ]]; then
    kill "${echo_pid}" >/dev/null 2>&1 || true
    wait "${echo_pid}" >/dev/null 2>&1 || true
  fi
  if docker inspect "${container_name}" >/dev/null 2>&1; then
    if docker inspect -f '{{.State.Running}}' "${container_name}" 2>/dev/null | grep -Fxq true; then
      docker stop -t 5 "${container_name}" >/dev/null 2>&1 || true
    fi
    docker logs "${container_name}" >"${container_log}" 2>&1 || true
    docker rm "${container_name}" >/dev/null 2>&1 || true
  fi

  if [[ "${mode}" == "live" && "${source_attempted}" == "true" ]]; then
    write_summary || true
  fi
  set -e
  return "${status}"
}

handle_signal() {
  final_result="INTERRUPTED"
  exit 130
}

validate_bool RVIZ "${rviz}"
validate_bool PUBLISH_STATIC_TF "${publish_static_tf}"
validate_low_speed_policy
validate_route_and_map
validate_fail_closed_source

if [[ "${mode}" == "check" ]]; then
  if [[ -n "${probe_summary}" ]]; then
    validate_probe_summary "${probe_summary}"
    probe_state="PASS"
  else
    probe_state="NOT_PROVIDED"
  fi
  printf 'ROUTE_ID=%s\n' "${route_id}"
  printf 'ROUTE_LENGTH_3D_M=%s\n' "${route_length_m}"
  printf 'ROUTE_START=x:%s,y:%s,z:%s,yaw:%s\n' \
    "${route_start_x}" "${route_start_y}" "${route_start_z}" "${route_start_yaw}"
  printf 'LIVE_LIMITS=max_x:%s,max_y:%s,max_yaw:%s\n' \
    "${sport_max_x}" "${sport_max_y}" "${sport_max_yaw}"
  printf 'START_ENVELOPE=max_xy:%s,max_z:%s,max_yaw:%s\n' \
    "${start_max_xy_error}" "${start_max_z_error}" "${start_max_yaw_error}"
  printf 'SOURCE_FAIL_CLOSED=PASS\n'
  printf 'PROBE_STATUS=%s\n' "${probe_state}"
  printf 'RESULT=GO2_XT16_RECORDED_ROUTE_OFFLINE_CHECK_PASS\n'
  exit 0
fi

[[ "${GO2_RECORDED_ROUTE_LIVE_CONFIRM:-}" == "${LIVE_CONFIRM_PHRASE}" ]] || \
  die "--live requires GO2_RECORDED_ROUTE_LIVE_CONFIRM=${LIVE_CONFIRM_PHRASE}"
[[ "${rviz}" == "true" ]] || die "--live requires RVIZ=true for route/corridor supervision."
validate_probe_summary "${probe_summary}"
assert_no_conflicting_runtime
source_host_go2_ros

printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-}"
printf 'CYCLONEDDS_URI=%s\n' "${CYCLONEDDS_URI:-}"

require_topic_type "${REAL_REQUEST_TOPIC}" "unitree_api/msg/Request"
require_topic_type "/sportmodestate" "unitree_go/msg/SportModeState"
require_topic_type "/lowstate" "unitree_go/msg/LowState"
require_topic_sample "/sportmodestate" "unitree_go/msg/SportModeState"
require_topic_sample "/lowstate" "unitree_go/msg/LowState"
require_publisher_count "${REAL_REQUEST_TOPIC}" 0

trap cleanup EXIT
trap handle_signal INT TERM

start_route_source
wait_for_source_readiness
check_source_shape
require_route_start_envelope
require_tf "base_link" "hesai_lidar"
require_topic_type "${CMD_TOPIC}" "geometry_msgs/msg/Twist"
require_topic_type "${DECISION_TOPIC}" "std_msgs/msg/String"
require_publisher_count "${CMD_TOPIC}" 1
require_publisher_count "${DECISION_TOPIC}" 1
require_publisher_count "${REAL_REQUEST_TOPIC}" 0

start_request_echo
start_live_adapter
require_publisher_count "${REAL_REQUEST_TOPIC}" 1
prompt_for_route_arm
enable_route_controller

if monitor_route; then
  exit 0
fi
exit 1
