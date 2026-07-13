#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/run_go2_sport_adapter_supervised_probe.sh [--dry-run|--typed-preview|--live]

Host-only supervised probe for the Go2 Sport adapter.

Modes:
  --dry-run        Default. Runs the adapter with sport output disabled.
  --typed-preview Runs the adapter against /dddmr_go2/sport_request_preview.
  --live          Publishes to /api/sport/request. Requires onsite supervision
                  and GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2.

Environment:
  GO2_SPORT_PROBE_X=0.05
  GO2_SPORT_PROBE_Y=0.0
  GO2_SPORT_PROBE_YAW=0.0
  GO2_SPORT_PROBE_DURATION=0.6
  GO2_SPORT_PROBE_DECISION=<optional d_align_heading>
  GO2_SPORT_PROBE_DECISION_PUB_COUNT=<auto>
  GO2_SPORT_PROBE_PRE_CMD_SLEEP=0.0
  GO2_SPORT_PROBE_LOG_DIR=/tmp
  GO2_SPORT_SKIP_LIVE_TOPIC_CHECK=false
  GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2
  GO2_SPORT_LIVE_ECHO_TIMEOUT=5
  GO2_SPORT_PUBLISH_RATE_HZ=50.0
  GO2_SPORT_CMD_TIMEOUT_SEC=0.20
  GO2_SPORT_MAX_X=0.10
  GO2_SPORT_MAX_Y=0.0
  GO2_SPORT_MAX_YAW=0.25
    # set GO2_SPORT_MAX_YAW=0.35 together with GO2_SPORT_PROBE_YAW=0.35
    # only for the supervised higher-yaw fallback probe
  GO2_SPORT_ZERO_EPSILON=0.001
  GO2_SPORT_STOP_KEEPALIVE_HZ=2.0
  GO2_ENABLE_YAW_ARC_SHIM=false
  GO2_YAW_ARC_SHIM_MODE=off        # off | preview | live
  GO2_YAW_ARC_SHIM_CONFIRM=I_AM_SUPERVISING_GO2_YAW_ARC_SHIM
  GO2_YAW_ARC_FORWARD_X=0.03
  GO2_YAW_ARC_MIN_ABS_YAW=0.20
  GO2_YAW_ARC_TRIGGER_ABS_YAW=0.03
  GO2_YAW_ARC_ALLOWED_DECISIONS=d_align_heading
  GO2_DECISION_TOPIC=/dddmr_go2/p2p_decision
  GO2_DECISION_TIMEOUT_SEC=0.30
  GO2_REQUIRE_MOTION_DECISION=false
  GO2_MOTION_ALLOWED_DECISIONS=d_controlling,d_align_heading,d_align_goal_heading,d_recovery_waitdone
  GO2_ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED=true
  GO2_MAX_CONTINUOUS_YAW_ARC_SEC=4.0

Live-mode hard caps:
  abs(x) <= 0.10
  abs(y) <= 0.05
  abs(yaw) <= 0.35
  duration <= 1.0 seconds normally
  duration <= 2.0 seconds only for pure-yaw probes (x=0, y=0)
EOF
}

mode="dry-run"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      mode="dry-run"
      shift
      ;;
    --typed-preview)
      mode="typed-preview"
      shift
      ;;
    --live)
      mode="live"
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
ADAPTER="${WS_ROOT}/src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py"
GO2_SETUP="${GO2_SETUP:-/home/lin/go2_workspace/unitree_ros2/setup.sh}"
CMD_TOPIC="${GO2_SPORT_CMD_TOPIC:-/dddmr_go2/safe_cmd_vel}"
REAL_REQUEST_TOPIC="/api/sport/request"
PREVIEW_REQUEST_TOPIC="/dddmr_go2/sport_request_preview"
CONFIRM_PHRASE="I_AM_SUPERVISING_GO2"
YAW_ARC_CONFIRM_PHRASE="I_AM_SUPERVISING_GO2_YAW_ARC_SHIM"

probe_x="${GO2_SPORT_PROBE_X:-0.05}"
probe_y="${GO2_SPORT_PROBE_Y:-0.0}"
probe_yaw="${GO2_SPORT_PROBE_YAW:-0.0}"
probe_duration="${GO2_SPORT_PROBE_DURATION:-0.6}"
probe_decision="${GO2_SPORT_PROBE_DECISION:-}"
probe_decision_pub_count="${GO2_SPORT_PROBE_DECISION_PUB_COUNT:-}"
probe_pre_cmd_sleep="${GO2_SPORT_PROBE_PRE_CMD_SLEEP:-0.0}"
publish_rate_hz="${GO2_SPORT_PUBLISH_RATE_HZ:-50.0}"
cmd_timeout_sec="${GO2_SPORT_CMD_TIMEOUT_SEC:-0.20}"
max_x="${GO2_SPORT_MAX_X:-0.10}"
max_y="${GO2_SPORT_MAX_Y:-0.0}"
max_yaw="${GO2_SPORT_MAX_YAW:-0.25}"
zero_epsilon="${GO2_SPORT_ZERO_EPSILON:-0.001}"
stop_keepalive_hz="${GO2_SPORT_STOP_KEEPALIVE_HZ:-2.0}"
enable_yaw_arc_shim="${GO2_ENABLE_YAW_ARC_SHIM:-false}"
yaw_arc_shim_mode="${GO2_YAW_ARC_SHIM_MODE:-off}"
yaw_arc_forward_x="${GO2_YAW_ARC_FORWARD_X:-0.03}"
yaw_arc_min_abs_yaw="${GO2_YAW_ARC_MIN_ABS_YAW:-0.20}"
yaw_arc_trigger_abs_yaw="${GO2_YAW_ARC_TRIGGER_ABS_YAW:-0.03}"
yaw_arc_allowed_decisions="${GO2_YAW_ARC_ALLOWED_DECISIONS:-d_align_heading}"
decision_topic="${GO2_DECISION_TOPIC:-/dddmr_go2/p2p_decision}"
decision_timeout_sec="${GO2_DECISION_TIMEOUT_SEC:-0.30}"
require_motion_decision="${GO2_REQUIRE_MOTION_DECISION:-false}"
motion_allowed_decisions="${GO2_MOTION_ALLOWED_DECISIONS:-d_controlling,d_align_heading,d_align_goal_heading,d_recovery_waitdone}"
zero_yaw_only_when_shim_disallowed="${GO2_ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED:-true}"
max_continuous_yaw_arc_sec="${GO2_MAX_CONTINUOUS_YAW_ARC_SEC:-4.0}"
skip_live_topic_check="${GO2_SPORT_SKIP_LIVE_TOPIC_CHECK:-false}"
live_echo_timeout="${GO2_SPORT_LIVE_ECHO_TIMEOUT:-5}"
log_dir="${GO2_SPORT_PROBE_LOG_DIR:-/tmp}"
stamp="$(date +%Y%m%d_%H%M%S)"
request_id_base="$(date +%s)"
adapter_log="${log_dir}/go2_sport_adapter_${mode}_${stamp}.log"
preview_echo_log="${log_dir}/go2_sport_adapter_${mode}_${stamp}_echo.log"
pub_log="${log_dir}/go2_sport_adapter_${mode}_${stamp}_pub.log"
decision_pub_log="${log_dir}/go2_sport_adapter_${mode}_${stamp}_decision_pub.log"
summary_log="${log_dir}/go2_sport_adapter_${mode}_${stamp}_summary.env"

adapter_pid=""
echo_pid=""
decision_pid=""

cleanup() {
  if [[ -n "${decision_pid}" ]]; then
    kill "${decision_pid}" >/dev/null 2>&1 || true
    wait "${decision_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${echo_pid}" ]]; then
    kill "${echo_pid}" >/dev/null 2>&1 || true
    wait "${echo_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${adapter_pid}" ]]; then
    kill "${adapter_pid}" >/dev/null 2>&1 || true
    wait "${adapter_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

die() {
  echo "ERROR: $*" >&2
  exit 1
}

validate_probe_bounds() {
  /usr/bin/python3 - \
    "$mode" \
    "$probe_x" \
    "$probe_y" \
    "$probe_yaw" \
    "$probe_duration" \
    "$max_x" \
    "$max_y" \
    "$max_yaw" <<'PY'
import math
import sys

mode, x_raw, y_raw, yaw_raw, duration_raw, max_x_raw, max_y_raw, max_yaw_raw = sys.argv[1:]
try:
    x = float(x_raw)
    y = float(y_raw)
    yaw = float(yaw_raw)
    duration = float(duration_raw)
    max_x = float(max_x_raw)
    max_y = float(max_y_raw)
    max_yaw = float(max_yaw_raw)
except ValueError as exc:
    raise SystemExit(f"invalid numeric probe parameter: {exc}")

values = [x, y, yaw, duration, max_x, max_y, max_yaw]
if not all(math.isfinite(v) for v in values):
    raise SystemExit("probe parameters must be finite")
if duration <= 0.0:
    raise SystemExit("GO2_SPORT_PROBE_DURATION must be > 0")
if max_x < 0.0 or max_y < 0.0 or max_yaw < 0.0:
    raise SystemExit("GO2_SPORT_MAX_X/Y/YAW must be non-negative")
if abs(x) > max_x:
    raise SystemExit(f"probe x={x} exceeds adapter GO2_SPORT_MAX_X={max_x}")
if abs(y) > max_y:
    raise SystemExit(f"probe y={y} exceeds adapter GO2_SPORT_MAX_Y={max_y}")
if abs(yaw) > max_yaw:
    raise SystemExit(f"probe yaw={yaw} exceeds adapter GO2_SPORT_MAX_YAW={max_yaw}")

if mode == "live":
    limits = {
        "x": (x, 0.10),
        "y": (y, 0.05),
        "yaw": (yaw, 0.35),
        "max_x": (max_x, 0.10),
        "max_y": (max_y, 0.05),
        "max_yaw": (max_yaw, 0.35),
    }
    for name, (value, limit) in limits.items():
        if abs(value) > limit:
            raise SystemExit(f"live {name}={value} exceeds hard cap {limit}")

    pure_yaw = abs(x) <= 1e-9 and abs(y) <= 1e-9 and abs(yaw) > 0.0
    duration_limit = 2.0 if pure_yaw else 1.0
    if duration > duration_limit:
        raise SystemExit(
            f"live duration={duration} exceeds hard cap {duration_limit}; "
            "2.0s is only allowed for pure-yaw probes"
        )
PY

  /usr/bin/python3 - "$probe_pre_cmd_sleep" <<'PY'
import math
import sys

try:
    value = float(sys.argv[1])
except ValueError as exc:
    raise SystemExit(f"GO2_SPORT_PROBE_PRE_CMD_SLEEP must be numeric: {exc}")
if not math.isfinite(value) or value < 0.0 or value > 5.0:
    raise SystemExit("GO2_SPORT_PROBE_PRE_CMD_SLEEP must be within [0.0, 5.0]")
PY

  if [[ -n "${probe_decision_pub_count}" ]]; then
    [[ "${probe_decision_pub_count}" =~ ^[1-9][0-9]*$ ]] || \
      die "GO2_SPORT_PROBE_DECISION_PUB_COUNT must be a positive integer"
  fi
}

validate_yaw_arc_settings() {
  case "${enable_yaw_arc_shim}" in
    true|false) ;;
    *) die "GO2_ENABLE_YAW_ARC_SHIM must be true or false" ;;
  esac
  case "${yaw_arc_shim_mode}" in
    off|preview|live) ;;
    *) die "GO2_YAW_ARC_SHIM_MODE must be off, preview, or live" ;;
  esac
  case "${zero_yaw_only_when_shim_disallowed}" in
    true|false) ;;
    *) die "GO2_ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED must be true or false" ;;
  esac
  case "${require_motion_decision}" in
    true|false) ;;
    *) die "GO2_REQUIRE_MOTION_DECISION must be true or false" ;;
  esac
  validate_allowed_decisions "${motion_allowed_decisions}"

  /usr/bin/python3 - \
    "${yaw_arc_forward_x}" \
    "${yaw_arc_min_abs_yaw}" \
    "${yaw_arc_trigger_abs_yaw}" \
    "${decision_timeout_sec}" \
    "${max_continuous_yaw_arc_sec}" <<'PY'
import math
import sys

names = [
    "GO2_YAW_ARC_FORWARD_X",
    "GO2_YAW_ARC_MIN_ABS_YAW",
    "GO2_YAW_ARC_TRIGGER_ABS_YAW",
    "GO2_DECISION_TIMEOUT_SEC",
    "GO2_MAX_CONTINUOUS_YAW_ARC_SEC",
]
values = []
for name, raw in zip(names, sys.argv[1:]):
    try:
        value = float(raw)
    except ValueError as exc:
        raise SystemExit(f"{name} must be numeric: {exc}")
    if not math.isfinite(value):
        raise SystemExit(f"{name} must be finite")
    values.append(value)

if values[0] < 0.0 or values[0] > 0.05:
    raise SystemExit("GO2_YAW_ARC_FORWARD_X must be within [0.0, 0.05]")
if values[1] < 0.0 or values[1] > 0.30:
    raise SystemExit("GO2_YAW_ARC_MIN_ABS_YAW must be within [0.0, 0.30]")
if values[2] < 0.0 or values[2] > 0.30:
    raise SystemExit("GO2_YAW_ARC_TRIGGER_ABS_YAW must be within [0.0, 0.30]")
if values[3] <= 0.0 or values[3] > 2.0:
    raise SystemExit("GO2_DECISION_TIMEOUT_SEC must be within (0.0, 2.0]")
if values[4] <= 0.0 or values[4] > 5.0:
    raise SystemExit("GO2_MAX_CONTINUOUS_YAW_ARC_SEC must be within (0.0, 5.0]")
PY

  if [[ "${enable_yaw_arc_shim}" == "true" && "${yaw_arc_shim_mode}" == "live" ]]; then
    [[ "${GO2_YAW_ARC_SHIM_CONFIRM:-}" == "${YAW_ARC_CONFIRM_PHRASE}" ]] || \
      die "live yaw-arc shim requires GO2_YAW_ARC_SHIM_CONFIRM=${YAW_ARC_CONFIRM_PHRASE}"
  fi
  validate_allowed_decisions "${yaw_arc_allowed_decisions}"
}

validate_allowed_decisions() {
  local raw="$1"
  [[ -n "${raw}" ]] || die "GO2_YAW_ARC_ALLOWED_DECISIONS must not be empty"
  local token
  IFS=',' read -r -a tokens <<<"${raw}"
  for token in "${tokens[@]}"; do
    [[ -n "${token}" ]] || die "GO2_YAW_ARC_ALLOWED_DECISIONS contains an empty entry"
    [[ "${token}" =~ ^[A-Za-z0-9_]+$ ]] || \
      die "GO2_YAW_ARC_ALLOWED_DECISIONS entry is invalid: ${token}"
  done
}

assert_no_conflicting_runtime() {
  local docker_matches
  docker_matches="$(docker ps --format '{{.Names}} {{.Image}}' 2>/dev/null | \
    rg 'go2_xt16|dddmr_go2_xt16|dddmr_navigation' || true)"
  if [[ -n "${docker_matches}" ]]; then
    echo "${docker_matches}" >&2
    die "stop Go2/DDDMR Docker containers before running the supervised probe"
  fi

  local proc_matches
  proc_matches="$(ps -eo pid,args | \
    rg 'go2_sport_cmd_vel_adapter|go2_sport_cmd_vel_dry_run|go2_xt16_navigation.launch|rviz2|p2p_move_base_node' | \
    rg -v 'run_go2_sport_adapter_supervised_probe|bash -lc|py_compile|sed -n|nl -ba|rg |ps -eo' || true)"
  if [[ -n "${proc_matches}" ]]; then
    echo "${proc_matches}" >&2
    die "stop existing navigation/RViz/Sport adapter processes before running the probe"
  fi
}

source_go2_ros() {
  [[ -f "${GO2_SETUP}" ]] || die "missing Go2 ROS setup: ${GO2_SETUP}"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
  set +u
  # shellcheck disable=SC1090
  source "${GO2_SETUP}"
  # shellcheck disable=SC1090
  source "${WS_ROOT}/scripts/setup_go2_dds_env.sh"
  set -u
}

check_topic_type() {
  local topic="$1"
  local expected="$2"
  local info
  info="$(timeout 10 ros2 topic info "${topic}" 2>&1)" || {
    echo "${info}" >&2
    die "topic ${topic} is not visible"
  }
  echo "=== ${topic}"
  echo "${info}"
  [[ "${info}" == *"Type: ${expected}"* ]] || die "${topic} type is not ${expected}"
}

publish_once() {
  local x="$1"
  local y="$2"
  local yaw="$3"
  ros2 topic pub --once "${CMD_TOPIC}" geometry_msgs/msg/Twist \
    "{linear: {x: ${x}, y: ${y}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${yaw}}}"
}

start_decision_publisher() {
  if [[ -z "${probe_decision}" ]]; then
    return
  fi
  local decision_pub_count
  if [[ -n "${probe_decision_pub_count}" ]]; then
    decision_pub_count="${probe_decision_pub_count}"
  else
    decision_pub_count="$(compute_decision_pub_count)"
  fi
  ros2 topic pub --rate 20 --times "${decision_pub_count}" "${decision_topic}" std_msgs/msg/String \
    "{data: '${probe_decision}'}" >"${decision_pub_log}" 2>&1 &
  decision_pid="$!"
  sleep 0.4
}

compute_decision_pub_count() {
  /usr/bin/python3 - "${probe_duration}" <<'PY'
import math
import sys

duration = float(sys.argv[1])
print(max(20, math.ceil((duration + 1.5) * 20.0)))
PY
}

compute_live_pub_count() {
  /usr/bin/python3 - "${probe_duration}" <<'PY'
import math
import sys

duration = float(sys.argv[1])
print(max(2, math.ceil(duration * 10.0)))
PY
}

start_adapter() {
  local request_topic="$1"
  local enable_output="$2"
  local allow_real="$3"

  /usr/bin/python3 "${ADAPTER}" \
    --ros-args \
    -p cmd_vel_topic:="${CMD_TOPIC}" \
    -p request_topic:="${request_topic}" \
    -p enable_sport_output:="${enable_output}" \
    -p allow_real_request_topic:="${allow_real}" \
    -p max_x:="${max_x}" \
    -p max_y:="${max_y}" \
    -p max_yaw:="${max_yaw}" \
    -p publish_rate_hz:="${publish_rate_hz}" \
    -p cmd_timeout_sec:="${cmd_timeout_sec}" \
    -p zero_epsilon:="${zero_epsilon}" \
    -p stop_keepalive_hz:="${stop_keepalive_hz}" \
    -p request_id_base:="${request_id_base}" \
    -p log_period_sec:=0.10 \
    -p enable_yaw_arc_shim:="${enable_yaw_arc_shim}" \
    -p yaw_arc_shim_mode:="'${yaw_arc_shim_mode}'" \
    -p yaw_arc_forward_x:="${yaw_arc_forward_x}" \
    -p yaw_arc_min_abs_yaw:="${yaw_arc_min_abs_yaw}" \
    -p yaw_arc_trigger_abs_yaw:="${yaw_arc_trigger_abs_yaw}" \
    -p yaw_arc_allowed_decisions:="'${yaw_arc_allowed_decisions}'" \
    -p decision_topic:="${decision_topic}" \
    -p decision_timeout_sec:="${decision_timeout_sec}" \
    -p require_motion_decision:="${require_motion_decision}" \
    -p motion_allowed_decisions:="'${motion_allowed_decisions}'" \
    -p zero_yaw_only_when_shim_disallowed:="${zero_yaw_only_when_shim_disallowed}" \
    -p max_continuous_yaw_arc_sec:="${max_continuous_yaw_arc_sec}" \
    >"${adapter_log}" 2>&1 &
  adapter_pid="$!"
  sleep 1.0
  ps -p "${adapter_pid}" >/dev/null || {
    cat "${adapter_log}" >&2 || true
    die "adapter exited during startup"
  }
}

print_node_info() {
  echo "=== /go2_sport_cmd_vel_adapter"
  timeout 10 ros2 node info /go2_sport_cmd_vel_adapter || true
}

sleep_before_cmd_if_requested() {
  if /usr/bin/python3 - "${probe_pre_cmd_sleep}" <<'PY'
import sys

value = float(sys.argv[1])
raise SystemExit(0 if value > 0.0 else 1)
PY
  then
    sleep "${probe_pre_cmd_sleep}"
  fi
}

validate_request_echo_for_probe() {
  local echo_log="$1"
  local base_id="$2"

  /usr/bin/python3 - "${echo_log}" "${base_id}" <<'PY'
import re
import sys

path = sys.argv[1]
base_id = int(sys.argv[2])

try:
    text = open(path, encoding="utf-8").read()
except FileNotFoundError:
    raise SystemExit(f"missing echo log: {path}")

seen = []
for block in re.split(r"\n---\s*\n", text):
    id_match = re.search(r"^\s*id:\s*(-?\d+)\s*$", block, re.MULTILINE)
    api_match = re.search(r"^\s*api_id:\s*(-?\d+)\s*$", block, re.MULTILINE)
    if not id_match or not api_match:
        continue
    req_id = int(id_match.group(1))
    api_id = int(api_match.group(1))
    if req_id > base_id:
        seen.append((req_id, api_id))

if not any(api_id == 1008 for _, api_id in seen):
    raise SystemExit(f"probe id>{base_id} did not observe Move api_id=1008; seen={seen}")
if not any(api_id == 1003 for _, api_id in seen):
    raise SystemExit(f"probe id>{base_id} did not observe StopMove api_id=1003; seen={seen}")

print(f"validated probe request ids: {seen}")
PY
}

validate_probe_bounds
validate_yaw_arc_settings
assert_no_conflicting_runtime

case "${skip_live_topic_check}" in
  true|false) ;;
  *) die "GO2_SPORT_SKIP_LIVE_TOPIC_CHECK must be true or false" ;;
esac

if [[ "${mode}" == "live" && "${GO2_SPORT_LIVE_CONFIRM:-}" != "${CONFIRM_PHRASE}" ]]; then
  die "live mode requires GO2_SPORT_LIVE_CONFIRM=${CONFIRM_PHRASE}"
fi
if [[ "${mode}" == "live" && "${skip_live_topic_check}" == "true" ]]; then
  die "live mode cannot skip live topic checks"
fi

source_go2_ros

echo "RESULT_DIR=${log_dir}"
echo "ADAPTER_LOG=${adapter_log}"
echo "PUB_LOG=${pub_log}"
echo "DECISION_PUB_LOG=${decision_pub_log}"
echo "SUMMARY_LOG=${summary_log}"
echo "MODE=${mode}"
echo "PROBE x=${probe_x} y=${probe_y} yaw=${probe_yaw} duration=${probe_duration} decision=${probe_decision:-<none>} pre_cmd_sleep=${probe_pre_cmd_sleep}"
echo "CMD_TOPIC=${CMD_TOPIC}"
echo "ADAPTER_TIMER publish_rate_hz=${publish_rate_hz} cmd_timeout_sec=${cmd_timeout_sec} max_x=${max_x} max_y=${max_y} max_yaw=${max_yaw} zero_epsilon=${zero_epsilon} stop_keepalive_hz=${stop_keepalive_hz}"
echo "REQUEST_ID_BASE=${request_id_base}"
echo "YAW_ARC_SHIM enabled=${enable_yaw_arc_shim} mode=${yaw_arc_shim_mode} allowed_decisions=${yaw_arc_allowed_decisions}"
echo "SKIP_LIVE_TOPIC_CHECK=${skip_live_topic_check}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"

if [[ "${skip_live_topic_check}" == "true" ]]; then
  echo "Skipping live Go2 topic checks for no-motion ${mode} probe."
else
  check_topic_type "${REAL_REQUEST_TOPIC}" "unitree_api/msg/Request"
  check_topic_type "/sportmodestate" "unitree_go/msg/SportModeState"
  check_topic_type "/lowstate" "unitree_go/msg/LowState"
fi

case "${mode}" in
  dry-run)
    start_adapter "${REAL_REQUEST_TOPIC}" "false" "false"
    start_decision_publisher
    sleep_before_cmd_if_requested
    publish_once "${probe_x}" "${probe_y}" "${probe_yaw}" >"${pub_log}" 2>&1 || true
    sleep 0.8
    print_node_info
    ;;

  typed-preview)
    start_adapter "${PREVIEW_REQUEST_TOPIC}" "true" "false"
    start_decision_publisher
    ros2 topic echo --once "${PREVIEW_REQUEST_TOPIC}" unitree_api/msg/Request \
      >"${preview_echo_log}" 2>&1 &
    echo_pid="$!"
    sleep 0.8
    sleep_before_cmd_if_requested
    publish_once "${probe_x}" "${probe_y}" "${probe_yaw}" >"${pub_log}" 2>&1 || true
    wait "${echo_pid}" || true
    echo_pid=""
    sleep 0.8
    print_node_info
    echo "PREVIEW_ECHO_LOG=${preview_echo_log}"
    ;;

  live)
    echo "LIVE MODE CAN MOVE THE GO2 THROUGH ${REAL_REQUEST_TOPIC}."
    echo "Onsite supervision confirmation accepted."
    start_adapter "${REAL_REQUEST_TOPIC}" "true" "true"
    start_decision_publisher
    live_pub_count="$(compute_live_pub_count)"
    timeout -s INT -k 1s "${live_echo_timeout}s" \
      ros2 topic echo "${REAL_REQUEST_TOPIC}" unitree_api/msg/Request \
      >"${preview_echo_log}" 2>&1 &
    echo_pid="$!"
    sleep 0.8
    sleep_before_cmd_if_requested
    timeout -s INT -k 1s 4s \
      ros2 topic pub --rate 10 --times "${live_pub_count}" "${CMD_TOPIC}" geometry_msgs/msg/Twist \
      "{linear: {x: ${probe_x}, y: ${probe_y}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${probe_yaw}}}" \
      >"${pub_log}" 2>&1 || true
    publish_once 0.0 0.0 0.0 >>"${pub_log}" 2>&1 || true
    wait "${echo_pid}" || true
    echo_pid=""
    validate_request_echo_for_probe "${preview_echo_log}" "${request_id_base}"
    sleep 0.8
    print_node_info
    echo "LIVE_REQUEST_ECHO_LOG=${preview_echo_log}"
    ;;
esac

echo "=== adapter log"
cat "${adapter_log}"
echo "=== publish log"
cat "${pub_log}" || true
if [[ -f "${preview_echo_log}" ]]; then
  echo "=== preview echo log"
  cat "${preview_echo_log}"
fi
if [[ -f "${decision_pub_log}" ]]; then
  echo "=== decision publish log"
  cat "${decision_pub_log}"
fi

result="GO2_SPORT_ADAPTER_${mode//-/_}_COMPLETE"
cat >"${summary_log}" <<EOF
MODE=${mode}
RESULT=${result}
REQUEST_ID_BASE=${request_id_base}
ADAPTER_LOG=${adapter_log}
PUB_LOG=${pub_log}
DECISION_PUB_LOG=${decision_pub_log}
REQUEST_ECHO_LOG=${preview_echo_log}
CMD_TOPIC=${CMD_TOPIC}
REQUEST_TOPIC=$([[ "${mode}" == "typed-preview" ]] && printf '%s' "${PREVIEW_REQUEST_TOPIC}" || printf '%s' "${REAL_REQUEST_TOPIC}")
PROBE_DECISION=${probe_decision}
PROBE_DECISION_PUB_COUNT=${probe_decision_pub_count}
PROBE_PRE_CMD_SLEEP=${probe_pre_cmd_sleep}
PUBLISH_RATE_HZ=${publish_rate_hz}
CMD_TIMEOUT_SEC=${cmd_timeout_sec}
MAX_X=${max_x}
MAX_Y=${max_y}
MAX_YAW=${max_yaw}
ZERO_EPSILON=${zero_epsilon}
STOP_KEEPALIVE_HZ=${stop_keepalive_hz}
ENABLE_YAW_ARC_SHIM=${enable_yaw_arc_shim}
YAW_ARC_SHIM_MODE=${yaw_arc_shim_mode}
YAW_ARC_FORWARD_X=${yaw_arc_forward_x}
YAW_ARC_MIN_ABS_YAW=${yaw_arc_min_abs_yaw}
YAW_ARC_TRIGGER_ABS_YAW=${yaw_arc_trigger_abs_yaw}
YAW_ARC_ALLOWED_DECISIONS=${yaw_arc_allowed_decisions}
DECISION_TOPIC=${decision_topic}
DECISION_TIMEOUT_SEC=${decision_timeout_sec}
ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED=${zero_yaw_only_when_shim_disallowed}
REQUIRE_MOTION_DECISION=${require_motion_decision}
MOTION_ALLOWED_DECISIONS=${motion_allowed_decisions}
MAX_CONTINUOUS_YAW_ARC_SEC=${max_continuous_yaw_arc_sec}
SKIP_LIVE_TOPIC_CHECK=${skip_live_topic_check}
EOF

echo "SUMMARY_LOG=${summary_log}"
echo "RESULT: ${result}"
