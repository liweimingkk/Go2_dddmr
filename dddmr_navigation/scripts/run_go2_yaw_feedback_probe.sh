#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/run_go2_yaw_feedback_probe.sh --live

Short supervised Go2 velocity/yaw feedback probe.

This starts Docker navigation-live-source only to expose
/dddmr_go2/robot_odom_standard, then publishes a short Sport Move to
/api/sport/request and records:
  - /dddmr_go2/robot_odom_standard yaw
  - /sportmodestate imu_state.rpy yaw and yaw_speed
  - /api/sport/request echo

Live mode can move the Go2. It requires:
  GO2_YAW_PROBE_CONFIRM=I_AM_SUPERVISING_GO2_YAW_PROBE

Environment:
  GO2_YAW_PROBE_X=0.0
  GO2_YAW_PROBE_Y=0.0
  GO2_YAW_PROBE_YAW=0.25
  GO2_YAW_PROBE_DURATION=0.6
  GO2_YAW_PROBE_PRE_BALANCE_STAND=false
  GO2_YAW_PROBE_PRE_BALANCE_DELAY=0.8
  GO2_YAW_PROBE_LOG_DIR=/tmp
  GO2_YAW_PROBE_ANALYZE=true
  GO2_SETUP=/home/lin/go2_workspace/unitree_ros2/setup.sh
  RVIZ=false
  PUBLISH_STATIC_TF=true

Live-mode hard caps:
  abs(x) <= 0.05 m/s
  abs(y) <= 0.05 m/s
  abs(yaw) <= 0.30 rad/s
  duration <= 0.8 seconds
EOF
}

mode=""
while [[ $# -gt 0 ]]; do
  case "$1" in
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

[[ "${mode}" == "live" ]] || {
  usage >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GO2_SETUP="${GO2_SETUP:-/home/lin/go2_workspace/unitree_ros2/setup.sh}"
DOCKER_WRAPPER="${WS_ROOT}/scripts/dddmr_docker_go2_xt16.sh"
ANALYZER="${WS_ROOT}/scripts/analyze_go2_yaw_probe_result.py"
CONFIRM_PHRASE="I_AM_SUPERVISING_GO2_YAW_PROBE"
REAL_REQUEST_TOPIC="/api/sport/request"
ODOM_TOPIC="/dddmr_go2/robot_odom_standard"
SPORT_TOPIC="/sportmodestate"
LOWSTATE_TOPIC="/lowstate"

probe_x="${GO2_YAW_PROBE_X:-0.0}"
probe_y="${GO2_YAW_PROBE_Y:-0.0}"
probe_yaw="${GO2_YAW_PROBE_YAW:-0.25}"
probe_duration="${GO2_YAW_PROBE_DURATION:-0.6}"
pre_balance_stand="${GO2_YAW_PROBE_PRE_BALANCE_STAND:-false}"
pre_balance_delay="${GO2_YAW_PROBE_PRE_BALANCE_DELAY:-0.8}"
analyze_probe="${GO2_YAW_PROBE_ANALYZE:-true}"
log_dir="${GO2_YAW_PROBE_LOG_DIR:-/tmp}"
stamp="$(date +%Y%m%d_%H%M%S)"
request_id_base="$(date +%s)"
docker_name="go2_yaw_probe_${stamp}"
docker_log="${log_dir}/go2_yaw_probe_${stamp}_docker.log"
request_echo_log="${log_dir}/go2_yaw_probe_${stamp}_request_echo.log"
probe_log="${log_dir}/go2_yaw_probe_${stamp}.log"
odom_csv="${log_dir}/go2_yaw_probe_${stamp}_odom.csv"
sport_csv="${log_dir}/go2_yaw_probe_${stamp}_sport.csv"
request_csv="${log_dir}/go2_yaw_probe_${stamp}_requests.csv"
summary_log="${log_dir}/go2_yaw_probe_${stamp}_summary.env"
analysis_log="${log_dir}/go2_yaw_probe_${stamp}_analysis.env"
analysis_output_log="${log_dir}/go2_yaw_probe_${stamp}_analysis.log"
docker_pid=""
echo_pid=""
cleanup_started="false"
live_runtime_started="false"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

source_host_go2_ros() {
  [[ -f "${GO2_SETUP}" ]] || die "missing Go2 ROS setup: ${GO2_SETUP}"
  [[ -f "${WS_ROOT}/scripts/setup_go2_dds_env.sh" ]] || die "missing DDS env script"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
  set +u
  # shellcheck disable=SC1090
  source "${GO2_SETUP}"
  # shellcheck disable=SC1090
  source "${WS_ROOT}/scripts/setup_go2_dds_env.sh"
  set -u
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
node = rclpy.create_node("go2_yaw_probe_stopmove_cleanup")
pub = node.create_publisher(Request, topic, 10)
time.sleep(0.2)
for i in range(3):
    req = Request()
    req.header.identity.id = base_id + i + 1
    req.header.identity.api_id = 1003
    req.parameter = ""
    pub.publish(req)
    node.get_logger().warn(f"{reason}: published {topic} api_id=1003 StopMove")
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
}

cleanup() {
  local status=$?
  if [[ "${cleanup_started}" == "true" ]]; then
    exit "${status}"
  fi
  cleanup_started="true"

  if [[ "${live_runtime_started}" == "true" ]]; then
    publish_stopmove_burst "yaw probe cleanup" || true
  fi

  if [[ -n "${echo_pid}" ]]; then
    kill "${echo_pid}" >/dev/null 2>&1 || true
    wait "${echo_pid}" >/dev/null 2>&1 || true
  fi
  if docker ps --format '{{.Names}}' 2>/dev/null | rg -qx "${docker_name}"; then
    docker stop "${docker_name}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${docker_pid}" ]]; then
    wait "${docker_pid}" >/dev/null 2>&1 || true
  fi
  exit "${status}"
}
trap cleanup EXIT INT TERM

validate_probe_bounds() {
  /usr/bin/python3 - "${probe_x}" "${probe_y}" "${probe_yaw}" "${probe_duration}" "${pre_balance_delay}" <<'PY'
import math
import sys

x = float(sys.argv[1])
y = float(sys.argv[2])
yaw = float(sys.argv[3])
duration = float(sys.argv[4])
balance_delay = float(sys.argv[5])
if not all(math.isfinite(v) for v in (x, y, yaw, duration, balance_delay)):
    raise SystemExit("probe x, y, yaw, duration, and balance delay must be finite")
if abs(x) > 0.05:
    raise SystemExit(f"live x={x} exceeds hard cap 0.05")
if abs(y) > 0.05:
    raise SystemExit(f"live y={y} exceeds hard cap 0.05")
if abs(yaw) > 0.30:
    raise SystemExit(f"live yaw={yaw} exceeds hard cap 0.30")
if duration <= 0.0 or duration > 0.8:
    raise SystemExit("GO2_YAW_PROBE_DURATION must be > 0 and <= 0.8")
if balance_delay < 0.0 or balance_delay > 2.0:
    raise SystemExit("GO2_YAW_PROBE_PRE_BALANCE_DELAY must be >= 0 and <= 2.0")
PY
}

validate_analyzer_setting() {
  case "${analyze_probe}" in
    true|false) ;;
    *) die "GO2_YAW_PROBE_ANALYZE must be true or false" ;;
  esac
  if [[ "${analyze_probe}" == "true" ]]; then
    [[ -x "${ANALYZER}" ]] || die "missing executable analyzer: ${ANALYZER}"
  fi
}

assert_no_conflicting_runtime() {
  local docker_matches
  docker_matches="$(docker ps --format '{{.Names}} {{.Image}}' 2>/dev/null | \
    rg 'go2_xt16|go2_yaw_probe|dddmr_go2_xt16|dddmr_navigation' || true)"
  if [[ -n "${docker_matches}" ]]; then
    echo "${docker_matches}" >&2
    die "stop existing Go2/DDDMR Docker containers before running yaw probe"
  fi

  local proc_matches
  proc_matches="$(ps -eo pid,args | \
    rg 'go2_sport_cmd_vel_adapter|go2_sport_cmd_vel_dry_run|go2_xt16_navigation.launch|rviz2|p2p_move_base_node|ros2 topic pub|ros2 topic echo' | \
    rg -v 'run_go2_yaw_feedback_probe|dddmr_docker_go2_xt16|bash -lc|py_compile|sed -n|nl -ba|rg |ps -eo' || true)"
  if [[ -n "${proc_matches}" ]]; then
    echo "${proc_matches}" >&2
    die "stop existing navigation/RViz/Sport adapter/ros2 pub/echo processes first"
  fi
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

wait_for_topic_type() {
  local topic="$1"
  local expected="$2"
  local deadline=$((SECONDS + 35))
  local info=""
  while (( SECONDS < deadline )); do
    info="$(ros2 topic info "${topic}" 2>&1 || true)"
    if [[ "${info}" == *"Type: ${expected}"* ]] && \
       [[ "${info}" =~ Publisher\ count:\ [1-9][0-9]* ]]; then
      echo "=== ${topic}"
      echo "${info}"
      return 0
    fi
    sleep 1
  done
  echo "${info}" >&2
  die "timed out waiting for ${topic} publisher"
}

start_docker_source() {
  echo "DOCKER_NAME=${docker_name}"
  echo "DOCKER_LOG=${docker_log}"
  DDDMR_DOCKER_NAME="${docker_name}" \
  RVIZ="${RVIZ:-false}" \
  PUBLISH_STATIC_TF="${PUBLISH_STATIC_TF:-true}" \
    "${DOCKER_WRAPPER}" navigation-live-source \
    >"${docker_log}" 2>&1 &
  docker_pid="$!"
  sleep 1
  if ! kill -0 "${docker_pid}" >/dev/null 2>&1; then
    cat "${docker_log}" >&2 || true
    die "Docker navigation source exited during startup"
  fi
}

start_request_echo() {
  echo "REQUEST_ECHO_LOG=${request_echo_log}"
  ros2 topic echo "${REAL_REQUEST_TOPIC}" unitree_api/msg/Request \
    >"${request_echo_log}" 2>&1 &
  echo_pid="$!"
}

run_probe_python() {
  /usr/bin/python3 - \
    "${REAL_REQUEST_TOPIC}" "${ODOM_TOPIC}" "${SPORT_TOPIC}" \
    "${probe_x}" "${probe_y}" "${probe_yaw}" "${probe_duration}" "${request_id_base}" \
    "${pre_balance_stand}" "${pre_balance_delay}" \
    "${probe_log}" "${odom_csv}" "${sport_csv}" "${request_csv}" "${summary_log}" <<'PY'
import csv
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

request_topic, odom_topic, sport_topic = sys.argv[1:4]
x_cmd = float(sys.argv[4])
y_cmd = float(sys.argv[5])
yaw_cmd = float(sys.argv[6])
duration = float(sys.argv[7])
request_id_base = int(sys.argv[8])
pre_balance_stand = sys.argv[9].strip().lower() in ("1", "true", "yes", "on")
pre_balance_delay = float(sys.argv[10])
probe_log, odom_csv, sport_csv, request_csv, summary_log = sys.argv[11:16]


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def norm_delta(a, b):
    return math.atan2(math.sin(b - a), math.cos(b - a))


class Probe(Node):
    def __init__(self):
        super().__init__("go2_yaw_feedback_probe")
        self.req_pub = self.create_publisher(Request, request_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.create_subscription(SportModeState, sport_topic, self.sport_cb, 20)
        self.odom = []
        self.sport = []
        self.requests = []

    def odom_cb(self, msg):
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.odom.append(
            (
                time.monotonic(),
                yaw,
                msg.twist.twist.angular.z,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )
        )

    def sport_cb(self, msg):
        self.sport.append(
            (
                time.monotonic(),
                float(msg.imu_state.rpy[2]),
                float(msg.yaw_speed),
                int(msg.mode),
                int(msg.gait_type),
                float(msg.position[0]),
                float(msg.position[1]),
            )
        )

    def publish_request(self, api_id, parameter, req_id):
        req = Request()
        req.header.identity.id = req_id
        req.header.identity.api_id = api_id
        req.parameter = parameter
        self.req_pub.publish(req)
        self.requests.append((time.monotonic(), req_id, api_id, parameter))


rclpy.init()
node = Probe()

with open(probe_log, "w", encoding="utf-8") as log:
    def write(line):
        print(line)
        log.write(line + "\n")
        log.flush()

    write(f"probe request_topic={request_topic} odom_topic={odom_topic} sport_topic={sport_topic}")
    write(
        f"probe x={x_cmd:.3f} y={y_cmd:.3f} yaw={yaw_cmd:.3f} duration={duration:.3f} "
        f"pre_balance_stand={pre_balance_stand} pre_balance_delay={pre_balance_delay:.3f} "
        f"request_id_base={request_id_base}"
    )

    # Let subscriptions and publisher match before sampling/publishing.
    start_wait = time.monotonic()
    while time.monotonic() - start_wait < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    pre_start = time.monotonic()
    while time.monotonic() - pre_start < 0.8:
        rclpy.spin_once(node, timeout_sec=0.05)

    if pre_balance_stand:
        balance_id = request_id_base + 1
        node.publish_request(1002, "", balance_id)
        write(f"published BalanceStand id={balance_id} api_id=1002")
        balance_start = time.monotonic()
        while time.monotonic() - balance_start < pre_balance_delay:
            rclpy.spin_once(node, timeout_sec=0.05)

    payload = json.dumps({"x": x_cmd, "y": y_cmd, "z": yaw_cmd}, separators=(",", ":"))
    move_count = max(2, math.ceil(duration * 10.0))
    next_pub = time.monotonic()
    for i in range(move_count):
        while time.monotonic() < next_pub:
            rclpy.spin_once(node, timeout_sec=0.01)
        req_id = request_id_base + 100 + i + 1
        node.publish_request(1008, payload, req_id)
        write(f"published Move id={req_id} api_id=1008 parameter={payload}")
        next_pub += 0.1

    stop_base = request_id_base + 500
    for i in range(3):
        req_id = stop_base + i + 1
        node.publish_request(1003, "", req_id)
        write(f"published StopMove id={req_id} api_id=1003")
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    post_start = time.monotonic()
    while time.monotonic() - post_start < 1.2:
        rclpy.spin_once(node, timeout_sec=0.05)

with open(odom_csv, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(["t_monotonic", "yaw_rad", "twist_angular_z", "x", "y"])
    writer.writerows(node.odom)

with open(sport_csv, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(["t_monotonic", "imu_rpy_yaw_rad", "yaw_speed", "mode", "gait_type", "x", "y"])
    writer.writerows(node.sport)

with open(request_csv, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(["t_monotonic", "request_id", "api_id", "parameter"])
    writer.writerows(node.requests)

def summarize_yaw(samples, yaw_index):
    if not samples:
        return None
    first = samples[0][yaw_index]
    last = samples[-1][yaw_index]
    values = [s[yaw_index] for s in samples]
    return first, last, norm_delta(first, last), min(values), max(values)

odom_summary = summarize_yaw(node.odom, 1)
sport_summary = summarize_yaw(node.sport, 1)
sport_yaw_speed_values = [s[2] for s in node.sport]

with open(summary_log, "w", encoding="utf-8") as f:
    def env(key, value):
        f.write(f"{key}={value}\n")

    env("RESULT", "GO2_YAW_FEEDBACK_PROBE_COMPLETE")
    env("REQUEST_TOPIC", request_topic)
    env("ODOM_TOPIC", odom_topic)
    env("SPORT_TOPIC", sport_topic)
    env("REQUEST_ID_BASE", request_id_base)
    env("X_CMD", f"{x_cmd:.6f}")
    env("Y_CMD", f"{y_cmd:.6f}")
    env("YAW_CMD", f"{yaw_cmd:.6f}")
    env("DURATION", f"{duration:.6f}")
    env("PRE_BALANCE_STAND", str(pre_balance_stand).lower())
    env("PRE_BALANCE_DELAY", f"{pre_balance_delay:.6f}")
    env("BALANCE_COUNT", sum(1 for _, _, api_id, _ in node.requests if api_id == 1002))
    env("MOVE_COUNT", sum(1 for _, _, api_id, _ in node.requests if api_id == 1008))
    env("STOP_COUNT", sum(1 for _, _, api_id, _ in node.requests if api_id == 1003))
    env("ODOM_SAMPLE_COUNT", len(node.odom))
    env("SPORT_SAMPLE_COUNT", len(node.sport))
    if odom_summary:
        env("ODOM_YAW_FIRST", f"{odom_summary[0]:.9f}")
        env("ODOM_YAW_LAST", f"{odom_summary[1]:.9f}")
        env("ODOM_YAW_DELTA", f"{odom_summary[2]:.9f}")
        env("ODOM_YAW_MIN", f"{odom_summary[3]:.9f}")
        env("ODOM_YAW_MAX", f"{odom_summary[4]:.9f}")
    if sport_summary:
        env("SPORT_RPY_YAW_FIRST", f"{sport_summary[0]:.9f}")
        env("SPORT_RPY_YAW_LAST", f"{sport_summary[1]:.9f}")
        env("SPORT_RPY_YAW_DELTA", f"{sport_summary[2]:.9f}")
        env("SPORT_RPY_YAW_MIN", f"{sport_summary[3]:.9f}")
        env("SPORT_RPY_YAW_MAX", f"{sport_summary[4]:.9f}")
    if sport_yaw_speed_values:
        env("SPORT_YAW_SPEED_MIN", f"{min(sport_yaw_speed_values):.9f}")
        env("SPORT_YAW_SPEED_MAX", f"{max(sport_yaw_speed_values):.9f}")
        env("SPORT_YAW_SPEED_LAST", f"{sport_yaw_speed_values[-1]:.9f}")
    env("PROBE_LOG", probe_log)
    env("ODOM_CSV", odom_csv)
    env("SPORT_CSV", sport_csv)
    env("REQUEST_CSV", request_csv)

node.destroy_node()
rclpy.shutdown()

print(f"SUMMARY_LOG={summary_log}")
if odom_summary:
    print(f"ODOM_YAW_DELTA={odom_summary[2]:.6f}")
else:
    print("ODOM_YAW_DELTA=NA")
if sport_summary:
    print(f"SPORT_RPY_YAW_DELTA={sport_summary[2]:.6f}")
else:
    print("SPORT_RPY_YAW_DELTA=NA")
if sport_yaw_speed_values:
    print(f"SPORT_YAW_SPEED_RANGE={min(sport_yaw_speed_values):.6f},{max(sport_yaw_speed_values):.6f}")
else:
    print("SPORT_YAW_SPEED_RANGE=NA")
PY
}

run_probe_analysis() {
  if [[ "${analyze_probe}" != "true" ]]; then
    {
      echo "ANALYSIS_ENABLED=false"
    } >>"${summary_log}"
    return 0
  fi

  local status
  set +e
  "${ANALYZER}" "${summary_log}" --write-env "${analysis_log}" \
    >"${analysis_output_log}" 2>&1
  status=$?
  set -e

  local analysis_result
  analysis_result="$(awk -F= '$1 == "RESULT" {print $2}' "${analysis_log}" 2>/dev/null || true)"
  {
    echo "ANALYSIS_ENABLED=true"
    echo "ANALYSIS_LOG=${analysis_log}"
    echo "ANALYSIS_OUTPUT_LOG=${analysis_output_log}"
    echo "ANALYSIS_EXIT_STATUS=${status}"
    echo "ANALYSIS_RESULT=${analysis_result}"
  } >>"${summary_log}"

  cat "${analysis_output_log}" || true
  return "${status}"
}

validate_probe_bounds
validate_analyzer_setting
assert_no_conflicting_runtime

[[ "${GO2_YAW_PROBE_CONFIRM:-}" == "${CONFIRM_PHRASE}" ]] || \
  die "live yaw probe requires GO2_YAW_PROBE_CONFIRM=${CONFIRM_PHRASE}"

source_host_go2_ros

echo "RESULT_DIR=${log_dir}"
echo "MODE=${mode}"
echo "X=${probe_x}"
echo "Y=${probe_y}"
echo "YAW=${probe_yaw}"
echo "DURATION=${probe_duration}"
echo "PRE_BALANCE_STAND=${pre_balance_stand}"
echo "PRE_BALANCE_DELAY=${pre_balance_delay}"
echo "ANALYZE=${analyze_probe}"
echo "REQUEST_ID_BASE=${request_id_base}"
echo "SUMMARY_LOG=${summary_log}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"

check_topic_type "${REAL_REQUEST_TOPIC}" "unitree_api/msg/Request"
check_topic_type "${SPORT_TOPIC}" "unitree_go/msg/SportModeState"
check_topic_type "${LOWSTATE_TOPIC}" "unitree_go/msg/LowState"

echo "LIVE YAW PROBE CAN MOVE THE GO2 THROUGH ${REAL_REQUEST_TOPIC}."
echo "Onsite supervision confirmation accepted."
start_docker_source
wait_for_topic_type "${ODOM_TOPIC}" "nav_msgs/msg/Odometry"
start_request_echo
live_runtime_started="true"
run_probe_python
live_runtime_started="false"

{
  echo "DOCKER_NAME=${docker_name}"
  echo "DOCKER_LOG=${docker_log}"
  echo "REQUEST_ECHO_LOG=${request_echo_log}"
} >>"${summary_log}"

if [[ -n "${echo_pid}" ]]; then
  sleep 0.5
  kill "${echo_pid}" >/dev/null 2>&1 || true
  wait "${echo_pid}" >/dev/null 2>&1 || true
  echo_pid=""
fi

analysis_status=0
run_probe_analysis || analysis_status=$?

echo "PROBE_LOG=${probe_log}"
echo "ODOM_CSV=${odom_csv}"
echo "SPORT_CSV=${sport_csv}"
echo "REQUEST_CSV=${request_csv}"
echo "REQUEST_ECHO_LOG=${request_echo_log}"
echo "SUMMARY_LOG=${summary_log}"
if [[ "${analyze_probe}" == "true" ]]; then
  echo "ANALYSIS_LOG=${analysis_log}"
  echo "ANALYSIS_OUTPUT_LOG=${analysis_output_log}"
fi
if [[ "${analysis_status}" -ne 0 ]]; then
  echo "RESULT: GO2_YAW_FEEDBACK_PROBE_ANALYSIS_FAIL"
  exit "${analysis_status}"
fi
echo "RESULT: GO2_YAW_FEEDBACK_PROBE_COMPLETE"
