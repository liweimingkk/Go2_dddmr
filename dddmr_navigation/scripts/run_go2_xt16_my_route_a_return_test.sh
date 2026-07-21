#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_my_route_a_return_test.sh [--check|--dry-run]

Dedicated regression for:
  my_route_a start -> recorded-route endpoint -> P2P return to route start.

Modes:
  --check    Validate the dedicated map profile, route fingerprint, launch
             isolation, and configured return goal. No ROS or Docker runtime.
  --dry-run  Run --check, then execute the complete mission state machine in
             a network-isolated Docker container with mock controllers and
             zero odometry. No planner, velocity publisher, Sport adapter, or
             real robot interface is started.

Environment:
  DDDMR_BAGS_DIR=<repo-parent>/bags
  DDDMR_IMAGE=dddmr_go2_xt16:x64
EOF
}

mode="check"
if (( $# > 1 )); then
  usage >&2
  exit 2
fi
case "${1:---check}" in
  --check) mode="check" ;;
  --dry-run) mode="dry-run" ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"
BAGS_DIR="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
IMAGE="${DDDMR_IMAGE:-dddmr_go2_xt16:x64}"
MAP_NAME="go2_xt16_mouth_mapping_20260714_153136_map_2026_07_14_07_31_36"
MAP_DIR="${BAGS_DIR}/${MAP_NAME}"
ROUTE_DIR="${BAGS_DIR}/routes"
NAV_CONFIG="${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml"
TEST_CONFIG="${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_my_route_a_return_test.yaml"
MISSION_SUPERVISOR="${SCRIPT_DIR}/run_go2_xt16_outdoor_indoor_supervised.sh"
INSTALL_SETUP="${WS_ROOT}/.docker_go2_xt16_install/setup.bash"

run_check() {
  DDDMR_BAGS_DIR="${BAGS_DIR}" \
  MISSION_MAP_DIR="${MAP_DIR}" \
  MISSION_ROUTE_DIR="${ROUTE_DIR}" \
  MISSION_ROUTE_ID="my_route_a" \
  NAV_CONFIG_FILE="${NAV_CONFIG}" \
  MISSION_MAP_CONFIG_FILE="${TEST_CONFIG}" \
    "${MISSION_SUPERVISOR}" --check

  python3 - "${TEST_CONFIG}" "${ROUTE_DIR}/my_route_a.json" <<'PY'
import json
import math
from pathlib import Path
import sys

import yaml

config = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
route = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
params = config["outdoor_indoor_no_motion"]["ros__parameters"]
if params["route_id"] != route["route_id"]:
    raise SystemExit("dedicated test route_id does not match my_route_a.json")
if params["active_map_directory"] != route["map"]["directory"]:
    raise SystemExit("dedicated test map directory does not match route metadata")
start = route["points"][0]
configured = tuple(float(params[f"return_goal_{axis}"]) for axis in ("x", "y", "z"))
recorded = tuple(float(start[axis]) for axis in ("x", "y", "z"))
if any(abs(lhs - rhs) > 1.0e-6 for lhs, rhs in zip(configured, recorded)):
    raise SystemExit(f"return goal {configured} does not match route start {recorded}")
qx, qy, qz, qw = (float(start[key]) for key in ("qx", "qy", "qz", "qw"))
recorded_yaw = math.atan2(
    2.0 * (qw * qz + qx * qy),
    1.0 - 2.0 * (qy * qy + qz * qz),
)
yaw_error = math.atan2(
    math.sin(float(params["return_goal_yaw"]) - recorded_yaw),
    math.cos(float(params["return_goal_yaw"]) - recorded_yaw),
)
if abs(yaw_error) > 1.0e-6:
    raise SystemExit("configured return yaw does not match route start yaw")
print("RETURN_GOAL_XYZ=%.9f %.9f %.9f" % configured)
print("RETURN_GOAL_YAW=%.9f" % float(params["return_goal_yaw"]))
PY
}

run_check
if [[ "${mode}" == "check" ]]; then
  echo "RESULT=MY_ROUTE_A_RETURN_CHECK_PASS"
  exit 0
fi

[[ -f "${INSTALL_SETUP}" ]] || {
  echo "ERROR: Docker navigation install is missing; run build-navigation first." >&2
  exit 1
}
docker image inspect "${IMAGE}" >/dev/null 2>&1 || {
  echo "ERROR: Docker image is missing: ${IMAGE}" >&2
  exit 1
}

docker run --rm --network=none \
  --env ROS_LOCALHOST_ONLY=1 \
  --env ROS_DOMAIN_ID=42 \
  --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  --volume "${WS_ROOT}:/root/dddmr_navigation:ro" \
  --volume "${BAGS_DIR}:/root/dddmr_bags:ro" \
  --workdir /root/dddmr_navigation \
  "${IMAGE}" \
  bash -lc 'set -eo pipefail
source /opt/ros/humble/setup.bash
source .docker_go2_xt16_install/setup.bash
timeout -s TERM -k 5s 20s ros2 run dddmr_route_navigation outdoor_indoor_mission_no_motion.py \
  --ros-args --params-file \
  /root/dddmr_navigation/src/dddmr_beginner_guide/config/go2_xt16_my_route_a_return_test.yaml'

echo "RESULT=MY_ROUTE_A_RETURN_NO_MOTION_DRY_RUN_PASS"
