#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_outdoor_indoor_supervised.sh [--check|--dry-run|--live]

Validate and launch the one-map outdoor-route to indoor-goal mission runtime.

Modes:
  --check    Default. Validate the selected nav map, route fingerprint, launch,
             and fail-closed configuration without Docker, ROS, or motion.
  --dry-run  Launch the mission runtime with the Sport dry-run logger only.
  --live     Reuse the existing supervised host Sport adapter and launch the
             mission runtime as a velocity source. This can move the Go2 after
             a mission Action is sent and requires both confirmations below.

Required selection:
  MISSION_MAP_DIR=<host unified pose-graph map directory>
  MISSION_ROUTE_DIR=<host route JSON directory>      # default: bags/routes
  MISSION_ROUTE_ID=<route file stem>                 # default: my_route_a
  NAV_CONFIG_FILE=<host navigation YAML>

Additional live confirmation:
  GO2_OUTDOOR_INDOOR_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_OUTDOOR_INDOOR_MISSION

The reused live supervisor also requires:
  GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV
  GO2_SPORT_PROBE_SUMMARY=<successful supervised adapter probe summary>

After the runtime is ready, send a mission in another sourced ROS terminal:
  ros2 run dddmr_route_navigation outdoor_indoor_mission_client.py \
    <mission_id> <route_id> <indoor_x> <indoor_y> <indoor_z> <indoor_yaw>

No mode automatically sends a mission or publishes a goal.
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
  --live) mode="live" ;;
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
DEFAULT_MAP_NAME="go2_xt16_mouth_mapping_20260720_155833_map_2026_07_20_07_58_32"
MAP_DIR="${MISSION_MAP_DIR:-${BAGS_DIR}/${DEFAULT_MAP_NAME}}"
ROUTE_DIR="${MISSION_ROUTE_DIR:-${BAGS_DIR}/routes}"
ROUTE_ID="${MISSION_ROUTE_ID:-my_route_a}"
NAV_CONFIG="${NAV_CONFIG_FILE:-${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml}"
MISSION_CONFIG="${WS_ROOT}/src/dddmr_route_navigation/config/go2_xt16_outdoor_indoor_mission.yaml"
MISSION_LAUNCH="${WS_ROOT}/src/dddmr_beginner_guide/launch/go2_xt16_outdoor_indoor_mission.launch"
MISSION_LIB="${WS_ROOT}/src/dddmr_route_navigation/scripts/outdoor_indoor_mission_lib.py"
SUPERVISOR="${SCRIPT_DIR}/run_go2_xt16_navigation_supervised_live.sh"
LIVE_CONFIRM="I_AM_SUPERVISING_GO2_OUTDOOR_INDOOR_MISSION"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

path_inside() {
  local base target
  base="$(realpath -m -- "$1")"
  target="$(realpath -m -- "$2")"
  [[ "${target}" == "${base}"/* ]] || return 1
  printf '%s\n' "${target#"${base}"/}"
}

validate_selection() {
  local map_relative route_relative container_map container_routes
  [[ -f "${MAP_DIR}/poses.pcd" ]] || die "unified map poses.pcd is missing: ${MAP_DIR}"
  [[ -f "${ROUTE_DIR}/${ROUTE_ID}.json" ]] || \
    die "mission route is missing: ${ROUTE_DIR}/${ROUTE_ID}.json"
  [[ -f "${NAV_CONFIG}" ]] || die "navigation config is missing: ${NAV_CONFIG}"
  [[ -f "${MISSION_CONFIG}" ]] || die "mission config is missing: ${MISSION_CONFIG}"
  [[ -f "${MISSION_LAUNCH}" ]] || die "mission launch is missing: ${MISSION_LAUNCH}"
  [[ -f "${MISSION_LIB}" ]] || die "mission validation library is missing: ${MISSION_LIB}"

  map_relative="$(path_inside "${BAGS_DIR}" "${MAP_DIR}")" || \
    die "MISSION_MAP_DIR must stay under DDDMR_BAGS_DIR"
  route_relative="$(path_inside "${BAGS_DIR}" "${ROUTE_DIR}")" || \
    die "MISSION_ROUTE_DIR must stay under DDDMR_BAGS_DIR"
  container_map="/root/dddmr_bags/${map_relative}"
  container_routes="/root/dddmr_bags/${route_relative}"

  grep -Fq "pose_graph_dir: \"${container_map}\"" "${NAV_CONFIG}" || \
    die "NAV_CONFIG_FILE does not select ${container_map} as pose_graph_dir"
  grep -Fq 'goals_enabled: false' "${MISSION_CONFIG}" || \
    die "mission config must start P2P goal execution disabled"
  grep -Fq 'output_command_topic: "/dddmr_go2/dry_run_cmd_vel"' "${MISSION_CONFIG}" || \
    die "mission mux output is not isolated on /dddmr_go2/dry_run_cmd_vel"
  grep -Fq '<arg name="start_move_base" value="false"/>' "${MISSION_LAUNCH}" || \
    die "mission launch must disable the legacy P2P process"
  grep -Fq '<arg name="start_go2_sport_adapter" value="false"/>' "${MISSION_LAUNCH}" || \
    die "mission launch must not start the real Sport adapter"
  grep -Fq '<remap from="/cmd_vel" to="/dddmr_go2/indoor_cmd_vel"/>' "${MISSION_LAUNCH}" || \
    die "mission launch does not isolate recovery commands on the indoor source"

  python3 - "${MISSION_LIB}" "${ROUTE_DIR}" "${ROUTE_ID}" "${MAP_DIR}" "${MISSION_LAUNCH}" <<'PY'
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

spec = importlib.util.spec_from_file_location("mission_lib", sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ET.parse(sys.argv[5])
try:
    route_path, document, fingerprint = module.load_validated_route(
        Path(sys.argv[2]), sys.argv[3], Path(sys.argv[4])
    )
except (OSError, ValueError) as exc:
    raise SystemExit(f"mission selection invalid: {exc}")
print(f"ROUTE_FILE={route_path}")
print(f"ROUTE_POINTS={len(document['points'])}")
print(f"UNIFIED_MAP_SHA256={fingerprint}")
PY

  export MISSION_MAP_DIRECTORY="${container_map}"
  export MISSION_ROUTE_DIRECTORY="${container_routes}"
  echo "MISSION_MAP_DIRECTORY=${MISSION_MAP_DIRECTORY}"
  echo "MISSION_ROUTE_DIRECTORY=${MISSION_ROUTE_DIRECTORY}"
  echo "MISSION_ROUTE_ID=${ROUTE_ID}"
}

validate_selection
if [[ "${mode}" == "check" ]]; then
  echo "RESULT=OUTDOOR_INDOOR_MISSION_CHECK_PASS"
  exit 0
fi

export GO2_NAV_DRY_RUN_COMMAND="outdoor-indoor-dry-run"
export GO2_NAV_DOCKER_COMMAND="outdoor-indoor-live-source"
if [[ "${mode}" == "live" ]]; then
  [[ "${GO2_OUTDOOR_INDOOR_LIVE_CONFIRM:-}" == "${LIVE_CONFIRM}" ]] || \
    die "--live requires GO2_OUTDOOR_INDOOR_LIVE_CONFIRM=${LIVE_CONFIRM}"
  exec "${SUPERVISOR}" --live
fi
exec "${SUPERVISOR}" --dry-run
