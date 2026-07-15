#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/check_go2_xt16_no_motion_runtime_clean.sh

Read-only residual runtime check for the Go2 XT16 no-motion audit.

This script only runs docker ps and ps. It does not source ROS, start Docker,
launch nodes, inspect live robot topics, publish /cmd_vel, publish
/dddmr_go2/dry_run_cmd_vel, or publish /api/sport/request.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo "Unknown argument: $1" >&2
  usage >&2
  exit 2
fi

docker_status="PASS"
process_status="PASS"

echo "=== Docker residual check"
docker_matches="$(
  docker ps --format '{{.Names}} {{.Image}}' 2>/dev/null | \
    grep -E 'go2_xt16|dddmr_go2_xt16|dddmr_navigation' || true
)"
if [[ -n "${docker_matches}" ]]; then
  docker_status="FAIL"
  echo "GO2_XT16_RUNTIME_DOCKER_MATCHES_BEGIN"
  printf '%s\n' "${docker_matches}"
  echo "GO2_XT16_RUNTIME_DOCKER_MATCHES_END"
fi
echo "GO2_XT16_RUNTIME_DOCKER_STATUS=${docker_status}"

echo "=== Process residual check"
process_matches="$(
  ps -eo pid,args | \
    grep -E 'go2_sport_cmd_vel_adapter|go2_sport_cmd_vel_dry_run|go2_xt16_navigation.launch|lego_loam_go2_xt16_live.launch|lego_loam_go2_xt16_mouth.launch|p2p_move_base_node|global_planner_node|mcl_3dl|mcl_feature|dddmr_pg_map_server_node|rviz2|ros2 topic pub|ros2 topic echo' | \
    grep -Ev 'check_go2_xt16_no_motion_runtime_clean|check_go2_xt16_manual_gate_status|check_go2_xt16_report_completion|bash -lc|py_compile|sed -n|nl -ba|grep |ps -eo' || true
)"
if [[ -n "${process_matches}" ]]; then
  process_status="FAIL"
  echo "GO2_XT16_RUNTIME_PROCESS_MATCHES_BEGIN"
  printf '%s\n' "${process_matches}"
  echo "GO2_XT16_RUNTIME_PROCESS_MATCHES_END"
fi
echo "GO2_XT16_RUNTIME_PROCESS_STATUS=${process_status}"

overall="PASS"
if [[ "${docker_status}" != "PASS" || "${process_status}" != "PASS" ]]; then
  overall="FAIL"
fi
echo "GO2_XT16_RUNTIME_CLEAN_STATUS=${overall}"

if [[ "${overall}" != "PASS" ]]; then
  exit 1
fi
