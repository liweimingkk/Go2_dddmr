#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EDITOR_DIR="/root/dddmr_bags/go2_xt16_mouth_mapping_20260723_153831_editor_clouds"

exec "${SCRIPT_DIR}/dddmr_navigation/scripts/dddmr_docker_go2_xt16.sh" \
  pose-graph-editor \
  "${EDITOR_DIR}/editor_map.pcd" \
  "${EDITOR_DIR}/editor_ground.pcd"
