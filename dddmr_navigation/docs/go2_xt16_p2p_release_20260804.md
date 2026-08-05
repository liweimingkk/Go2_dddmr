# Go2 XT16 + DDDMR P2P stable baseline (2026-08-04)

## Release decision

This is the deployable navigation default for `v1.0.0-xt16-p2p`:

- map geometry: top-mounted Hesai XT16 point cloud only;
- localization/global planning: DDDMR pose graph, `mcl_3dl`, and the DDDMR
  static global planner;
- local planning: ordinary DDDMR P2P (`omni_drive_simple`), not SCAN-Planner;
- lateral velocity: locked at `0.0 m/s` through the planner, command gate, and
  Unitree Sport adapter;
- supervised live runtime: 300-second default and 1800-second hard maximum;
- static-layer readiness timeout: 180 seconds for this map contract.

SCAN-Planner remains available in the source tree as an explicit experimental
backend. Its separate entry points contain `scan-navigation` in their names;
the stable launcher never selects them. The older dual-lidar map is not the
default and is retained only as historical data outside Git.

## Immutable map contract

The deployed container directory is:

```text
/root/dddmr_bags/go2_xt16_mapping_20260804_141647_xt16_only
```

It was selected from the mouth-lidar-off A/B output:

```text
go2_xt16_mouth_mapping_20260804_141647_ab_compare/
  mouth_off_map_2026_08_04_06_57_16
```

Map inspection reports:

```text
KEY_FRAME_COUNT=144
MAP_POINT_COUNT=18138
GROUND_POINT_COUNT=86749
EDGE_POINT_COUNT=144
STATIC_LAYER_TIMEOUT_SEC=180
```

SHA-256 fingerprints of the deployed map copy are:

```text
53a3e8c4b13916886f8f76c1f749964835f68bcb8f4ce030f68d90b996b9710b  edges.pcd
a77786d5db2e336c8179e731532abf2cd589253d6c6cbd4512d1158fd07ad46b  ground.pcd
f2b8e701a385efd341a8a1cee9fa30600efb708021b47b288f16050754daaa23  map.pcd
b52e40dae4cf52db2bbd0e6b6a4c631e6d65107d5f044adc59c4e94a2ccf3d2c  poses.pcd
d7f43a38496a8a0d0f21c316b5f92401371a5c5728e4deac45c1e14a304fe69d  pcd/ canonical checksum-list digest
```

The canonical `pcd/` digest is the SHA-256 of the sorted output from
`sha256sum pcd/<file>` for all 432 key-frame cloud files.

"XT16 only" describes the point-cloud geometry used to build the map. The
2026-08-04 bag still supplied `/utlidar/robot_odom` as the odometry input; this
release does not claim an XT16-only odometry pipeline.

## Field evidence and remaining boundary

The map was recorded for about 281 seconds on 2026-08-04. The ordinary P2P
launcher completed outdoor goals against the selected map in the supervised
sessions starting at 15:21:43, 15:35:58, and 16:11:36. The last session logged
two completed goals; an additional goal was preempted by a newer request.

This evidence accepts the map/P2P combination as the default baseline. It does
not accept SCAN-Planner or the experimental P2P lateral-avoidance changes. Any
future physical test still requires onsite supervision and the launcher's live
confirmation gates.

## Reproduction checks

No-motion configuration and map validation:

```bash
python3 scripts/go2_pose_graph_map_contract.py inspect \
  --map-dir ../bags/go2_xt16_mapping_20260804_141647_xt16_only
python3 src/dddmr_beginner_guide/test/test_go2_xt16_lateral_lockout.py
scripts/run_go2_xt16_navigation_test.sh --quick --dry-run
```

Run these commands from `dddmr_navigation/`. The last command requires the
normal Docker/ROS sensor environment but cannot publish real Sport requests.
