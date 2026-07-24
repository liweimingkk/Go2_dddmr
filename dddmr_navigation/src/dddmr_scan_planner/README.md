# DDDMR SCAN-Planner integration

This package integrates the Go2-oriented SCAN-Planner local planner with the
existing DDDMR Go2 + Hesai XT16 stack.

The integration keeps:

- DDDMR pose-graph maps, `mcl_3dl`, and the static 3-D global planner
- the filtered, non-ground XT16 obstacle stream, transformed from `base_link`
  into the `map` frame by the adapter
- the existing `/dddmr_go2/dry_run_cmd_vel -> safe_cmd_vel` command gate
- the separately approval-gated Unitree Sport adapter

It replaces only the discrete DDDMR local trajectory sampler with
SCAN-Planner's route-guided B-spline local replanning and controller.

## Data flow

```text
DDDMR /get_plan -> scan_route_bridge -> SCAN reference path
MCL TF + odom   -> scan_input_adapter -> SCAN body/sensor pose
DDDMR obstacles -> scan_input_adapter -> SCAN sliding occupancy map
SCAN B-spline   -> SCAN controller -> scan_command_guard
scan_command_guard -> DDDMR dry-run topic -> existing safety gate
```

The adapter removes DDDMR's duplicated route-start pose before SCAN's
minimum-snap initialization, converts body-frame velocity and obstacle clouds
into the map frame, and rejects stale localization transforms. The command
guard requires fresh odometry, obstacle cloud, controller output, and SCAN
planner heartbeat.

The bridge preserves the final quaternion from a `3D Goal Pose`. SCAN tracks
the route tangent while translating, then holds position and aligns to that
terminal yaw. A goal is complete only when both the position and yaw
tolerances are satisfied.

Direct launches cannot publish Unitree Sport requests.

## Build and no-motion run

```bash
./scripts/dddmr_docker_go2_xt16.sh build-navigation
RVIZ=false RUN_SECONDS=30 \
  ./scripts/dddmr_docker_go2_xt16.sh scan-navigation-dry-run
```

The direct launch defaults `scan_max_vel_y` to zero. A nonzero lateral limit is
only injected by an explicitly configured dry-run or the existing supervised
live workflow.

To select SCAN in that existing supervised workflow, set:

```bash
export GO2_NAV_DRY_RUN_COMMAND=scan-navigation-dry-run
export GO2_NAV_DOCKER_COMMAND=scan-navigation-live-source
```

The live mode still requires all confirmations and probe evidence enforced by
`run_go2_xt16_navigation_supervised_live.sh`.

## Sequential waypoint missions

The top-level SCAN launcher also supports a fail-closed mission layer. It does
not replace SCAN path tracking: every waypoint is submitted to the existing
DDDMR global planner, and SCAN tracks the returned path before performing the
terminal-yaw alignment.

```bash
# First-time fixed-start calibration and waypoint recording. Set a correct
# RViz 3D Pose Estimate, press i to save it, then record each waypoint.
./scripts/run_go2_xt16_scan_navigation.sh \
  --record bags/scan_missions/route_a.json \
  --initial-pose bags/scan_missions/go2_start.json

# Full mission state machine, no real Unitree Sport output.
./scripts/run_go2_xt16_scan_navigation.sh \
  --multi-dry-run bags/scan_missions/route_a.json

# Supervised physical execution.
./scripts/run_go2_xt16_scan_navigation.sh \
  --multi-live bags/scan_missions/route_a.json
```

The executor loads the saved pose through `/initial_3d_pose`, the same topic as
RViz `3D Pose Estimate`. It waits briefly after map-ground delivery so MCL can
finish its ground tree, retries an unconfirmed seed, and requires a fresh
`/mcl_pose` near the saved position and yaw before accepting post-seed
`TRACKING` and `HEALTHY`. It remains disabled until the operator types
`EXECUTE <mission_id>`. It executes each waypoint once, requires both SCAN's
position/yaw tolerances and a stopped raw command for a continuous window,
dwells for the waypoint's required `dwell_sec`, and then submits the next
global-plan request.

The supported multi-point launcher isolates mission goals on
`/scan_multi_point/goal_pose_3d` and disconnects manual RViz/clicked-point
goals while the mission is active. Legacy single-goal modes keep their original
goal topics.

Mission files deliberately do not contain a map fingerprint. Their `map`
coordinates are valid only while the operator keeps the selected map frame
consistent with the recording session.

## Upstream

The semantically unmodified planner-only ROS 2 source is under
`../scan_planner_vendor/`. Its exact repository, branch, commit, license, and
NOTICE are recorded there.
