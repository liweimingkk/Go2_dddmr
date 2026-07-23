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

## Upstream

The semantically unmodified planner-only ROS 2 source is under
`../scan_planner_vendor/`. Its exact repository, branch, commit, license, and
NOTICE are recorded there.
