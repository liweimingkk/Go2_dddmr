# Go2 XT16 and Mouth LiDAR Navigation Review Baseline

Review date: 2026-07-10

Repository revision reviewed:
`fbb875ecc7e153e5454cadcc35751d0216912061`

Baseline status: **NOT READY for unsupervised live navigation**

This document records the source, configuration, saved-map, and stored-log
findings from the 2026-07-10 review. It is the factual baseline for follow-up
work, including the wall-crossing investigation. A finding remains open until
its implementation and acceptance evidence are both updated here.

## Scope and Safety Boundary

The review covered:

- the XT16 and mouth LiDAR LeGO-LOAM integration;
- local and global `perception_3d` configuration;
- global A* graph expansion and static-layer behavior;
- navigation and Sport adapter launch scripts;
- the map selected by `go2_xt16_navigation.yaml`;
- stored mapping/navigation logs and review packets; and
- the repository's existing acceptance scripts and test results.

No motion command was published. No live robot or container test was run.
Docker API access was unavailable in the review environment, so container
runtime state could not be independently verified.

## Confirmed Architecture

- Main XT16 input: `/lidar_points`, frame `hesai_lidar`.
- Mouth LiDAR input: `/utlidar/cloud_base`, frame `base_link`.
- Mouth points passing the configured ROI are appended only to
  `patched_ground_` in the LeGO-LOAM projection node.
- Both local and global navigation obstacle layers subscribe to
  `segmented_cloud_pure`.
- The current design is therefore XT16 online obstacle perception with mouth
  LiDAR ground augmentation. It is not dual-LiDAR online obstacle fusion.
- The selected map is:
  `/root/dddmr_bags/go2_xt16_mouth_mapping_20260709_130235_map_2026_07_09_05_02_34`.

## Findings Summary

| ID | Severity | State | Finding |
| --- | --- | --- | --- |
| NAV-REV-01 | Blocker | Fixed; live pending | Local LiDAR `xy_resolution` loads as zero |
| NAV-REV-02 | Blocker | Fixed; synthetic pass | Sparse A* expansion mismatches neighbor indices and distances |
| NAV-REV-03 | Blocker | Mitigated; field pending | Generic live launcher bypasses the supervised-live gate and can leak a real Sport publisher |
| NAV-REV-04 | High | Open | Mouth LiDAR is absent from online obstacle perception |
| NAV-REV-05 | High | Open | Mouth ground extraction is a stationary fixed ROI, not traversability classification |
| NAV-REV-06 | High | Open | Ground graph can connect low or disconnected layers without slope/step/drop limits |
| NAV-REV-07 | High | Revised after failed live; field pending | Planner clearance parameters are smaller than the configured robot body |
| NAV-REV-08 | Medium | Open | XT16 hardware ring and per-point timestamp fields are not consumed |
| NAV-REV-09 | Medium | Open | Map promotion does not prove that mouth fusion was active and valid |
| NAV-REV-10 | Medium | Open | Stored evidence does not demonstrate repeatable goal completion |
| NAV-REV-11 | Low | Open | Saved pose `time` fields contain uninitialized values |
| NAV-REV-12 | High | Fixed; live pending | Active 2000-sample rings were projected into a 4000-column range image |

## Detailed Findings

### NAV-REV-01: Local LiDAR resolution is zero

Confirmed defect:

- `src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml:330` sets
  `resolution: 0.05` for `perception_3d_local.lidar`.
- `src/dddmr_perception_3d/plugins/multilayer_spinning_lidar.cpp:112-118`
  declares and reads only `xy_resolution`, whose default is `0.0`.
- The stored stair-navigation log reports local `xy_resolution: 0.00` and
  global `xy_resolution: 0.05` in the same process startup:
  `../report_packages/stair_nav_ground_drop_review_20260708_152518/evidence/docker_nav_filtered_key_events.log:87,120`.
- The plugin divides coordinates and window bounds by `resolution_` at
  `multilayer_spinning_lidar.cpp:473-474` and `:551-554` without rejecting a
  non-positive value.

Impact:

- local marking and clearing voxel indices are not reliable;
- floating-point division by zero followed by integer conversion can produce
  undefined behavior; and
- local obstacle handling and DWA conflict decisions cannot be accepted while
  this configuration is active.

Required disposition:

- use `xy_resolution: 0.05` in the local configuration;
- make the plugin fail startup when either resolution is non-positive; and
- add a parameter-loading regression test.

2026-07-10 update: the local key is now `xy_resolution: 0.05`, the plugin fails
startup for non-positive XY or height resolution, and the wall-safety config
gate passes. Field dry-run remains pending.

### NAV-REV-02: Sparse A* neighbor arrays are inconsistent

Confirmed defect in
`src/dddmr_global_planner/src/a_star_on_pc.cpp:237-261`:

- the first radius search fills `pointIdxRadiusSearch` and
  `pointRadiusSquaredDistance`;
- when fewer than eight neighbors are found, a larger search fills the `X2`
  index and distance vectors;
- only `pointIdxRadiusSearch` is replaced; and
- the expansion loop indexes the old distance vector using the new index-vector
  length.

Impact:

- successor costs can use distances belonging to different neighbors;
- the loop can read beyond the old distance vector; and
- path selection can be inconsistent or fail on sparse ground regions.

Required disposition:

- replace both vectors together, or retain a single search-result structure;
- assert equal vector lengths before expansion; and
- add a sparse-node planner test that exercises the expanded-radius branch.

2026-07-10 update: the expanded-radius branch now swaps both result vectors and
rejects empty or mismatched search results before indexing. The target packages
compile and the synthetic wall integration test passes.

### NAV-REV-03: Unsupervised real Sport publisher lifecycle

Confirmed defect:

- `scripts/run_go2_xt16_navigation_test.sh:251-279` starts a detached adapter
  publishing to `/api/sport/request` with `enable_sport_output:=true` and
  `allow_real_request_topic:=true`.
- This path does not require the confirmation phrase and validated Sport probe
  used by `scripts/run_go2_xt16_navigation_supervised_live.sh`.
- When `RUN_SECONDS` is empty, `run_go2_xt16_navigation_test.sh:327-333`
  returns while the navigation container and adapter remain active.
- The adapter intentionally sends `StopMove` keepalives after command staleness
  at `src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py:291-322`.

Stored evidence:

`run_logs/go2_xt16_nav_live_x050_yaw050_20260709_134445_adapter.log` contains:

- 128 logged `api_id=1008` Move entries (Move logging is throttled);
- 25,107 `api_id=1003` StopMove publications;
- 1,618 `ddsi_udp_conn_write` errors; and
- a final stale-command age of 13,198.613 seconds.

The stale state does not itself continue a Move command, but the residual real
publisher can conflict with later control sessions and shows that the live
lifecycle is not closed.

Required disposition:

- remove real output from the generic test launcher, or route it exclusively
  through the supervised-live gate;
- require a bounded runtime and cleanup trap for every live adapter; and
- make shutdown send a bounded stop sequence and confirm publisher exit.

2026-07-10 update:

- generic live mode now requires
  `GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV`;
- live runtime defaults to 300 seconds and is capped at 1800 seconds;
- exit, interrupt, startup failure, and timeout run adapter-first cleanup;
- navigation containers carry an explicit label and current-name record, so an
  arbitrary `NAV_CONTAINER_NAME` is still found by `--stop`;
- a no-motion arbitrary-name lifecycle test confirmed stop and removal; and
- the Sport adapter now requires a fresh allowed planner decision, blocking
  planning, waiting, and recovery commands with StopMove.

The first supervised wall run proved why the decision gate is required: after
losing its path, the pre-gate adapter admitted `d_recovery_waitdone` yaw
commands. A no-motion probe now confirms that recovery is blocked while
`d_controlling` remains permitted. A second supervised field run is pending.

### NAV-REV-04: Mouth LiDAR is not an online obstacle source

Confirmed architecture gap:

- `imageProjection.cpp:589-617` applies only finite/range/XYZ ROI filters to
  mouth points.
- Accepted points are appended to `patched_ground_` at
  `imageProjection.cpp:656-664`.
- Local and global navigation layers use `segmented_cloud_pure` at
  `go2_xt16_navigation.yaml:322,370`.

Impact:

The mouth sensor does not currently supply online near-field marking, clearing,
drop detection, emergency stopping, or a navigation terrain update. A saved map
may contain mouth-derived ground points, but that does not make the runtime
navigation stack dual-LiDAR.

Required disposition:

- decide whether the mouth sensor is mapping-only or a runtime safety sensor;
- if it is a runtime sensor, integrate a separately validated obstacle/terrain
  output into perception instead of injecting all accepted points as ground.

### NAV-REV-05: Mouth ground filtering is stationary-only

Confirmed limitation in
`src/dddmr_lego_loam/lego_loam_bor/config/loam_go2_xt16_mouth_config.yaml:40-69`:

- filter frame: `base_link`;
- z window: `-0.42` to `-0.18` metres;
- x window: `0.05` to `3.0` metres; and
- absolute y limit: `1.2` metres.

The implementation has no plane normal, slope, step, drop, temporal
consistency, leg/self mask, or roll/pitch/body-height compensation. The existing
`docs/go2_xt16_mouth_ground_fusion_notes.md` also labels this setup as
stationary-only.

Impact:

Body attitude and standing-height changes can move real ground outside the ROI
or move low obstacles, kerbs, and stair geometry inside it. The output must not
be treated as validated walking traversability.

### NAV-REV-06: Ground layers are connected without traversability limits

Current saved-map statistics were reproduced with:

```bash
python3 \
  review_packets/go2_xt16_ground_layer_issue_20260704_0852/tools/ground_pcd_z_stats.py \
  ../bags/go2_xt16_mouth_mapping_20260709_130235_map_2026_07_09_05_02_34
```

Results:

```text
top_level_ground_pcd: points=10447 z_min=-1.314 p05=-0.557 p50=-0.363 p95=-0.090
pg_map_server_style_merged_ground: points=59280 z_min=-1.314 p05=-0.530 p50=-0.372 p95=-0.111
merged points below -0.5 m: 5299
```

The same map contains 43 poses whose z values range from approximately
`-0.133` to `0.006` metres. Low z values alone do not prove that every point is
false ground; visible lower terrain can be real. They do prove that the planner
must not assume all spatially nearby ground samples belong to one traversable
surface.

The A* implementation:

- builds successors using a radius search at `a_star_on_pc.cpp:237-249`; and
- has its only pitch rejection code commented out at `:287-288`.

There is no effective maximum edge slope, upward step, downward drop, surface
normal, support continuity, or connected-layer constraint. The stored stair
review packet records a path appearing below the ground surface, repeated DWA
planning aborts, and `No path found` events.

Required disposition:

- reject graph edges using separately configured maximum slope, step-up, and
  drop-down limits;
- prevent connections between unsupported surface layers;
- clean or rebuild the map only after mouth classification is corrected; and
- add synthetic multi-level and wall-adjacent planner tests.

### NAV-REV-07: Clearance is smaller than the robot body

Confirmed configuration mismatch:

- the configured local collision cuboid extends `0.42 m` forward,
  `0.35 m` backward, and `0.18 m` to each side at
  `go2_xt16_navigation.yaml:158-166`;
- local and global `inscribed_radius` and `inflation_radius` are all `0.1 m` at
  `:300-307` and `:337-344`;
- `StaticLayer` sets a ground node to `0.25` when enough overhead map points
  exist at `src/dddmr_perception_3d/plugins/static_layer.cpp:386-412`; and
- A* rejects a node only when `dGraphValue < inscribed_radius` at
  `a_star_on_pc.cpp:263-270`.

With the current `0.1 m` threshold, a static node assigned `0.25` is not lethal;
it contributes cost instead. Online observations and line-of-sight checks can
still reject some paths, but the static graph's lethal semantics and the local
physical cuboid are inconsistent. This can produce a global path that the local
planner repeatedly refuses.

Required disposition:

- derive planner clearance from the actual swept footprint, including legs and
  localization uncertainty;
- define one consistent lethal-distance contract for static and dynamic layers;
  and
- test narrow passages and wall-parallel paths at measured clearances.

2026-07-10 update:

- the first `0.50 m`/zero-distance correction rejected the wall but also
  classified 1446 of 2717 real-map ground nodes and isolated the doorway start
  in a 71-node component;
- the revised centerline hard clearance is `0.32 m`, covering the `0.18 m`
  half-width plus `0.14 m` margin while the local oriented cuboid checks the
  longer body extent;
- static classification uses a `0.35 m` XY window and stores measured
  horizontal obstacle distance instead of a fixed value;
- exact saved-map replay restores the start to the 1874-node main component;
  and
- isolated tests reject a closed wall while accepting both 0.8 m and 1.4 m
  doorways.

Subsequent field tuning:

- the operator confirmed that the `0.32 m`/`0.35 m` revision generated the
  required real-scene path, but judged live clearance too small;
- an operator-requested consistent `0.50 m` hard/classification trial produced
  no field path, matching exact-map replay that isolated the start in a
  62-node component; and
- the current `0.40 m` hard/classification candidate restores the start to the
  1642-node main component, rejects a closed synthetic wall, accepts the 0.8 m
  and 1.4 m doorway controls, and loads with matching counts in a bounded
  no-Sport real-sensor run.

The exact field goal check for `0.40 m` and subsequent supervised-live
acceptance remain pending. See `docs/go2_xt16_wall_crossing_case_20260710.md`.

### NAV-REV-08: XT16 ring and hardware timestamp fields are discarded

Confirmed implementation/documentation mismatch:

- `src/dddmr_lego_loam/lego_loam_bor/include/utility.h:48` defines the working
  point type as `pcl::PointXYZI`;
- `imageProjection.cpp:698-702` converts incoming data to that point type;
- scan rows are recalculated from vertical angle at `imageProjection.cpp:769-773`;
  and
- `featureAssociation.cpp:339-345` derives relative time from azimuth.

The incoming XT16 `ring` and per-point `timestamp` fields are therefore not
used. The code has azimuth-derived LOAM distortion adjustment, not hardware
per-point timestamp deskew. Documentation that claims active ring/time use is
stronger than the implementation.

### NAV-REV-09: Map promotion does not prove mouth fusion

Confirmed evidence gap in
`scripts/run_go2_xt16_mouth_mapping_save_to_nav.sh`:

- the default mouth time offset is fixed at `0.03424` seconds at `:58-59`;
- startup waits for `/save_mapped_point_cloud` and `/lego_loam_map`, but not a
  valid `/mouth_ground_cloud` sample, at `:392-395`; and
- a newly saved map can automatically replace the navigation map path at
  `:433-435`.

The selected map directory contains PCD artifacts but no timing measurement,
fusion counters, configuration snapshot, or mapping log. Logs from 2026-07-08
show that an offset near `0.034` seconds was valid in that runtime, but this does
not prove that every later promoted map used stable, accepted mouth samples.

Required disposition:

- require a fresh stable-offset measurement;
- require nonzero accepted mouth-ground statistics for a defined interval;
- save the launch configuration and validation summary with the map; and
- promote the map only after those checks pass.

### NAV-REV-10: No repeatable navigation success evidence

A case-insensitive search of repository logs found no `goal reached`,
`succeeded`, or equivalent completion record. The latest reviewed live log
contains Move output followed by an extended goal-heading-alignment period and
then stale-command stopping. Other stored runs end in waiting, planning-wait,
or recovery-wait states.

`docs/go2_xt16_axis_tilt_acceptance_evidence.env:35-48` records operator
confirmation that the robot moved correctly toward a selected goal, not that it
reached the goal within measured pose and yaw tolerances. The evidence does not
contain endpoint error, stop distance, clearance, localization error,
time-to-goal, or repeated-trial results.

Required disposition:

- define a machine-readable success state and final pose/yaw error;
- record minimum wall/obstacle clearance and commanded/observed stop behavior;
- repeat representative trials; and
- retain failure logs and map/config hashes with each result.

### NAV-REV-11: Pose time fields are uninitialized

The selected map's `poses.pcd` contains 43 `time` values near
`6.589e-310`. The map-save conversion fills pose, attitude, and intensity but
does not initialize the `time` field. The current map server does not use this
field, so no direct navigation failure was established, but the artifact is
corrupt and unsuitable for future time-dependent processing.

### NAV-REV-12: Active scan width and range-image width were inconsistent

Read-only live preflight on 2026-07-10 measured 32000 points per scan, 16 rings,
2000 points per ring, and approximately 10 Hz. The operator confirmed this is
an intentional rate tradeoff: the 64000-point mode delivered only about 5 Hz.
The navigation, live mapping, and mouth-fusion mapping configurations still
declared 4000 horizontal scans.

A pair of bounded, no-Sport-output navigation runs showed that the mismatched
4000-column projection produced `/segmented_cloud_pure` at about 8.5-9.0 Hz,
while a matching 2000-column projection sustained 9.96-10.01 Hz. The global
LiDAR layer's repeated approximately 0.20-second cadence warnings also cleared
in the matched run. Static map classification remained 1446 lethal nodes out
of 2717 ground nodes in both runs.

2026-07-10 disposition: the three live XT16 configurations and the default
preflight contract now use 2000 horizontal scans / 32000 points. Historical
bag-specific configurations retain their recorded scan dimensions.

## Validation and Tooling Limitations

- Targeted XML parsing, YAML parsing, shell syntax checks, and Python byte-code
  compilation passed during the review.
- `colcon test-result --all --verbose` reported zero tests.
- Docker API access returned permission denied, so no container build or runtime
  smoke test was performed.
- `scripts/check_go2_xt16_no_motion_runtime_clean.sh:29-40` suppresses Docker
  errors and leaves `docker_status=PASS`; a Docker permission failure can
  therefore be reported as a clean runtime.
- Current axis/tilt evidence contains absolute `/home/lin/...` paths and is not
  portable to this workspace. Its verifier self-test is not a valid current
  acceptance result until those paths are made portable.

## Wall-Crossing Follow-Up Boundary

The 2026-07-10 screenshot classified the observed failure as item 1 below. The
case evidence and first fix are recorded in
`docs/go2_xt16_wall_crossing_case_20260710.md`.

The investigation categories remain:

1. global path crosses a mapped wall;
2. global path is valid but the local plan crosses or ignores the wall;
3. both paths are valid but the executed robot footprint contacts the wall; or
4. map/localization alignment makes a valid path appear to cross the wall.

The revised implementations for NAV-REV-01, NAV-REV-02, NAV-REV-03, and
NAV-REV-07 are in the current working tree, but revised field no-Sport-output
evidence is still required. Until that passes, wall investigation should remain
map/log/RViz or no-Sport-output only.

## Remediation Order

1. Complete revised field no-Sport-output acceptance for NAV-REV-01,
   NAV-REV-02, NAV-REV-03, and NAV-REV-07.
2. Add a saved-map replay test using a retained path/dGraph/map debug bag.
3. Complete supervised lifecycle acceptance for NAV-REV-03.
4. Add height/slope/drop and surface-layer constraints for NAV-REV-06.
5. Validate narrow passages and wall-parallel clearance for NAV-REV-07.
6. Decide and implement the runtime role of the mouth LiDAR for NAV-REV-04 and
   NAV-REV-05.
7. Make map provenance and promotion checks satisfy NAV-REV-09.
8. Run repeatable dry-run and supervised acceptance for NAV-REV-10.

Do not mark this baseline ready for autonomous live use merely because the
software starts or publishes a path. Readiness requires the relevant findings
to be closed with reproducible evidence.
