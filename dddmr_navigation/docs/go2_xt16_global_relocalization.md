# Go2 XT16 Global Relocalization

## Behavior

`go2_xt16_navigation.launch` now enables automatic global relocalization through
`config/go2_xt16_relocalization.yaml`.

The localization lifecycle is:

```text
UNINITIALIZED -> LOCALIZING -> TRACKING
                       |          |
                       v          v
                      LOST <------+
                       |
                       +---- global search ----> LOCALIZING
```

- `UNINITIALIZED`: waiting for the complete pose-graph map, key poses, odometry,
  and live LeGO-LOAM feature clouds.
- `LOCALIZING`: the full-map coarse search has seeded multiple MCL hypotheses,
  but consecutive observations have not proved convergence yet.
- `TRACKING`: the published pose's own match ratio/residual, particle
  x/y/z/roll/pitch/yaw spread, `map -> odom` tilt, local ground-normal error,
  observed base height, and map-ground pose height have passed the configured
  thresholds for several distinct feature frames.
- `LOST`: match quality/spread, LiDAR freshness, odometry freshness, or the
  localization timeout failed. Automatic full-map search is requested again.

Only a fresh `TRACKING` state together with a fresh `HEALTHY` geometry state
permits the Go2 command gate to forward a nonzero command. Every other state,
missing/stale state or health messages, and every failed geometry check produce
a zero `/dddmr_go2/safe_cmd_vel` output.

## Flat-floor 2.5D State

The Go2 profile enables `flat_ground.enabled`. MCL estimates only map-frame
`x`, `y`, and yaw:

- particle roll/pitch is composed from the current odometry/IMU gravity
  orientation, so `map -> odom` is yaw-only;
- particle z is the local map-ground plane height plus the configured measured
  `base_link` clearance;
- likelihood ground weighting evaluates the ground-reference point below
  `base_link`, rather than pulling the `base_link` origin onto map ground;
- live ground height/normal is estimated with a gravity-constrained RANSAC
  plane after rejecting features outside the expected ground-height window.

For this Go2, stationary live observations measured a base height near
`0.33 m`; the profile uses `flat_ground.base_link_height: 0.32`.

## Global Search

The MCL node subscribes to the pose-graph server's transient-local complete
map, ground map, and key poses. It then:

1. Samples key-pose positions using `global_localization_grid`.
2. Tests uniformly spaced map-frame yaw hypotheses at every sampled position.
3. Constrains each hypothesis to local map ground plus base height and scores a
   bounded sparse feature observation against the complete map.
4. Rejects hypotheses whose own match ratio or own mean nearest-neighbor
   residual fails acceptance, then retains the lowest-residual hypotheses.
5. Seeds an expanded particle set around those hypotheses.
6. Refines against consecutive LiDAR feature frames and evaluates the final
   weighted pose itself (not the best hit ratio of any particle) before
   publishing a usable `map -> odom` transform and entering `TRACKING`.

This search does not command exploratory rotation or translation. It remains a
stationary/no-motion localization process.

## Runtime Interfaces

```text
/localization_status   std_msgs/msg/String
/localization_health   std_msgs/msg/String
/localization_quality  std_msgs/msg/Float32
/localization_residual std_msgs/msg/Float32
/global_localization   std_srvs/srv/Trigger
```

Monitor state and quality:

```bash
ros2 topic echo /localization_status
ros2 topic echo /localization_health
ros2 topic echo /localization_quality
ros2 topic echo /localization_residual
```

Force a new full-map search without restarting navigation:

```bash
ros2 service call /global_localization std_srvs/srv/Trigger '{}'
```

Calling the service immediately blocks motion. Motion remains blocked until a
new localization reaches `TRACKING`.

## Configuration

The Go2-specific parameters are isolated in:

```text
src/dddmr_beginner_guide/config/go2_xt16_relocalization.yaml
```

Important groups:

- Search coverage and cost: `global_localization_grid`,
  `global_localization_div_yaw`, and `global_localization_max_candidates`.
- Coarse acceptance: `global_localization_min_match_ratio` and
  `global_localization_max_residual`.
- 2.5D ground reference: `flat_ground.*`.
- Refinement: `global_localization_top_candidates`,
  `global_localization_num_particles`, and `global_localization_seed_std_*`.
- State hysteresis and geometry limits: `localization_tracking_*` and
  `localization_lost_*`.
- Freshness: `localization_sensor_timeout_sec` and the command gate's
  `localization_status_timeout_sec` and `localization_health_timeout_sec`.

Do not loosen thresholds merely to obtain `TRACKING`. First inspect map/scan
overlap, `/localization_quality`, particle spread, TF freshness, and XT16
feature quality. Repetitive geometry may remain ambiguous and should stay in
`LOCALIZING` until observations separate the hypotheses.

## Current Map Gravity Check

A read-only local-plane calculation on the current 96-pose mouth map found:

- mean ground normal tilt: `2.752 deg`;
- local ground tilt: median `4.527 deg`, p95 `7.723 deg`, maximum `9.273 deg`;
- adjacent local-normal change: p95 `9.405 deg`;
- local plane residual: median `0.0169 m`;
- saved pose-to-ground height: median `0.348 m`, p05/p95
  `0.289/0.420 m`.

The varying normal direction and large adjacent changes show local deformation,
not only one uniform gravity-frame rotation. A whole-map rotation can remove
the mean `2.752 deg` component but cannot make all local planes level. Flat-floor
operation should therefore keep the new geometry gate enabled and re-constrain
or rebuild this map before relying on autonomous motion through the distorted
regions. No map-audit utility is installed by this change.

## Offline No-Motion Replay

The repository includes an ASCII-PCD keyframe publisher that never emits a
velocity or Unitree request:

```bash
python3 scripts/replay_pose_graph_keyframe_for_relocalization.py \
  /root/dddmr_bags/<pose_graph_directory> \
  --keyframe 20 --duration 8
```

For the current 43-keyframe mouth map, the implemented pipeline was verified
offline as follows:

- keyframe 20: 612 coarse position/yaw candidates, best coarse quality 0.925,
  then `TRACKING` near the saved keyframe pose;
- feature stream stopped: transition to `LOST`, with the command gate retaining
  zero output;
- keyframe 35 replayed several metres away: a new full-map search selected the
  second location and returned to `TRACKING`.

This validates software behavior with saved pose-graph data. It does not
replace a supervised, no-goal live acceptance test for real XT16 timing,
environmental change, and threshold calibration.

## Live Acceptance Order

1. Start navigation in dry-run mode with no goal.
2. Confirm the complete map and key poses arrive.
3. Keep the robot stationary and wait for `TRACKING`.
4. Require `/localization_health == HEALTHY`, inspect
   `/localization_residual`, and verify `map -> odom` roll/pitch remain zero.
5. Verify observed features overlap the map and inspect `/mcl_pose` covariance
   and `base_link` height above local map ground.
6. Confirm `/dddmr_go2/safe_cmd_vel` remains zero in `UNINITIALIZED`,
   `LOCALIZING`, and `LOST`.
7. Under supervision and with no navigation goal, relocate the robot to a
   different mapped area; confirm `LOST -> LOCALIZING -> TRACKING` and correct
   map overlap.
8. Only after the no-motion checks pass should the existing supervised motion
   acceptance process be considered.
