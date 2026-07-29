# Go2 XT16 Global Relocalization

## Behavior

`go2_xt16_navigation.launch` enables fail-closed local recovery and gated
global relocalization through `config/go2_xt16_relocalization.yaml`.

The localization lifecycle is:

```text
UNINITIALIZED -> LOCALIZING -> TRACKING
                       |          |
                       v          v
                      LOST <------+
                       |
                       +---- local recovery ----> LOCALIZING
                       |
                       +---- operator-confirmed or local-recovery-failed
                             global search -----> LOCALIZING
```

- `UNINITIALIZED`: waiting for the complete pose-graph map, key poses, odometry,
  and live LeGO-LOAM feature clouds.
- `LOCALIZING`: a fixed/local seed or a fully gated global candidate has
  seeded MCL hypotheses, but consecutive observations have not proved
  convergence yet.
- `TRACKING`: the published pose's own match ratio/residual, particle
  x/y/z/roll/pitch/yaw spread, `map -> odom` tilt, local ground-normal error,
  observed base height, and map-ground pose height have passed the configured
  thresholds for several distinct feature frames.
- `LOST`: match quality/spread, LiDAR freshness, odometry freshness, or the
  localization timeout failed. Navigation is stopped and the last trusted
  pose is preserved. A feature timeout never requests a global jump.

Only a fresh `TRACKING` state together with a fresh `HEALTHY` geometry state,
fresh `/mcl_pose`, and a non-empty latched `/weighted_ground` static layer
permits the Go2 command gate to forward a nonzero command. Every other state
produces a zero `/dddmr_go2/safe_cmd_vel` output.

At startup, a fixed-pose mission always uses its configured or recorded local
seed. It does not run a global search before `/initial_3d_pose`.

## Immutable Keyframe Synchronization

The deployed map is configured for exactly 169 keyframes. Submaps and global
localization remain unavailable until all 169 responses pass validation:

- at most one keyframe service request is in flight;
- each callback is bound to its requested index and writes only that
  preallocated slot;
- duplicate, out-of-range, stale-generation, and out-of-order responses are
  rejected;
- map-frame and `base_link` clouds must have equal sizes and sampled points
  must agree with the immutable key-pose transform;
- any failed slot leaves localization fail-closed.

Submap warmups carry generation numbers. A superseded warmup is discarded, and
switching copies the warmup clouds then rebuilds the current KD-trees from
those copied clouds; KD-tree objects are never copied across generations.

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
   bounded sparse feature observation against the validated feature map.
4. Rejects hypotheses that fail feature, surface, residual, or odometry
   continuity gates.
5. Requires the best distinct candidate basin to beat the second basin by the
   configured margin and win on three distinct synchronized feature frames.
6. Seeds an expanded particle set around the retained basin.
7. Refines against consecutive LiDAR feature frames and evaluates the final
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
/feature_stream_metrics std_msgs/msg/String
/global_localization   std_srvs/srv/Trigger
```

Monitor state and quality:

```bash
ros2 topic echo /localization_status
ros2 topic echo /localization_health
ros2 topic echo /localization_quality
ros2 topic echo /localization_residual
ros2 topic echo /feature_stream_metrics
```

Explicitly confirm and request a new full-map search without restarting
navigation:

```bash
ros2 service call /global_localization std_srvs/srv/Trigger '{}'
```

Calling the service immediately blocks motion. Motion remains blocked until a
new localization reaches `TRACKING`. Without this operator confirmation,
global search is permitted only after a local recovery around the last trusted
pose has failed.

## Configuration

The Go2-specific parameters are isolated in:

```text
src/dddmr_beginner_guide/config/go2_xt16_relocalization.yaml
```

Important groups:

- Search coverage and cost: `global_localization_grid`,
  `global_localization_div_yaw`, and `global_localization_max_candidates`.
- Coarse acceptance: feature/surface thresholds, first/second basin margin,
  odometry continuity limits, and `global_localization_confirmation_*`.
- Local-first recovery: `local_recovery_std_*` and
  `local_recovery_timeout_sec`.
- 2.5D ground reference: `flat_ground.*`.
- Refinement: `global_localization_top_candidates`,
  `global_localization_num_particles`, and `global_localization_seed_std_*`.
- State hysteresis and geometry limits: `localization_tracking_*` and
  `localization_lost_*`.
- Freshness and observability: `localization_sensor_timeout_sec`,
  `/feature_stream_metrics`, and the command gate's
  `localization_status_timeout_sec` and `localization_health_timeout_sec`.

Do not loosen thresholds merely to obtain `TRACKING`. First inspect map/scan
overlap, `/localization_quality`, particle spread, TF freshness, and XT16
feature quality. Repetitive geometry may remain ambiguous and should stay in
`LOCALIZING` until observations separate the hypotheses.

## Historical Map Gravity Check

A previous read-only local-plane calculation on a 96-pose mouth map found:

- mean ground normal tilt: `2.752 deg`;
- local ground tilt: median `4.527 deg`, p95 `7.723 deg`, maximum `9.273 deg`;
- adjacent local-normal change: p95 `9.405 deg`;
- local plane residual: median `0.0169 m`;
- saved pose-to-ground height: median `0.348 m`, p05/p95
  `0.289/0.420 m`.

Those measurements showed local deformation, not only one uniform
gravity-frame rotation. They are retained as historical calibration evidence;
they do not validate the deployed 169-keyframe snapshot. The deployed map must
pass the same no-motion geometry checks before autonomous use.

## No-Motion Validation

The repository includes an ASCII-PCD keyframe publisher that never emits a
velocity or Unitree request:

```bash
python3 scripts/replay_pose_graph_keyframe_for_relocalization.py \
  /root/dddmr_bags/<pose_graph_directory> \
  --keyframe 20 --duration 8
```

Legacy 43-keyframe replay results do not validate the deployed 169-keyframe
snapshot. Re-run the no-motion replay against that exact immutable snapshot,
including a feature-timeout check, local-first recovery check, ambiguous
candidate rejection, and three-distinct-frame global confirmation. Offline
replay does not replace a supervised, no-goal live acceptance test for XT16
timing, environmental change, and threshold calibration.

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
7. Under supervision and with no navigation goal, induce a recoverable
   localization loss; confirm local recovery is tried first and the command
   gate remains stopped.
8. Test global recovery only after explicit operator confirmation or a
   recorded local-recovery failure; confirm odometry, candidate-margin, and
   three-frame confirmation logs before `LOCALIZING`.
9. Only after the no-motion checks pass should the existing supervised motion
   acceptance process be considered.
