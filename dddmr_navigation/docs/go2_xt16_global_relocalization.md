# Go2 XT16 Global Relocalization

## Behavior

`go2_xt16_navigation.launch` enables automatic global relocalization through
`config/go2_xt16_relocalization.yaml`. At startup, MCL waits for the complete
map, odometry, and live features, then searches the full pose graph. If tracking
is lost, it automatically retries the search. An operator can still reset the
initial pose with RViz or request a search through the `/global_localization`
service.

The localization lifecycle is:

```text
UNINITIALIZED -> LOCALIZING -> TRACKING
                       |          |
                       v          v
                      LOST <------+
                       |
                       +---- manual pose ------> LOCALIZING
```

- `UNINITIALIZED`: waiting for the map, odometry, and live feature inputs.
- `LOCALIZING`: the normal particle filter is refining its configured or
  operator-provided initial pose, but consecutive observations have not proved
  convergence yet.
- `TRACKING`: match quality, particle XY spread, particle yaw spread, and the
  normal particle count have passed the configured thresholds for several
  distinct feature frames.
- `LOST`: match quality/spread, LiDAR freshness, odometry freshness, or the
  localization timeout failed. Motion stays blocked while automatic global
  search retries; it is released only after localization returns to `TRACKING`.

Only `TRACKING` permits the Go2 command gate to forward a nonzero command.
Every other state, a missing state message, and a stale state message produce a
zero `/dddmr_go2/safe_cmd_vel` output.

## Global Search

The MCL node subscribes to the pose-graph server's transient-local complete
map, ground map, key poses, and ordered keyframe clouds. It then:

1. Samples key-pose positions using `global_localization_grid`.
2. Tests uniformly spaced map-frame yaw hypotheses at every sampled position.
3. Compares a bounded live less-flat surface observation with the individual
   surface cloud saved for each candidate keyframe.
4. Rejects weak or ambiguous surface matches rather than blending different
   corridor locations into one pose.
5. Scores the surviving candidate against a pure pose-graph feature map; dense
   planning surfaces are intentionally excluded from this localization tree.
6. Seeds an expanded particle set around the accepted hypotheses.
7. Refines against consecutive LiDAR feature frames and evaluates the weighted
   pose that will actually be published—not merely the best individual
   particle—before publishing a usable `map -> odom` transform and entering
   `TRACKING`.

This search does not command exploratory rotation or translation. It remains a
stationary/no-motion localization process.

The map, ground, key poses, and per-keyframe clouds form one immutable
snapshot. Keyframe service requests time out and retry, but a replacement map
generation invalidates the snapshot, immediately blocks `TRACKING`/TF, and
requires restarting MCL. Empty keyframe placeholders remain supported; every
non-empty map-frame cloud is checked against its base-link cloud and saved pose
before global localization is enabled.

## Runtime Interfaces

```text
/localization_status   std_msgs/msg/String
/localization_quality  std_msgs/msg/Float32
/global_localization   std_srvs/srv/Trigger
```

Monitor state and quality:

```bash
ros2 topic echo /localization_status
ros2 topic echo /localization_quality
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

- Automatic search: `auto_global_localization` is `true`, so full-map search
  runs at startup and retries after localization enters `LOST`.
- Search coverage and cost: `global_localization_grid`,
  `global_localization_div_yaw`, and `global_localization_max_candidates`.
- Coarse feature acceptance: `global_localization_min_match_ratio`.
- Candidate disambiguation: `global_localization_max_surface_points`,
  `global_localization_surface_match_distance`,
  `global_localization_min_surface_match_ratio`,
  `global_localization_min_surface_match_margin`, and
  `global_localization_surface_candidate_max_drop`.
- Refinement: `global_localization_top_candidates`,
  `global_localization_num_particles`, and `global_localization_seed_std_*`.
- State hysteresis: `localization_tracking_*` and `localization_lost_*`.
- Freshness: `localization_sensor_timeout_sec` and the command gate's
  `localization_status_timeout_sec`.

Do not loosen thresholds merely to obtain `TRACKING`. First inspect map/scan
overlap, `/localization_quality`, particle spread, TF freshness, and XT16
feature quality. Repetitive geometry may remain ambiguous and should stay in
`LOCALIZING` until observations separate the hypotheses.

## Offline No-Motion Replay

The repository includes an ASCII-PCD keyframe publisher that never emits a
velocity or Unitree request:

```bash
python3 scripts/replay_pose_graph_keyframe_for_relocalization.py \
  /root/dddmr_bags/<pose_graph_directory> \
  --keyframe 20 --duration 8
```

For the current mouth map, the updated search was verified in an isolated ROS
domain with only the map server, MCL, saved clouds, and identity odometry. Each
run evaluated 1,080 position/yaw candidates:

- keyframe 0: selected keyframe 0, surface match 1.000, with no other qualified
  pose basin, then `TRACKING` with weighted-pose match 1.000;
- keyframe 20: selected the adjacent sampled keyframe 21 (0.18 m away), surface
  match 0.811, margin 0.419, then `TRACKING` near the saved pose;
- keyframe 35: selected keyframe 35, surface match 0.996, margin 0.624, then
  `TRACKING` with weighted-pose match 1.000.
- after the replay feature stream stopped: the node changed from `TRACKING` to
  `LOST` on the configured LiDAR timeout.
- after a replacement key-pose publication: the immutable snapshot was
  invalidated and the status changed from `TRACKING` to `LOST` before another
  TF could be accepted.

The localization state-machine tests also cover quality loss and explicit
recovery. The current configuration invokes the same global search
automatically at startup and after tracking is lost.

This validates software behavior with saved pose-graph data. It does not
replace a supervised, no-goal live acceptance test for real XT16 timing,
environmental change, and threshold calibration.

## Live Acceptance Order

1. Start navigation in dry-run mode with no goal.
2. Confirm the complete map and key poses arrive.
3. Keep the robot stationary and verify the automatic search changes
   `UNINITIALIZED -> LOCALIZING` without an RViz initial pose.
4. Wait for `TRACKING`, then verify observed features overlap the map and
   inspect `/mcl_pose` covariance.
5. Confirm `/dddmr_go2/safe_cmd_vel` remains zero in `UNINITIALIZED`,
   `LOCALIZING`, and `LOST`.
6. Confirm that `LOST` triggers a new global search while the command gate
   remains blocked, and verify `LOST -> LOCALIZING -> TRACKING` only with
   correct map overlap.
7. Only after the no-motion checks pass should the existing supervised motion
   acceptance process be considered.
