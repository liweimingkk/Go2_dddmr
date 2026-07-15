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
- `TRACKING`: match quality, particle XY spread, particle yaw spread, and the
  normal particle count have passed the configured thresholds for several
  distinct feature frames.
- `LOST`: match quality/spread, LiDAR freshness, odometry freshness, or the
  localization timeout failed. Automatic full-map search is requested again.

Only `TRACKING` permits the Go2 command gate to forward a nonzero command.
Every other state, a missing state message, and a stale state message produce a
zero `/dddmr_go2/safe_cmd_vel` output.

## Global Search

The MCL node subscribes to the pose-graph server's transient-local complete
map, ground map, and key poses. It then:

1. Samples key-pose positions using `global_localization_grid`.
2. Tests uniformly spaced map-frame yaw hypotheses at every sampled position.
3. Scores a bounded sparse feature observation against the complete map.
4. Retains the highest-scoring hypotheses.
5. Seeds an expanded particle set around those hypotheses.
6. Refines against consecutive LiDAR feature frames before publishing a usable
   `map -> odom` transform and entering `TRACKING`.

This search does not command exploratory rotation or translation. It remains a
stationary/no-motion localization process.

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

- Search coverage and cost: `global_localization_grid`,
  `global_localization_div_yaw`, and `global_localization_max_candidates`.
- Coarse acceptance: `global_localization_min_match_ratio`.
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
4. Verify observed features overlap the map and inspect `/mcl_pose` covariance.
5. Confirm `/dddmr_go2/safe_cmd_vel` remains zero in `UNINITIALIZED`,
   `LOCALIZING`, and `LOST`.
6. Under supervision and with no navigation goal, relocate the robot to a
   different mapped area; confirm `LOST -> LOCALIZING -> TRACKING` and correct
   map overlap.
7. Only after the no-motion checks pass should the existing supervised motion
   acceptance process be considered.
