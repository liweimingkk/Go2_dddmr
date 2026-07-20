# Go2 XT16 Mouth Ground Fusion Notes

This note tracks the current local validation status for
`lego_loam_go2_xt16_mouth.launch`.

## Validated Live Input

The current Go2 live graph was validated with:

- main XT16 cloud: `/lidar_points`
- XT16 frame: `hesai_lidar`
- mouth ground source: `/utlidar/cloud_base`
- mouth source frame: `base_link`
- mouth filter frame: `base_link`
- mouth frame override: empty by default, because `/utlidar/cloud_base` already
  publishes `frame_id=base_link`

The raw `/utlidar/cloud` path is not currently validated in this workspace. Its
live `header.frame_id` was observed as `utlidar_lidar`, but that frame was not
available in the current TF tree. Use raw `/utlidar/cloud` only after adding and
checking the correct `utlidar_lidar` transform.

## Stair-Surface Classification

The Go2 mouth mapping config now uses:

```yaml
mouth_ground_mode: connected_surface
```

This mode does not limit every accepted point to one fixed `base_link` height.
It first finds locally planar support patches. A seed must be below and
immediately ahead of the robot and must agree with the current XT16
`patched_ground`; the mouth LiDAR therefore extends the roof-LiDAR ground map
instead of inventing an unrelated floating surface. It then follows
independently sized patches through the configured Go2 step-capability
envelope. A floor plus at least two successive tread heights is accepted as a
stair chain. A single low box top, an unsupported horizontal platform, a
narrow patch, a riser, and an excessive slope remain non-ground.

`mouth_ground_z_min/max` are retained only for the explicit `fixed_z`
stationary troubleshooting fallback. The connected mode instead uses the
broad `mouth_support_seed_*` footprint to identify the current support surface;
it does not encode the temporary test stair's rise or total physical step
count. `mouth_minimum_stair_height_levels` is a conservative evidence count,
not the expected staircase length.

Accepted mouth points are appended to the existing `patched_ground` path and
are saved in `ground.pcd/mapground`. Rejected mouth points enter the existing
surface-keyframe path. The Go2 map server removes surface points overlapping
`mapground` and merges the remainder into the existing `mapcloud`. No
`mapterrain` or third navigation map is created.

Before walking mapping, explicitly verify in RViz that `/mouth_ground_cloud`
contains treads but not risers or nearby boxes under expected body attitude
changes.

## Time Synchronization

Strict fusion uses corrected mouth timestamps:

```text
corrected_mouth_stamp = raw_mouth_stamp + mouth_time_offset_sec
```

The corrected stamp is used both for nearest-frame matching and stamped TF
lookup. Default:

```yaml
mouth_time_offset_sec: 0.0
mouth_max_time_diff: 0.08
```

In live testing after old containers were stopped, `/utlidar/cloud_base` header
timestamps lagged `/lidar_points` by about 2.5 seconds. With
`mouth_time_offset_sec:=0.0` and `mouth_max_time_diff:=0.08`, mouth points were
skipped.

Only use an offset such as the current measured `mouth_time_offset_sec:=2.397`
after confirming the offset is stable in the current live graph. Final
deployment should keep
`mouth_max_time_diff` around `0.08-0.15` after timestamps are fixed.

The mapping-and-save script now performs this measurement automatically and
refuses to save until it receives a non-empty `/mouth_ground_cloud` sample.
For a manual check, run:

```bash
source /opt/ros/humble/setup.bash
source scripts/setup_go2_dds_env.sh
python3 scripts/measure_go2_mouth_xt16_time_offset.py \
  --mouth-topic /utlidar/cloud_base \
  --xt16-topic /lidar_points \
  --duration 8
```

Use the reported `recommended_mouth_time_offset_sec` only when
`OFFSET_STABLE_FOR_SMOKE=True`. Recheck again if any driver, container, clock,
network, or robot runtime state changes.

Timestamp offset alone is not enough for deployment. With the robot stationary,
move a box or card quickly in front of the mouth LiDAR and verify in RViz that
the mouth point cloud response is visually real-time. Do not use
`mouth_time_offset_sec` as deployment calibration until this content-latency
check shows negligible lag.

For geometry-only perception tests on the current live graph, use:

```bash
mouth_max_time_diff:=3.0
```

Do not use `mouth_max_time_diff:=3.0` for motion or walking mapping. It is only
a stationary smoke-test override for checking ROI geometry and frame transforms.

## Launch Integration

`lego_loam_go2_xt16_mouth.launch` defaults to:

```text
publish_static_tf:=false
standardize_odom:=false
odom_topic:=/utlidar/robot_odom
standardized_odom_topic:=/dddmr_go2/robot_odom_standard
```

Use `publish_static_tf:=false` when `robot_state_publisher` or another trusted
Unitree TF source is active. Use `publish_static_tf:=true` only for standalone
Docker smoke tests that would otherwise have no trusted TF source.

`go2_odom_standardizer` is not a deployment default. Validate x-forward, yaw,
and twist signs separately before using it with DDDMR mapping. If an existing
single-XT16 odom topic already works, keep that path first.

If `standardize_odom:=true` is used after separate validation, also set:

```bash
odom_topic:=/dddmr_go2/robot_odom_standard
```

The standardizer publishes to `standardized_odom_topic`; it refuses to run if
its input and output topics are identical.

The current standardizer rotates pose and twist but copies covariance unchanged.
Do not rely on standardized covariance downstream until covariance rotation is
implemented and validated.

## Safety Boundary

These checks are perception-only. Do not publish `/cmd_vel` or start autonomous
navigation closed loop until the mouth ground cloud is inspected in RViz and the
map/ground outputs are accepted.
