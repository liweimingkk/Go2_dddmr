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

## Stationary Test Assumption

The current `base_link` z-window filter is a stationary-only setup for the Go2
standing pose:

- `mouth_ground_z_min: -0.42`
- `mouth_ground_z_max: -0.18`

Do not treat this as a walking-mapping filter until one of these is true:

- a stable `base_footprint` or another roll/pitch-compensated
  `mouth_filter_frame` is implemented and validated; or
- the current `base_link` z ROI is validated in RViz under expected
  pitch/roll/body-height changes.

Before walking mapping, explicitly verify in RViz that `/mouth_ground_cloud`
remains ground-only under the expected body pitch, roll, and height changes.

## Time Synchronization

The reusable YAML defaults to `header_offset` mode for deterministic bag or
direct-handler replay. In that mode fusion uses corrected mouth timestamps:

```text
corrected_mouth_stamp = raw_mouth_stamp + mouth_time_offset_sec
```

The corrected stamp is used both for nearest-frame matching and stamped TF
lookup. The live launch explicitly overrides this with:

```yaml
mouth_sync_mode: receipt_time
mouth_time_offset_sec: 0.0
mouth_max_time_diff: 0.03
```

`receipt_time` pairs the two clouds by their local steady-clock arrival times.
It avoids depending on the sensor header-clock epoch after a link reconnect,
and ignores `mouth_time_offset_sec`. The `0.03` second value is both the pairing
tolerance and the maximum wait for a newer mouth sample.

This path does not compensate robot motion between the two cloud times. Keep
the robot stationary while validating the fixed-z mouth filter, and do not
increase the live tolerance for walking mapping.

When explicitly using `header_offset` mode for replay or diagnosis, measure the
current header relationship first:

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
network, or robot runtime state changes. Do not inject this value into live
`receipt_time` mode.

Timestamp offset alone is not enough for deployment. With the robot stationary,
move a box or card quickly in front of the mouth LiDAR and verify in RViz that
the mouth point cloud response is visually real-time. Do not use
`mouth_time_offset_sec` as deployment calibration until this content-latency
check shows negligible lag.

The separate validation smoke script may use a larger stationary-only window
for geometry diagnosis. That override is not a deployment or walking-mapping
setting.

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
