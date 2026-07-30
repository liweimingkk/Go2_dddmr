# Go2 XT16 Known-Glass-Plane Filtering

## Scope and safety

This is the fixed-site solution for a known glass facade. It handles both a
real return transmitted through glass and an apparent reflection endpoint when
the measured straight ray crosses the configured facade first.

The tools in this document do not publish Go2 motion commands. Keep navigation
and every Sport/low-level command adapter stopped while capturing, calibrating,
or replaying. Moving the robot between calibration views remains a separately
supervised operator action.

The filter is deliberately disabled by default. Do not point LeGO-LOAM at the
filtered topic until the configuration has passed offline visualization and
synthetic tests.

## Data flow

```text
known odom-frame polygons -----------+
standardized odometry ----+          |
base_link -> XT16 static TF          v
/lidar_points ----------------> glass_plane_filter
                                      |             \
                                      |              +-> /glass_plane_obstacles
                                      v
                         /lidar_points_glass_filtered
                                      |
                                      v
                                  LeGO-LOAM
```

The recommended `glass_surface` action replaces each endpoint behind glass
with the ray/plane intersection. Point count, field layout, ring, timestamp,
and message dimensions stay unchanged. LeGO-LOAM receives one range-image
candidate at the glass instead of the indoor or reflected endpoint.

If odometry or the static sensor transform is unavailable while filtering is
enabled, the filter drops that cloud. It never silently falls back to the raw
cloud.

## Build

Use the repository's Docker build path:

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh build-navigation
```

`build-navigation` now includes `dddmr_glass_filter`.

## Analyze an existing bag

This command reads the bag without publishing topics. It samples frames,
reports the input contract and intensity distribution, and checks for two
different ranges in the same ring/azimuth bin:

```bash
ros2 run dddmr_glass_filter analyze_glass_bag \
  --bag /root/dddmr_bags/BAG_DIRECTORY \
  --topic /lidar_points \
  --horizontal-bins 2000 \
  --sample-every 100 \
  --output-dir /root/dddmr_bags/BAG_DIRECTORY_glass_analysis
```

The output directory contains:

- `summary.json`;
- `range_image.pgm`;
- `intensity_image.pgm`;
- `return_separation_image.pgm`.

The PGM images are diagnostic range-image rasters. Zero-valued pixels mean no
sample. The separation image should be empty for the currently recorded
single-return stream.

## Tomorrow's bounded, read-only capture

At each pose, keep the robot stationary and record 20-30 seconds. Enter the
return mode exactly as shown by the XT16 configuration page:

```bash
cd /home/lin/new2/dddmr_navigation

./scripts/capture_go2_xt16_glass_calibration.sh \
  --label normal --duration 30 --return-mode "REPLACE_WITH_OBSERVED_MODE"

./scripts/capture_go2_xt16_glass_calibration.sh \
  --label angle30 --duration 30 --return-mode "REPLACE_WITH_OBSERVED_MODE"

./scripts/capture_go2_xt16_glass_calibration.sh \
  --label angle60 --duration 30 --return-mode "REPLACE_WITH_OBSERVED_MODE"
```

Before recording, the helper verifies real `/lidar_points` samples and the
`/utlidar/robot_odom` type. It always records those two topics and adds
`/lidar_packets`, `/tf`, and `/tf_static` when discovered. It neither starts
mapping/navigation nor publishes a command topic.

Test discovery without writing a bag:

```bash
./scripts/capture_go2_xt16_glass_calibration.sh \
  --label normal --return-mode "REPLACE_WITH_OBSERVED_MODE" --dry-run
```

## Plane configuration

Use a fixed coordinate frame backed independently by odometry during mapping.
For the current Go2 path this should normally be `odom`, with the filter reading
`/dddmr_go2/robot_odom_standard`. Do not use `base_link`; that would make the
facade move with the robot.

Each polygon must be convex, coplanar, and ordered clockwise or
counter-clockwise. Example:

```yaml
version: 1
frame_id: odom
planes:
  - id: west_facade
    vertices:
      - [5.0, -2.0, -0.5]
      - [5.0,  2.0, -0.5]
      - [5.0,  2.0,  3.0]
      - [5.0, -2.0,  3.0]
    minimum_behind_distance: 0.15
```

The repository example contains placeholder coordinates and must never be used
unchanged on a real map:

```text
src/dddmr_glass_filter/config/glass_planes.example.yaml
```

### Export a fixed-frame cloud for selection

Create an aggregate cloud directly in the recorded odometry frame. The default
base-to-XT16 transform is the composed transform from the currently documented
`base_link -> go2_imu -> hesai_lidar` chain. If tomorrow's read-only TF check
does not match it, pass the measured translation/RPY explicitly instead of
using the defaults.

```bash
ros2 run dddmr_glass_filter export_odom_cloud \
  --bag /root/dddmr_bags/go2_xt16_glass_normal_TIMESTAMP \
  --output /root/dddmr_bags/glass_normal_odom_cloud.pcd \
  --odometry-topic /utlidar/robot_odom \
  --odom-frame odom \
  --sample-every 5 \
  --maximum-clouds 60 \
  --voxel-size 0.10
```

The exporter performs nearest-timestamp odometry matching with a 0.05 s limit,
range filtering, rigid transformation, and voxel downsampling. It refuses to
overwrite an output file and reports matched/skipped cloud counts. Review the
export in RViz or a PCD editor, then select only the facade plane.

### Fit from an RViz PCD selection

Select only points belonging to the glass/frame plane, not the ghost wall or
unrelated ground. The selection coordinates must already be expressed in the
same fixed frame written to `--frame-id`.

```bash
ros2 run dddmr_glass_filter fit_glass_plane \
  --selection-pcd /root/dddmr_bags/glass_surface_selection.pcd \
  --output /root/dddmr_bags/site_glass_planes.yaml \
  --frame-id odom \
  --plane-id west_facade \
  --distance-threshold 0.05 \
  --minimum-inlier-ratio 0.60 \
  --padding 0.10
```

The fitter refuses to overwrite an existing file and reports the inlier ratio,
normal, four fitted vertices, and residual statistics. Reject the fit if the
selected PCD mixes facade and ghost-wall points or if its 95th-percentile
residual is not small relative to the chosen threshold.

## Start the filter without motion

Terminal 1:

```bash
ros2 launch dddmr_glass_filter glass_plane_filter.launch.py \
  enabled:=true \
  plane_config_file:=/root/dddmr_bags/site_glass_planes.yaml \
  pose_source:=odometry \
  odometry_topic:=/dddmr_go2/robot_odom_standard \
  filtered_point_action:=glass_surface
```

Terminal 2 starts mapping with the explicit filtered input. Its odometry
standardizer and static XT16 transforms also satisfy the filter's pose inputs:

```bash
ros2 launch lego_loam_bor lego_loam_go2_xt16_live.launch \
  xt16_topic:=/lidar_points_glass_filtered \
  rviz:=true
```

For bag replay, start both launches first and then replay the captured topics.
Do not use this as the first validation: initially subscribe to and visualize
the raw, filtered, and `/glass_plane_obstacles` clouds without saving or
navigating.

## Required offline acceptance

Compare raw and filtered clouds at all three angles and require:

1. filtered message dimensions and fields remain identical to the raw XT16
   message;
2. output cadence stays close to the 10 Hz input contract;
3. red-region transmitted/reflected endpoints are replaced at the configured
   facade;
4. ordinary walls, people, vegetation, curbs, and low obstacles whose rays do
   not cross the bounded polygon are unchanged;
5. points just in front of and on the glass are not removed;
6. missing odometry/TF causes visible dropped-frame errors, not raw fallback;
7. the configured facade is reviewed as an obstacle or converted into a
   separately tested no-entry layer before navigation.

Only after these checks should an offline replay save a candidate pose graph.
Compare that candidate with the unfiltered map before any supervised motion
test.

## Current limitations

- This is not automatic glass recognition. Moving or previously unknown glass
  is outside its model.
- XYZ fields must be scalar `FLOAT32`, and padded PointCloud2 rows are rejected.
- Plane accuracy depends on odometry, the XT16 static extrinsic, and careful
  selection in the same fixed frame.
- `/glass_plane_obstacles` is a reference/visualization cloud. The active Go2
  navigation configuration does not yet load `NoEntryLayer`; enabling that is
  a separate navigation-safety change.
- The latest saved map cannot be deterministically re-filtered without the
  corresponding raw lidar and odometry recording.
