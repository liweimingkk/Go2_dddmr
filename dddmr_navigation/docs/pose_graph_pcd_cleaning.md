# Pose-Graph PCD Cleaning

The upstream RViz PCD deleting tool edits one aggregate `map.pcd` and
`ground.pcd`. DDDMR navigation instead rebuilds its map from every
`pcd/<index>_feature.pcd`, `pcd/<index>_surface.pcd`, and saved pose. Use the
pose-graph adapter so a global RViz selection is applied to all contributing
keyframes.

The exported-PCD workflow below is offline and publishes no velocity or
Unitree request. Do not run `pose-graph-editor` alongside navigation, because
its PCD publisher uses the global `mapcloud` and `mapground` topic names. The
direct map-server editor in the next section is subscriber-only, but robot
motion should still be disabled while selecting and reviewing map points.

## Optional: select directly from `/map1/mapcloud`

When the pose-graph map server is already publishing `/map1/mapcloud`, the
selection can be made directly against that cloud:

```bash
./dddmr_navigation/scripts/dddmr_docker_go2_xt16.sh mapcloud-editor
```

If `/map1/mapcloud` has no publisher, this command starts only the pose-graph
map server from `go2_xt16_navigation.yaml`. It does not start localization,
planning, velocity output, or the Go2 Sport adapter.

The optional positional arguments select other map-server topics:

```bash
./dddmr_navigation/scripts/dddmr_docker_go2_xt16.sh mapcloud-editor \
  /map1/mapcloud /map1/mapground
```

This entry opens below the map (`Pitch: -0.85`), hides the ground reference,
and renders editable points at one pixel to reduce accidental selection.
Activate `DeletePointCloud` or press `S`, zoom in, and use several small
rectangles. Selected points appear in magenta. Press `Z` immediately if a
rectangle includes unrelated points.

The editor subscribes to the map-server topics but does not modify them or the
source pose graph. `Save Modified PointCloud` exports a
`<timestamp>_deleted_points.pcd`; use that selection with `inspect` and
`clean-copy` below. Do not replace a pose-graph directory with the aggregate
`remaining_map.pcd`.

For a live map-server selection, pass the server's
`complete_map_voxel_size` as `--selection-voxel-size`. The current Go2 XT16
configuration uses `0.20`:

```bash
python3 dddmr_navigation/scripts/pose_graph_pcd_cleaner.py inspect \
  --map-dir /path/to/source_pose_graph \
  --selection-pcd /path/to/timestamp_deleted_points.pcd \
  --selection-voxel-size 0.20
```

## 1. Build the editor

```bash
cd /root/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh build-navigation
```

`build-navigation` includes the custom point-selection tool and the
`map_delete_panel` export panel.

## 2. Export a global editor cloud

Run this on the host, using host paths:

```bash
python3 dddmr_navigation/scripts/pose_graph_pcd_cleaner.py export-editor \
  --map-dir /path/to/source_pose_graph \
  --output /path/to/editor_clouds
```

The output contains `editor_map.pcd`, combining feature and surface keyframes
in the map frame, and a non-selectable `editor_ground.pcd` reference.

## 3. Select unwanted points in RViz

Pass paths as seen inside the Docker container. The repository Docker wrapper
mounts the host bags directory at `/root/dddmr_bags`.

```bash
./dddmr_navigation/scripts/dddmr_docker_go2_xt16.sh pose-graph-editor \
  /root/dddmr_bags/editor_clouds/editor_map.pcd \
  /root/dddmr_bags/editor_clouds/editor_ground.pcd
```

In RViz:

1. Activate `DeletePointCloud` in the toolbar or press `S`.
2. Drag a left-button rectangle around unwanted map points. Hold `Alt` to move
   the camera.
3. Use `Last Step`/`Z` to undo or `Clear Selection`/`C` to reset.
4. Click `Save Modified PointCloud` and choose a mounted output directory.

The panel saves the visual previews plus
`<timestamp>_deleted_points.pcd`. The latter is the global selection consumed
by the pose-graph adapter.

## 4. Inspect and create a cleaned copy

The RViz publisher uses 0.10 m voxels. Use the same value when mapping the
selection back to raw keyframes:

```bash
python3 dddmr_navigation/scripts/pose_graph_pcd_cleaner.py inspect \
  --map-dir /path/to/source_pose_graph \
  --selection-pcd /path/to/timestamp_deleted_points.pcd \
  --selection-voxel-size 0.10

python3 dddmr_navigation/scripts/pose_graph_pcd_cleaner.py clean-copy \
  --source /path/to/source_pose_graph \
  --output /path/to/new_cleaned_pose_graph \
  --selection-pcd /path/to/timestamp_deleted_points.pcd \
  --selection-voxel-size 0.10
```

`clean-copy` refuses an existing output directory. It edits only feature and
surface PCDs, then verifies that the top-level `ground.pcd`, every keyframe
ground PCD, `poses.pcd`, and `edges.pcd` are byte-for-byte unchanged. The
output contains `pose_graph_cleaning_report.json` with affected keyframes and
point counts.

For a known three-dimensional artifact, `inspect` and `clean-copy` also accept
an explicit map-frame box:

```bash
--box X_MIN X_MAX Y_MIN Y_MAX Z_MIN Z_MAX
```

Point navigation at the cleaned directory only after reviewing the report and
performing a no-motion map/static-layer validation.
