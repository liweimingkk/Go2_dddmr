# DDDMR MAP SERVER

The map server publishes `mapcloud`, `mapsurface`, `mapground`, and
`navigation_ground` with transient-local reliable QoS.
`complete_ground_voxel_size` may be configured independently from
`complete_map_voxel_size`; when omitted it inherits the complete-map value,
preserving existing configurations.

An optional high-resolution ground ROI can be published as
`/<map node name>/terrain_ground`. It is disabled by default. Enable it only
with finite, ordered map-frame bounds:

```yaml
map1:
  ros__parameters:
    complete_map_voxel_size: 0.2
    complete_ground_voxel_size: 0.2
    source_map_sha256: "<64-hex digest reported for verified artifacts>"
    terrain_roi_enabled: true
    terrain_roi_voxel_size: 0.05
    terrain_roi_min_x: -2.0
    terrain_roi_min_y: 1.0
    terrain_roi_min_z: -0.5
    terrain_roi_max_x: 3.0
    terrain_roi_max_y: 6.0
    terrain_roi_max_z: 2.0
```

The ROI voxel size must be positive and no larger than the complete-ground
voxel size. Invalid bounds, invalid resolution, or an ROI containing no ground
points produce no `terrain_ground` or terrain-aware `navigation_ground`
message. The ordinary map topics remain available, so consumers must require a
valid terrain message before enabling terrain-specific navigation.

When the terrain ROI is disabled, `navigation_ground` is identical to the
downsampled `mapground`. When it is enabled and valid, `navigation_ground`
contains the coarse ground outside the ROI plus the high-resolution
`terrain_ground` inside it. ROI replacement removes coarse points within the
ROI, non-finite points, and exact coordinate duplicates. `mapground` remains
unchanged for localization consumers.

## Loaded-map identity

The server computes `/<map node name>/map_sha256` from the exact pose-graph
artifacts it loaded; it never republishes `source_map_sha256` as if that value
had been measured. The deterministic manifest contains `poses.pcd` and every
referenced `pcd/<index>_{feature,surface,ground}.pcd`, ordered by stable path
relative to the pose-graph directory. SHA-256 input is a version marker, the
artifact count, then each logical path and exact file bytes, with paths and
payloads framed by unsigned 64-bit big-endian lengths. Absolute host paths and
timestamps are excluded, so moving an unchanged pose graph does not change its
identity.

When `terrain_roi_enabled` is true, `source_map_sha256` must be a complete
64-hex digest and must exactly match the computed artifact identity. A missing
or mismatched value disables `terrain_ground` and terrain-aware
`navigation_ground` fail-closed; the log reports the measured digest for an
operator to verify before updating configuration. With terrain ROI disabled,
ordinary flat `mapcloud`, `mapground`, and `navigation_ground` publication
remains backward compatible even when the optional configured digest is absent
or stale. If available, the identity topic always reports the measured digest.
