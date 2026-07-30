# DDDMR Known Glass Filter

This ROS 2 package implements the fixed-site glass solution. It does not try to
recognize unknown glass. A versioned YAML file describes bounded glass polygons
in a fixed frame. For each XT16 return, the filter tests whether the measured
ray crosses one of those polygons before reaching the endpoint.

The package is disabled by default. When enabled, transform or odometry
failures drop the affected cloud instead of silently inserting unfiltered ghost
points into a map.

Executables:

- `glass_plane_filter`: filter `/lidar_points` using known polygons;
- `analyze_glass_bag`: read-only intensity and same-direction return analysis;
- `export_odom_cloud`: create a downsampled odom-frame PCD for selection;
- `fit_glass_plane`: fit a bounded rectangle from a carefully selected PCD.

The default `glass_surface` action replaces a return behind glass with its
glass-plane intersection. This removes the indoor/reflected endpoint while
retaining the facade as an obstacle. The alternative `nan` action only removes
the endpoint and must be paired with a separately validated glass obstacle or
no-entry layer.

See `docs/go2_xt16_known_glass_filter.md` in the repository for the complete
capture, calibration, replay, and acceptance workflow.
