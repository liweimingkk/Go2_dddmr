# Vendored SCAN-Planner source

This directory vendors the planner-only ROS 2 packages from:

- Repository: https://github.com/wuyi2121/SCAN-Planner
- Branch: `ros2-community`
- Commit: `d0b921c9b05a6d291d144d60882b2e0e88d2c0e0`
- Imported: 2026-07-23

Only `src/planner/` was imported. Simulator packages, maps, meshes, and media
assets are intentionally excluded because the DDDMR workspace supplies its own
Go2 localization, perception, mapping, visualization, and command-safety
chain.

The imported source is semantically unmodified; CRLF line endings and trailing
whitespace were normalized to satisfy repository checks. DDDMR-specific topic,
frame, route, and command adapters live in the separate
`dddmr_scan_planner` package so future upstream updates can be reviewed
separately.

The vendored work is licensed under Apache-2.0. Keep `LICENSE`, `NOTICE`, and
the upstream README with any redistribution.
