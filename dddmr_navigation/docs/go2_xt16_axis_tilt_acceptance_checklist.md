# Go2 XT16 Axis/Tilt Acceptance Checklist

This checklist is the remaining handoff path for
`/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md`.

## Boundary

- Keep the Go2 no-motion until a specific supervised motion step is explicitly
  approved onsite.
- Do not publish `/lowcmd`.
- Do not publish `/api/sport/request` except through the supervised live Sport
  scripts after their confirmation gates pass.
- Do not modify Go2 network, services, startup files, firmware, calibration, or
  `/home/unitree/xt16_ws`.
- Docker navigation must not publish the real Sport API. It may publish only the
  isolated DDDMR velocity topics such as `/dddmr_go2/dry_run_cmd_vel`.

## Gate 0: Static Safety Audit

Run:

```bash
cd /home/lin/new2/dddmr_navigation
scripts/check_go2_xt16_manual_gate_status.sh
```

Required evidence:

```text
GO2_XT16_STATIC_SHELL_STATUS=PASS
GO2_XT16_STATIC_PYTHON_STATUS=PASS
GO2_XT16_STATIC_NO_MOTION_DEFAULTS=PASS
GO2_XT16_STATIC_LIVE_GATES=PASS
GO2_XT16_MANUAL_GATE_STATUS=PASS
```

This gate does not source ROS, start Docker, inspect live robot topics, or
publish any topic.

## Gate 1: RViz base_link Front-Object Check

Purpose: prove that a known object physically placed in front of the Go2 appears
on the +X side when RViz fixed frame is `base_link`.

Start observation-only RViz:

```bash
cd /home/lin/new2/dddmr_navigation
RVIZ=true PUBLISH_STATIC_TF=true \
  scripts/dddmr_docker_go2_xt16.sh navigation-live-source \
  start_move_base:=false \
  start_go2_nav_cmd_gate:=false
```

Manual steps:

```text
1. Set RViz Fixed Frame to base_link.
2. Display /lidar_points or the configured XT16 point cloud.
3. Put a clear object directly in front of the dog.
4. Confirm the object appears at base_link +X, not +Y, -X, or -Y.
5. Record a screenshot path and the observed direction.
6. Stop the launch and verify no residual Docker/RViz process remains.
```

Pass evidence:

```text
RVIZ_BASE_LINK_FRONT_OBJECT_STATUS=PASS
RVIZ_FIXED_FRAME=base_link
RVIZ_FRONT_OBJECT_DIRECTION=+X
SCREENSHOT=/absolute/path/to/gate1_base_link_front_object.png
```

`SCREENSHOT` must be an absolute path to an existing non-empty PNG or JPEG file.

This gate is still required even though the numeric base cloud summary already
passed, because the numeric summary cannot replace a known-object visual check.

## Gate 2: Initial Pose Then TF Tilt Recheck

Purpose: prove that the current localization state no longer tilts
`map -> odom` or `map -> base_link` after a human sets the initial pose.

Use the same observation-only RViz launch as Gate 1. In RViz, set the initial
pose on the current map. Then run this read-only checker in another terminal:

```bash
docker run --rm --network=host \
  --volume /home/lin/new2/dddmr_navigation:/root/dddmr_navigation \
  dddmr_go2_xt16:x64 \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh && \
    cd /root/dddmr_navigation && \
    python3 scripts/check_go2_xt16_tf_health.py --timeout-sec 8'
```

Pass evidence:

```text
TF_HEALTH_MAP_ODOM_STATUS=PASS
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=PASS
TF_HEALTH_LOG=<absolute existing output/log path>
```

`TF_HEALTH_LOG` must contain:

```text
TF_HEALTH_MAP_ODOM_STATUS=PASS
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=PASS
```

Current known blocker to clear:

```text
The latest observation-only check recorded map->odom roll/pitch=4.153deg,
above the 3deg threshold, before the required manual initial-pose recheck.
```

## Gate 3: Pass-through Odom Axis Probe

Purpose: prove that the raw odom and standardized odom agree with the physical
forward direction and that the pass-through standardizer has not reintroduced a
90 degree frame rotation.

This gate needs controlled physical displacement. The checker itself is
read-only, but the displacement must be produced only by a safe manual push,
approved supervised low-speed movement, or another explicitly approved onsite
method. Do not use an autonomous navigation goal for this gate.

Start observation-only runtime so `/dddmr_go2/robot_odom_standard` exists:

```bash
cd /home/lin/new2/dddmr_navigation
RVIZ=false PUBLISH_STATIC_TF=true \
  scripts/dddmr_docker_go2_xt16.sh navigation-live-source \
  start_move_base:=false \
  start_go2_nav_cmd_gate:=false
```

During the controlled forward displacement, run both checks:

```bash
docker run --rm --network=host \
  --volume /home/lin/new2/dddmr_navigation:/root/dddmr_navigation \
  dddmr_go2_xt16:x64 \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh && \
    cd /root/dddmr_navigation && \
    python3 scripts/check_go2_odom_axis_consistency.py \
      --odom-topic /utlidar/robot_odom \
      --duration 5 \
      --min-forward 0.05'
```

```bash
docker run --rm --network=host \
  --volume /home/lin/new2/dddmr_navigation:/root/dddmr_navigation \
  dddmr_go2_xt16:x64 \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh && \
    cd /root/dddmr_navigation && \
    python3 scripts/check_go2_odom_axis_consistency.py \
      --odom-topic /dddmr_go2/robot_odom_standard \
      --duration 5 \
      --min-forward 0.05'
```

Pass evidence:

```text
RAW_ODOM_AXIS_STATUS=PASS
STANDARD_ODOM_AXIS_STATUS=PASS
RAW_STANDARD_ODOM_MATERIAL_MATCH=true
ODOM_AXIS_ROTATION_CHECK=no_90deg_rotation
RAW_ODOM_AXIS_LOG=<absolute existing output/log path>
STANDARD_ODOM_AXIS_LOG=<absolute existing output/log path>
```

The raw odom log must contain:

```text
ODOM_AXIS_TOPIC=/utlidar/robot_odom
ODOM_AXIS_STATUS=PASS
```

The standardized odom log must contain:

```text
ODOM_AXIS_TOPIC=/dddmr_go2/robot_odom_standard
ODOM_AXIS_STATUS=PASS
```

If either run returns insufficient movement, the gate is inconclusive, not
failed.

## Gate 4: Supervised Live Short Goal

The current accepted supervised live navigation wrapper is capped at
`0.30m/s`:

```text
GO2_XT16_LIVE_WRAPPER_DEFAULT_MAX_X=0.30
GO2_XT16_LIVE_PROBE_HARD_CAP_X=0.10
```

Therefore the valid current supervised live acceptance target is a short,
clear-space goal under the `0.30m/s` wrapper cap. The separate Sport live probe
remains capped at `0.10m/s`; it only proves the adapter can publish Move and
StopMove before navigation is enabled.

Read-only readiness:

```bash
cd /home/lin/new2/dddmr_navigation
scripts/check_go2_xt16_sport_live_readiness.sh
```

Required readiness evidence:

```text
GO2_XT16_SPORT_LIVE_READINESS=PASS
SPORT_LIVE_READINESS_LOG=<absolute existing output/log path>
```

`SPORT_LIVE_READINESS_LOG` must contain:

```text
RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS
```

Only after onsite approval and supervision:

```bash
GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2 \
  scripts/run_go2_sport_adapter_supervised_probe.sh --live
```

Then, only with the live probe summary:

```bash
GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
GO2_SPORT_PROBE_SUMMARY=/tmp/go2_sport_adapter_live_<timestamp>_summary.env \
  scripts/run_go2_xt16_navigation_supervised_live.sh --live
```

Pass evidence:

```text
GO2_XT16_SPORT_LIVE_READINESS=PASS
SPORT_LIVE_READINESS_LOG=<absolute existing output/log path>
SPORT_PROBE_RESULT=GO2_SPORT_ADAPTER_live_COMPLETE
SPORT_PROBE_SUMMARY=<absolute existing summary env path>
SPORT_PROBE_HAS_MOVE_1008=true
SPORT_PROBE_HAS_STOPMOVE_1003=true
LIVE_NAV_SHORT_GOAL_STATUS=PASS
LIVE_NAV_SUMMARY=<absolute existing summary env path>
LIVE_NAV_LOG=<existing log path>
LIVE_NAV_MAX_X_MPS=0.30
NO_RESIDUAL_RUNTIME_STATUS=PASS
NO_RESIDUAL_RUNTIME_LOG=<absolute existing output/log path>
```

`SPORT_PROBE_SUMMARY` must contain:

```text
MODE=live
RESULT=GO2_SPORT_ADAPTER_live_COMPLETE
REQUEST_TOPIC=/api/sport/request
REQUEST_ECHO_LOG=<existing echo log path>
```

The referenced request echo log must contain both:

```text
api_id: 1008
api_id: 1003
```

`LIVE_NAV_SUMMARY` must contain:

```text
MODE=live
RESULT=GO2_XT16_MIXED_LIVE_NAV_STOPPED
REQUEST_TOPIC=/api/sport/request
SPORT_MAX_X=<value <= 0.30>
```

`LIVE_NAV_LOG` must be an existing non-empty log containing:

```text
RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING
SUMMARY_LOG=
```

`NO_RESIDUAL_RUNTIME_LOG` must contain:

```text
GO2_XT16_RUNTIME_DOCKER_STATUS=PASS
GO2_XT16_RUNTIME_PROCESS_STATUS=PASS
GO2_XT16_RUNTIME_CLEAN_STATUS=PASS
```

## Completion Rule

Do not mark the axis/tilt report complete until all four remaining gates have
direct evidence in the report:

```text
1. raw and standardized odom axis probe: PASS
2. RViz base_link known front-object check: PASS
3. initial-pose TF tilt recheck: PASS
4. supervised live short goal: PASS under the current approved wrapper cap
```

The evidence file must also link to this report:

```text
REPORT_PATH=/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md
```

Use the evidence verifier before claiming completion:

```bash
cd /home/lin/new2/dddmr_navigation
scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh \
  --evidence docs/go2_xt16_axis_tilt_acceptance_evidence.env
```

For a compact reviewer/CI status summary:

```bash
scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh \
  --evidence docs/go2_xt16_axis_tilt_acceptance_evidence.env \
  --status-report \
  --allow-incomplete
```

Current evidence file:

```text
docs/go2_xt16_axis_tilt_acceptance_evidence.env
```

Completion checker:

```bash
scripts/check_go2_xt16_report_completion.sh
```

The template is:

```text
docs/go2_xt16_axis_tilt_acceptance_evidence.env.example
```

The verifier must end with:

```text
GO2_XT16_ACCEPTANCE_STATUS=PASS
```

If it returns `GO2_XT16_ACCEPTANCE_STATUS=NOT_READY`, the report remains
incomplete even if static checks pass.

Use the report completion checker before marking the report complete:

```bash
scripts/check_go2_xt16_report_completion.sh
```

During handoff, this can be run without failing the shell:

```bash
scripts/check_go2_xt16_report_completion.sh --allow-incomplete
```

To keep the top authority block in the report synchronized with the current
evidence file after evidence updates, run:

```bash
scripts/sync_go2_xt16_axis_tilt_report_status.sh --dry-run
```

If the dry-run reports `GO2_XT16_REPORT_STATUS_SYNC_CHANGED=true`, inspect the
output and rerun without `--dry-run` only after confirming the evidence file is
the intended source of truth. This helper updates only the four top status
lines. It does not replace the four manual/supervised gate artifacts, and the
completion rule remains `GO2_XT16_ACCEPTANCE_STATUS=PASS`.

Before marking the report complete, run the requirement-to-evidence audit:

```bash
scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh
scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh --require-complete
```

The first command prints the current gate matrix. The second command must fail
until all four required gates are backed by current evidence and the acceptance
verifier returns PASS.

## Evidence Update Helper

After a manual or supervised gate has produced logs/screenshots, write only the
new KEY=VALUE lines into a small update file and let the helper update the
current evidence file. The helper rejects unknown keys, does not source the
update file, and runs the evidence verifier before writing.

All screenshot, log, and summary artifact paths must be absolute paths. The
verifier rejects relative paths even if the file exists in the current
directory.

Example dry-run:

```bash
cat >/tmp/go2_xt16_gate_update.env <<'EOF'
RVIZ_BASE_LINK_FRONT_OBJECT_STATUS=PASS
RVIZ_FIXED_FRAME=base_link
RVIZ_FRONT_OBJECT_DIRECTION=+X
SCREENSHOT=/absolute/path/to/gate1_base_link_front_object.png
EOF

scripts/update_go2_xt16_axis_tilt_acceptance_evidence.sh \
  --evidence docs/go2_xt16_axis_tilt_acceptance_evidence.env \
  --update /tmp/go2_xt16_gate_update.env \
  --dry-run
```

If the dry-run passes, rerun without `--dry-run`, then run:

```bash
scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh \
  --evidence docs/go2_xt16_axis_tilt_acceptance_evidence.env
scripts/check_go2_xt16_report_completion.sh
```

## Artifact-To-Update Builder

Prefer generating the update file from real gate artifacts instead of typing all
fields by hand. The builder validates artifact contents and prints only
KEY=VALUE lines.

The builder also rejects relative artifact paths before printing update lines.

Gate 1 example:

```bash
scripts/build_go2_xt16_axis_tilt_gate_update.sh \
  --gate gate1 \
  --screenshot /absolute/path/to/gate1_base_link_front_object.png \
  >/tmp/go2_xt16_gate1_update.env
```

Gate 2 example:

```bash
scripts/build_go2_xt16_axis_tilt_gate_update.sh \
  --gate gate2 \
  --tf-health-log /absolute/path/to/gate2_tf_health.txt \
  >/tmp/go2_xt16_gate2_update.env
```

Gate 3 example:

```bash
scripts/build_go2_xt16_axis_tilt_gate_update.sh \
  --gate gate3 \
  --raw-odom-axis-log /absolute/path/to/gate3_raw_odom_axis.txt \
  --standard-odom-axis-log /absolute/path/to/gate3_standard_odom_axis.txt \
  >/tmp/go2_xt16_gate3_update.env
```

Gate 4 example:

```bash
scripts/build_go2_xt16_axis_tilt_gate_update.sh \
  --gate gate4 \
  --sport-readiness-log /absolute/path/to/gate4_sport_readiness.txt \
  --sport-probe-summary /absolute/path/to/gate4_sport_probe_summary.env \
  --live-nav-summary /absolute/path/to/gate4_live_nav_summary.env \
  --live-nav-log /absolute/path/to/gate4_live_nav.log \
  --runtime-clean-log /absolute/path/to/no_motion_runtime_clean.txt \
  >/tmp/go2_xt16_gate4_update.env
```

Then feed the generated file into the evidence updater with `--dry-run` first.

## Synthetic Pipeline Self-Test

Before a handoff, verify that the local evidence path still works end to end:

```bash
scripts/check_go2_xt16_acceptance_pipeline_selftest.sh
```

This creates only temporary synthetic artifacts and a temporary evidence file.
It proves the builder, updater, and verifier can produce
`GO2_XT16_ACCEPTANCE_STATUS=PASS` when all required evidence is present. It does
not modify `docs/go2_xt16_axis_tilt_acceptance_evidence.env` and does not
replace the real four-gate acceptance evidence.

## Gate Gap Summary

To see exactly which evidence fields and artifacts are still missing from the
current evidence file, run:

```bash
scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh \
  --evidence docs/go2_xt16_axis_tilt_acceptance_evidence.env
```

This is a no-motion checklist helper. It validates absolute artifact paths,
basic file presence, screenshot type, and required PASS markers, but the final
completion gate remains:

```text
GO2_XT16_ACCEPTANCE_STATUS=PASS
```

## No-Motion Status Snapshot

To archive the current state before handoff or before running the next manual
gate, collect a text snapshot:

```bash
scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh \
  --output /home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/status_snapshot_20260704_current.txt
```

The snapshot includes the status-report output, gate gap summary, report
completion audit, and residual runtime clean check. It is a local static
artifact only and does not replace real gate evidence.

## Manual Gate Handoff

To generate a compact operator handoff with the current gaps and the exact
builder/updater commands to use after artifacts are captured, run:

```bash
scripts/build_go2_xt16_axis_tilt_manual_handoff.sh \
  --output /home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/manual_gate_handoff_20260704_current.md
```

The handoff does not authorize motion. It only organizes the four remaining
gate artifacts and the local evidence-update commands.

## Refresh Static Artifacts

Before handoff, refresh all local static artifacts with:

```bash
scripts/refresh_go2_xt16_axis_tilt_static_artifacts.sh
```

This rewrites the runtime-clean log, status snapshot, and manual handoff, then
runs the report completion and manual gate static audits. It does not run any
manual/supervised gate and does not change the final `NOT_READY` state unless
real gate evidence has already been applied.
