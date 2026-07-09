#!/usr/bin/env python3
"""Analyze a Go2 yaw feedback probe SUMMARY_LOG without publishing anything."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze scripts/run_go2_yaw_feedback_probe.sh SUMMARY_LOG output "
            "against the yaw-arc live-gate criteria."
        )
    )
    parser.add_argument("summary_log", type=Path, help="Probe *_summary.env file")
    parser.add_argument(
        "--min-move-yaw",
        type=float,
        default=0.03,
        help="Minimum absolute Move-window yaw delta in rad",
    )
    parser.add_argument(
        "--min-retained-yaw",
        type=float,
        default=0.02,
        help="Minimum absolute yaw retained at first StopMove + retained window",
    )
    parser.add_argument(
        "--retained-window-sec",
        type=float,
        default=0.5,
        help="Seconds after first StopMove used for retained yaw",
    )
    parser.add_argument(
        "--max-move-displacement",
        type=float,
        default=0.05,
        help="Maximum Move-window displacement in meters",
    )
    parser.add_argument(
        "--max-final-yaw-speed",
        type=float,
        default=0.03,
        help="Maximum absolute final Sport yaw_speed after StopMove",
    )
    parser.add_argument(
        "--write-env",
        type=Path,
        help="Optional path for machine-readable analysis env output",
    )
    return parser.parse_args()


def parse_env(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key] = value
    return values


def resolve_artifact(summary_log: Path, raw_path: str, fallback_name: str) -> Path:
    candidates: list[Path] = []
    if raw_path:
        value = Path(raw_path)
        candidates.append(value)
        candidates.append(summary_log.parent / value.name)
        candidates.append(Path.cwd() / value.name)
        candidates.append(Path("/tmp") / value.name)
    candidates.append(summary_log.with_name(fallback_name))
    for candidate in candidates:
        if candidate.exists():
            return candidate
    tried = ", ".join(str(p) for p in candidates)
    raise FileNotFoundError(f"could not resolve artifact for {fallback_name}; tried: {tried}")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def to_float(row: dict[str, str], key: str) -> float:
    return float(row[key])


def norm_delta(start: float, end: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


def sign(value: float) -> int:
    if value > 0.0:
        return 1
    if value < 0.0:
        return -1
    return 0


def sample_at_or_after(rows: list[dict[str, str]], t: float) -> dict[str, str]:
    for row in rows:
        if to_float(row, "t_monotonic") >= t:
            return row
    return rows[-1]


def sample_at_or_before(rows: list[dict[str, str]], t: float) -> dict[str, str]:
    result = rows[0]
    for row in rows:
        if to_float(row, "t_monotonic") > t:
            break
        result = row
    return result


def nearest_sample(rows: list[dict[str, str]], t: float) -> dict[str, str]:
    return min(rows, key=lambda row: abs(to_float(row, "t_monotonic") - t))


def request_times(rows: list[dict[str, str]]) -> tuple[list[float], list[float]]:
    move_times = [
        to_float(row, "t_monotonic")
        for row in rows
        if int(row["api_id"]) == 1008
    ]
    stop_times = [
        to_float(row, "t_monotonic")
        for row in rows
        if int(row["api_id"]) == 1003
    ]
    return move_times, stop_times


def infer_yaw_cmd(summary: dict[str, str], requests: list[dict[str, str]]) -> float:
    if summary.get("YAW_CMD"):
        return float(summary["YAW_CMD"])
    for row in requests:
        if int(row["api_id"]) != 1008 or not row.get("parameter"):
            continue
        return float(json.loads(row["parameter"])["z"])
    raise ValueError("cannot infer YAW_CMD from summary or first Move request")


def displacement(start: dict[str, str], end: dict[str, str], x_key: str, y_key: str) -> float:
    return math.hypot(to_float(end, x_key) - to_float(start, x_key), to_float(end, y_key) - to_float(start, y_key))


def motion_metrics(
    rows: list[dict[str, str]],
    *,
    yaw_key: str,
    x_key: str,
    y_key: str,
    first_move: float,
    first_stop: float,
    retained_t: float,
) -> dict[str, float]:
    move_start = sample_at_or_after(rows, first_move)
    move_end = sample_at_or_before(rows, first_stop)
    retained = nearest_sample(rows, retained_t)
    return {
        "move_start_t": to_float(move_start, "t_monotonic"),
        "move_end_t": to_float(move_end, "t_monotonic"),
        "retained_t": to_float(retained, "t_monotonic"),
        "move_yaw_delta": norm_delta(to_float(move_start, yaw_key), to_float(move_end, yaw_key)),
        "retained_yaw_delta": norm_delta(to_float(move_start, yaw_key), to_float(retained, yaw_key)),
        "move_displacement": displacement(move_start, move_end, x_key, y_key),
        "retained_displacement": displacement(move_start, retained, x_key, y_key),
    }


def pass_fail(condition: bool) -> str:
    return "PASS" if condition else "FAIL"


def emit_env(path: Path, values: dict[str, object]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for key in sorted(values):
            handle.write(f"{key}={values[key]}\n")


def main() -> int:
    args = parse_args()
    summary_log = args.summary_log.expanduser().resolve()
    summary = parse_env(summary_log)

    request_csv = resolve_artifact(
        summary_log, summary.get("REQUEST_CSV", ""), summary_log.name.replace("_summary.env", "_requests.csv")
    )
    odom_csv = resolve_artifact(
        summary_log, summary.get("ODOM_CSV", ""), summary_log.name.replace("_summary.env", "_odom.csv")
    )
    sport_csv = resolve_artifact(
        summary_log, summary.get("SPORT_CSV", ""), summary_log.name.replace("_summary.env", "_sport.csv")
    )

    requests = read_csv(request_csv)
    odom = read_csv(odom_csv)
    sport = read_csv(sport_csv)
    if not requests:
        raise SystemExit("request CSV has no rows")
    if not odom:
        raise SystemExit("odom CSV has no rows")
    if not sport:
        raise SystemExit("sport CSV has no rows")

    move_times, stop_times = request_times(requests)
    if not move_times:
        raise SystemExit("request CSV has no Move(api_id=1008)")
    later_stops = [t for t in stop_times if t >= move_times[0]]
    if not later_stops:
        raise SystemExit("request CSV has no StopMove(api_id=1003) after first Move")

    yaw_cmd = infer_yaw_cmd(summary, requests)
    first_move = move_times[0]
    first_stop = later_stops[0]
    retained_t = first_stop + args.retained_window_sec

    odom_metrics = motion_metrics(
        odom,
        yaw_key="yaw_rad",
        x_key="x",
        y_key="y",
        first_move=first_move,
        first_stop=first_stop,
        retained_t=retained_t,
    )
    sport_metrics = motion_metrics(
        sport,
        yaw_key="imu_rpy_yaw_rad",
        x_key="x",
        y_key="y",
        first_move=first_move,
        first_stop=first_stop,
        retained_t=retained_t,
    )
    final_yaw_speed = to_float(sport[-1], "yaw_speed")

    checks: dict[str, bool] = {
        "move_count_positive": len(move_times) > 0,
        "stop_count_positive": len(later_stops) > 0,
        "odom_yaw_sign_correct": sign(odom_metrics["move_yaw_delta"]) == sign(yaw_cmd),
        "sport_yaw_sign_correct": sign(sport_metrics["move_yaw_delta"]) == sign(yaw_cmd),
        "odom_move_yaw_large_enough": abs(odom_metrics["move_yaw_delta"]) >= args.min_move_yaw,
        "sport_move_yaw_large_enough": abs(sport_metrics["move_yaw_delta"]) >= args.min_move_yaw,
        "odom_retained_yaw_large_enough": abs(odom_metrics["retained_yaw_delta"]) >= args.min_retained_yaw,
        "sport_retained_yaw_large_enough": abs(sport_metrics["retained_yaw_delta"]) >= args.min_retained_yaw,
        "odom_move_displacement_bounded": odom_metrics["move_displacement"] <= args.max_move_displacement,
        "sport_move_displacement_bounded": sport_metrics["move_displacement"] <= args.max_move_displacement,
        "final_yaw_speed_near_zero": abs(final_yaw_speed) <= args.max_final_yaw_speed,
    }
    overall_pass = all(checks.values())

    env_values: dict[str, object] = {
        "RESULT": "GO2_YAW_PROBE_ANALYSIS_PASS" if overall_pass else "GO2_YAW_PROBE_ANALYSIS_FAIL",
        "SUMMARY_LOG": summary_log,
        "REQUEST_CSV": request_csv,
        "ODOM_CSV": odom_csv,
        "SPORT_CSV": sport_csv,
        "YAW_CMD": f"{yaw_cmd:.9f}",
        "MOVE_COUNT": len(move_times),
        "STOP_COUNT": len(later_stops),
        "FIRST_MOVE_T": f"{first_move:.9f}",
        "FIRST_STOP_T": f"{first_stop:.9f}",
        "RETAINED_TARGET_T": f"{retained_t:.9f}",
        "ODOM_MOVE_YAW_DELTA": f"{odom_metrics['move_yaw_delta']:.9f}",
        "SPORT_MOVE_YAW_DELTA": f"{sport_metrics['move_yaw_delta']:.9f}",
        "ODOM_RETAINED_YAW_DELTA": f"{odom_metrics['retained_yaw_delta']:.9f}",
        "SPORT_RETAINED_YAW_DELTA": f"{sport_metrics['retained_yaw_delta']:.9f}",
        "ODOM_MOVE_DISPLACEMENT": f"{odom_metrics['move_displacement']:.9f}",
        "SPORT_MOVE_DISPLACEMENT": f"{sport_metrics['move_displacement']:.9f}",
        "ODOM_RETAINED_DISPLACEMENT": f"{odom_metrics['retained_displacement']:.9f}",
        "SPORT_RETAINED_DISPLACEMENT": f"{sport_metrics['retained_displacement']:.9f}",
        "SPORT_FINAL_YAW_SPEED": f"{final_yaw_speed:.9f}",
    }
    for name, ok in checks.items():
        env_values[f"CHECK_{name.upper()}"] = pass_fail(ok)

    print(env_values["RESULT"])
    print(f"SUMMARY_LOG={summary_log}")
    print(f"YAW_CMD={yaw_cmd:.6f}")
    print(
        "ODOM move_yaw={:.6f} retained_yaw={:.6f} move_disp={:.6f}".format(
            odom_metrics["move_yaw_delta"],
            odom_metrics["retained_yaw_delta"],
            odom_metrics["move_displacement"],
        )
    )
    print(
        "SPORT move_yaw={:.6f} retained_yaw={:.6f} move_disp={:.6f} final_yaw_speed={:.6f}".format(
            sport_metrics["move_yaw_delta"],
            sport_metrics["retained_yaw_delta"],
            sport_metrics["move_displacement"],
            final_yaw_speed,
        )
    )
    for name, ok in checks.items():
        print(f"{pass_fail(ok)} {name}")

    if args.write_env:
        emit_env(args.write_env, env_values)
        print(f"ANALYSIS_ENV={args.write_env}")

    return 0 if overall_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
