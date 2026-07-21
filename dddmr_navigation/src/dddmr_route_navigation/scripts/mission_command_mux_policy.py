#!/usr/bin/env python3
"""Pure fail-closed selection policy for outdoor/indoor velocity sources."""

from __future__ import annotations

import math
from typing import Iterable, Sequence, Set, Tuple


Command = Tuple[float, float, float]


def _finite(values: Iterable[object]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def select_command(
    mode: str,
    mode_age_sec: float,
    command: Sequence[float],
    command_age_sec: float,
    decision: str,
    decision_age_sec: float,
    mode_timeout_sec: float,
    command_timeout_sec: float,
    decision_timeout_sec: float,
    allowed_motion_decisions: Set[str],
    max_x: float,
    max_y: float,
    max_yaw: float,
    zero_epsilon: float,
) -> Tuple[Command, str, str]:
    zero: Command = (0.0, 0.0, 0.0)
    normalized_mode = mode.strip().lower()
    if normalized_mode not in {"outdoor", "indoor"}:
        return zero, "d_mission_stopped", "mode_none"
    if not _finite((mode_age_sec, mode_timeout_sec)) or not 0.0 <= mode_age_sec <= mode_timeout_sec:
        return zero, "d_mission_stopped", "mode_stale"
    if len(command) != 3 or not _finite(command):
        return zero, "d_mission_stopped", "command_invalid"
    if not _finite((command_age_sec, command_timeout_sec)) or not 0.0 <= command_age_sec <= command_timeout_sec:
        return zero, "d_mission_stopped", "command_stale"
    if not _finite((decision_age_sec, decision_timeout_sec)) or not 0.0 <= decision_age_sec <= decision_timeout_sec:
        return zero, "d_mission_stopped", "decision_stale"

    x, y, yaw = (float(value) for value in command)
    motion_requested = any(abs(value) > zero_epsilon for value in (x, y, yaw))
    if motion_requested and decision not in allowed_motion_decisions:
        return zero, "d_mission_stopped", "decision_disallowed"

    output = (
        max(-max_x, min(max_x, x)),
        max(-max_y, min(max_y, y)),
        max(-max_yaw, min(max_yaw, yaw)),
    )
    output = tuple(0.0 if abs(value) <= zero_epsilon else value for value in output)
    return output, decision, "pass"
