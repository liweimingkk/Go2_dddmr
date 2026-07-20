#!/usr/bin/env python3

import math
from typing import NamedTuple, Optional


TRACKING_STATE = "TRACKING"
HEALTHY_STATE = "HEALTHY"
RECOVERY_ROTATION_DECISION = "d_recovery_waitdone"


class RecoveryRotationGateResult(NamedTuple):
    allowed: bool
    reason: str
    yaw: float


def recovery_rotation_gate(
    decision: str,
    decision_age_sec: Optional[float],
    decision_timeout_sec: float,
    command_time_ns: Optional[int],
    decision_transition_time_ns: Optional[int],
    x: float,
    y: float,
    yaw: float,
    zero_epsilon: float,
    max_yaw: float,
) -> RecoveryRotationGateResult:
    """Admit only a fresh, newly received, bounded pure-yaw recovery command."""
    blocked = RecoveryRotationGateResult(False, "wrong_state", 0.0)
    if decision != RECOVERY_ROTATION_DECISION:
        return blocked
    if (
        decision_age_sec is None
        or not math.isfinite(decision_age_sec)
        or decision_age_sec < 0.0
        or not math.isfinite(decision_timeout_sec)
        or decision_timeout_sec <= 0.0
        or decision_age_sec > decision_timeout_sec
    ):
        return RecoveryRotationGateResult(False, "stale_decision", 0.0)
    if (
        command_time_ns is None
        or decision_transition_time_ns is None
        or not math.isfinite(command_time_ns)
        or not math.isfinite(decision_transition_time_ns)
        or command_time_ns <= decision_transition_time_ns
    ):
        return RecoveryRotationGateResult(False, "old_command", 0.0)
    if (
        not math.isfinite(zero_epsilon)
        or zero_epsilon < 0.0
        or not math.isfinite(max_yaw)
        or max_yaw <= zero_epsilon
    ):
        return RecoveryRotationGateResult(False, "invalid_limits", 0.0)
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        return RecoveryRotationGateResult(False, "nonfinite_command", 0.0)
    if abs(x) > zero_epsilon or abs(y) > zero_epsilon:
        return RecoveryRotationGateResult(False, "translation_not_allowed", 0.0)
    if abs(yaw) <= zero_epsilon:
        return RecoveryRotationGateResult(False, "zero_yaw", 0.0)

    bounded_yaw = max(-max_yaw, min(max_yaw, yaw))
    if abs(bounded_yaw) <= zero_epsilon:
        return RecoveryRotationGateResult(False, "zero_yaw", 0.0)
    return RecoveryRotationGateResult(True, "pure_yaw", bounded_yaw)


def localization_block_reason(
    required: bool,
    status: Optional[str],
    status_age_sec: Optional[float],
    timeout_sec: float,
) -> Optional[str]:
    """Return a stable machine-readable reason when localization blocks motion."""
    if not required:
        return None
    if status is None or status_age_sec is None:
        return "localization_no_status"
    if (
        not math.isfinite(status_age_sec)
        or not math.isfinite(timeout_sec)
        or timeout_sec <= 0.0
        or status_age_sec > timeout_sec
    ):
        return "localization_status_stale"
    normalized = status.strip().upper()
    if normalized != TRACKING_STATE:
        return "localization_%s" % (normalized.lower() or "unknown")
    return None


def localization_health_block_reason(
    required: bool,
    health: Optional[str],
    health_age_sec: Optional[float],
    timeout_sec: float,
) -> Optional[str]:
    """Block motion unless the full localization geometry health is fresh."""
    if not required:
        return None
    if health is None or health_age_sec is None:
        return "localization_no_health"
    if (
        not math.isfinite(health_age_sec)
        or not math.isfinite(timeout_sec)
        or timeout_sec <= 0.0
        or health_age_sec > timeout_sec
    ):
        return "localization_health_stale"
    normalized = health.strip().upper()
    if normalized != HEALTHY_STATE:
        return "localization_health_%s" % (normalized.lower() or "unknown")
    return None
