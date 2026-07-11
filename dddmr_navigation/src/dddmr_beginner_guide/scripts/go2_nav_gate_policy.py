#!/usr/bin/env python3

import math
from typing import NamedTuple, Optional


TRACKING_STATE = "TRACKING"
RECOVERY_ROTATION_DECISION = "d_recovery_waitdone"

TERRAIN_UNKNOWN = 0
TERRAIN_FLAT = 1
TERRAIN_RAMP = 2
TERRAIN_STAIR_TREAD = 3
TERRAIN_STAIR_RISER = 4
TERRAIN_EDGE = 5
TERRAIN_DROP = 6

STAIR_STATE_NORMAL = 0
STAIR_STATE_PRECHECK = 1
STAIR_STATE_APPROACH = 2
STAIR_STATE_ALIGN = 3
STAIR_STATE_COMMITTED = 4
STAIR_STATE_LANDING_VERIFY = 5
STAIR_STATE_FAULT_LATCH = 6


class RecoveryRotationGateResult(NamedTuple):
    allowed: bool
    reason: str
    yaw: float


class TerrainGateStatus(NamedTuple):
    terrain_class: int
    traversal_state: int
    allow_forward: bool
    allow_reverse: bool
    drop_detected: bool
    dynamic_obstacle: bool
    gait_unchanged: bool
    confidence: float
    support_ratio: float
    heading_error_rad: float
    lateral_error_m: float
    longitudinal_slope_rad: float = 0.0
    cross_slope_rad: float = 0.0
    body_roll_rad: float = 0.0
    body_pitch_rad: float = 0.0


class TerrainGateLimits(NamedTuple):
    timeout_sec: float
    min_confidence: float
    min_support_ratio: float
    max_heading_error_rad: float
    max_lateral_error_m: float
    align_max_forward_mps: float
    align_max_yaw_rps: float
    committed_max_yaw_rps: float
    zero_epsilon: float
    committed_max_forward_mps: float = 0.0
    ramp_max_up_slope_rad: float = 0.0
    ramp_max_down_slope_rad: float = 0.0
    ramp_max_cross_slope_rad: float = 0.0
    ramp_up_max_x_mps: float = 0.0
    ramp_down_max_x_mps: float = 0.0
    ramp_max_yaw_rps: float = 0.0
    max_body_roll_rad: float = 0.0
    max_body_pitch_rad: float = 0.0
    output_max_x_mps: float = 0.0
    output_max_y_mps: float = 0.0
    output_max_yaw_rps: float = 0.0


class TerrainGateResult(NamedTuple):
    allowed: bool
    reason: str
    x: float
    y: float
    yaw: float


class MotionProgressMonitor:
    """Pure monotonic-time command/odometry mismatch monitor."""

    def __init__(
        self,
        minimum_command_mps: float,
        minimum_measured_mps: float,
        minimum_speed_ratio: float,
        mismatch_timeout_sec: float,
    ) -> None:
        values = (
            minimum_command_mps,
            minimum_measured_mps,
            minimum_speed_ratio,
            mismatch_timeout_sec,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("motion progress limits must be finite")
        if (
            minimum_command_mps < 0.0
            or minimum_measured_mps < 0.0
            or minimum_speed_ratio < 0.0
            or minimum_speed_ratio > 1.0
            or mismatch_timeout_sec <= 0.0
        ):
            raise ValueError("motion progress limits are outside their safe range")
        self.minimum_command_mps = minimum_command_mps
        self.minimum_measured_mps = minimum_measured_mps
        self.minimum_speed_ratio = minimum_speed_ratio
        self.mismatch_timeout_sec = mismatch_timeout_sec
        self._mismatch_started_sec: Optional[float] = None

    def reset(self) -> None:
        self._mismatch_started_sec = None

    def update(
        self,
        now_sec: float,
        commanded_x_mps: float,
        measured_x_mps: float,
    ) -> Optional[str]:
        if not all(
            math.isfinite(value)
            for value in (now_sec, commanded_x_mps, measured_x_mps)
        ):
            self.reset()
            return "odom_nonfinite"
        if abs(commanded_x_mps) < self.minimum_command_mps:
            self.reset()
            return None
        required_speed = max(
            self.minimum_measured_mps,
            abs(commanded_x_mps) * self.minimum_speed_ratio,
        )
        same_direction = commanded_x_mps * measured_x_mps > 0.0
        if same_direction and abs(measured_x_mps) >= required_speed:
            self.reset()
            return None
        if self._mismatch_started_sec is None or now_sec < self._mismatch_started_sec:
            self._mismatch_started_sec = now_sec
            return None
        if now_sec - self._mismatch_started_sec >= self.mismatch_timeout_sec:
            return "motion_progress_mismatch"
        return None


class TerrainFaultLatch:
    """Latch a terrain fault only after the gate has observed a healthy state."""

    ALWAYS_LATCH_REASONS = frozenset(
        {
            "terrain_drop",
            "terrain_dynamic_obstacle",
            "terrain_gait_changed",
            "stair_fault_latched",
            "terrain_body_roll",
            "terrain_body_pitch",
            "motion_progress_mismatch",
            "odom_nonfinite",
            "stair_approach_unscored_command",
            "stair_align_unscored_command",
            "stair_committed_unscored_command",
        }
    )

    def __init__(self) -> None:
        self.armed = False
        self.reason: Optional[str] = None

    @property
    def latched(self) -> bool:
        return self.reason is not None

    def observe(self, allowed: bool, reason: str, motion_requested: bool) -> None:
        if self.latched:
            return
        if allowed:
            self.armed = True
            return
        if reason in self.ALWAYS_LATCH_REASONS or (self.armed and motion_requested):
            self.reason = reason

    def reset(self, stopped: bool, inputs_healthy: bool) -> bool:
        if not self.latched or not stopped or not inputs_healthy:
            return False
        self.reason = None
        self.armed = False
        return True


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


def terrain_motion_gate(
    required: bool,
    status: Optional[TerrainGateStatus],
    status_age_sec: Optional[float],
    limits: TerrainGateLimits,
    x: float,
    y: float,
    yaw: float,
) -> TerrainGateResult:
    """Fail-closed terrain and stair command policy.

    This changes navigation command shape only. It never selects or changes a
    Unitree gait.
    """
    if not required:
        return TerrainGateResult(True, "terrain_optional", x, y, yaw)

    blocked = TerrainGateResult(False, "terrain_no_status", 0.0, 0.0, 0.0)
    if status is None or status_age_sec is None:
        return blocked
    numeric_limits = tuple(limits)
    if not all(math.isfinite(value) for value in numeric_limits):
        return TerrainGateResult(False, "terrain_invalid_limits", 0.0, 0.0, 0.0)
    if (
        limits.timeout_sec <= 0.0
        or limits.min_confidence < 0.0
        or limits.min_confidence > 1.0
        or limits.min_support_ratio < 0.0
        or limits.min_support_ratio > 1.0
        or limits.max_heading_error_rad < 0.0
        or limits.max_lateral_error_m < 0.0
        or limits.align_max_forward_mps < 0.0
        or limits.align_max_yaw_rps < 0.0
        or limits.committed_max_yaw_rps < 0.0
        or limits.committed_max_forward_mps < 0.0
        or limits.zero_epsilon < 0.0
        or limits.ramp_max_up_slope_rad < 0.0
        or limits.ramp_max_up_slope_rad > math.pi / 2.0
        or limits.ramp_max_down_slope_rad < 0.0
        or limits.ramp_max_down_slope_rad > math.pi / 2.0
        or limits.ramp_max_cross_slope_rad < 0.0
        or limits.ramp_max_cross_slope_rad > math.pi / 2.0
        or limits.ramp_up_max_x_mps < 0.0
        or limits.ramp_down_max_x_mps < 0.0
        or limits.ramp_max_yaw_rps < 0.0
        or limits.max_body_roll_rad < 0.0
        or limits.max_body_roll_rad > math.pi
        or limits.max_body_pitch_rad < 0.0
        or limits.max_body_pitch_rad > math.pi / 2.0
        or limits.output_max_x_mps < 0.0
        or limits.output_max_y_mps < 0.0
        or limits.output_max_yaw_rps < 0.0
    ):
        return TerrainGateResult(False, "terrain_invalid_limits", 0.0, 0.0, 0.0)
    if (
        not math.isfinite(status_age_sec)
        or status_age_sec < 0.0
        or status_age_sec > limits.timeout_sec
    ):
        return TerrainGateResult(False, "terrain_status_stale", 0.0, 0.0, 0.0)
    if not all(
        math.isfinite(value)
        for value in (
            status.confidence,
            status.support_ratio,
            status.heading_error_rad,
            status.lateral_error_m,
            status.longitudinal_slope_rad,
            status.cross_slope_rad,
            status.body_roll_rad,
            status.body_pitch_rad,
            x,
            y,
            yaw,
        )
    ):
        return TerrainGateResult(False, "terrain_nonfinite", 0.0, 0.0, 0.0)
    if status.drop_detected or status.terrain_class == TERRAIN_DROP:
        return TerrainGateResult(False, "terrain_drop", 0.0, 0.0, 0.0)
    if status.dynamic_obstacle:
        return TerrainGateResult(False, "terrain_dynamic_obstacle", 0.0, 0.0, 0.0)
    if not status.gait_unchanged:
        return TerrainGateResult(False, "terrain_gait_changed", 0.0, 0.0, 0.0)
    if status.terrain_class in (TERRAIN_UNKNOWN, TERRAIN_EDGE):
        return TerrainGateResult(False, "terrain_not_traversable", 0.0, 0.0, 0.0)
    if status.confidence < limits.min_confidence:
        return TerrainGateResult(False, "terrain_low_confidence", 0.0, 0.0, 0.0)
    if status.support_ratio < limits.min_support_ratio:
        return TerrainGateResult(False, "terrain_low_support", 0.0, 0.0, 0.0)
    if abs(status.body_roll_rad) > limits.max_body_roll_rad:
        return TerrainGateResult(False, "terrain_body_roll", 0.0, 0.0, 0.0)
    if abs(status.body_pitch_rad) > limits.max_body_pitch_rad:
        return TerrainGateResult(False, "terrain_body_pitch", 0.0, 0.0, 0.0)

    stair_state = status.traversal_state
    if stair_state == STAIR_STATE_FAULT_LATCH:
        return TerrainGateResult(False, "stair_fault_latched", 0.0, 0.0, 0.0)
    if stair_state in (STAIR_STATE_PRECHECK, STAIR_STATE_LANDING_VERIFY):
        return TerrainGateResult(False, "stair_hold", 0.0, 0.0, 0.0)

    if x > limits.zero_epsilon and not status.allow_forward:
        return TerrainGateResult(
            False, "terrain_forward_not_allowed", 0.0, 0.0, 0.0
        )
    if x < -limits.zero_epsilon and not status.allow_reverse:
        return TerrainGateResult(
            False, "terrain_reverse_not_allowed", 0.0, 0.0, 0.0
        )

    if status.terrain_class == TERRAIN_RAMP:
        effective_slope = status.longitudinal_slope_rad
        if x < -limits.zero_epsilon:
            effective_slope = -effective_slope
        ascending = effective_slope > limits.zero_epsilon
        descending = effective_slope < -limits.zero_epsilon
        if ascending and effective_slope > limits.ramp_max_up_slope_rad:
            return TerrainGateResult(False, "ramp_up_slope", 0.0, 0.0, 0.0)
        if descending and -effective_slope > limits.ramp_max_down_slope_rad:
            return TerrainGateResult(False, "ramp_down_slope", 0.0, 0.0, 0.0)
        if abs(status.cross_slope_rad) > limits.ramp_max_cross_slope_rad:
            return TerrainGateResult(False, "ramp_cross_slope", 0.0, 0.0, 0.0)
        speed_limit = (
            limits.ramp_up_max_x_mps
            if ascending
            else limits.ramp_down_max_x_mps
            if descending
            else min(limits.ramp_up_max_x_mps, limits.ramp_down_max_x_mps)
        )
        if abs(x) > limits.zero_epsilon and speed_limit <= limits.zero_epsilon:
            return TerrainGateResult(
                False, "ramp_speed_not_authorized", 0.0, 0.0, 0.0
            )
        bounded_x = max(-speed_limit, min(speed_limit, x))
        bounded_yaw = max(
            -limits.ramp_max_yaw_rps,
            min(limits.ramp_max_yaw_rps, yaw),
        )
        direction = "up" if ascending else "down" if descending else "level"
        return TerrainGateResult(True, "ramp_%s" % direction, bounded_x, y, bounded_yaw)

    if stair_state == STAIR_STATE_APPROACH:
        if (
            abs(x) > limits.output_max_x_mps
            or abs(y) > limits.output_max_y_mps
            or abs(yaw) > limits.output_max_yaw_rps
        ):
            return TerrainGateResult(
                False, "stair_approach_unscored_command", 0.0, 0.0, 0.0
            )
        return TerrainGateResult(True, "stair_approach", x, y, yaw)
    if stair_state in (STAIR_STATE_ALIGN, STAIR_STATE_COMMITTED):
        if abs(status.heading_error_rad) > limits.max_heading_error_rad:
            return TerrainGateResult(False, "stair_heading_error", 0.0, 0.0, 0.0)
        if abs(status.lateral_error_m) > limits.max_lateral_error_m:
            return TerrainGateResult(False, "stair_lateral_error", 0.0, 0.0, 0.0)

    if stair_state == STAIR_STATE_ALIGN:
        max_x = min(limits.align_max_forward_mps, limits.output_max_x_mps)
        max_yaw = min(limits.align_max_yaw_rps, limits.output_max_yaw_rps)
        if abs(x) > max_x or y != 0.0 or abs(yaw) > max_yaw:
            return TerrainGateResult(
                False, "stair_align_unscored_command", 0.0, 0.0, 0.0
            )
        return TerrainGateResult(True, "stair_align", x, y, yaw)

    if stair_state == STAIR_STATE_COMMITTED:
        if not status.allow_forward or x <= limits.zero_epsilon:
            return TerrainGateResult(False, "stair_forward_only", 0.0, 0.0, 0.0)
        if limits.committed_max_forward_mps <= limits.zero_epsilon:
            return TerrainGateResult(
                False, "stair_speed_not_authorized", 0.0, 0.0, 0.0
            )
        max_x = min(limits.committed_max_forward_mps, limits.output_max_x_mps)
        max_yaw = min(limits.committed_max_yaw_rps, limits.output_max_yaw_rps)
        if x > max_x or y != 0.0 or abs(yaw) > max_yaw:
            return TerrainGateResult(
                False, "stair_committed_unscored_command", 0.0, 0.0, 0.0
            )
        return TerrainGateResult(True, "stair_committed", x, y, yaw)

    return TerrainGateResult(True, "terrain_pass", x, y, yaw)
