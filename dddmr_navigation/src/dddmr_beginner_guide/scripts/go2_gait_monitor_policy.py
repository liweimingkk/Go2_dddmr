#!/usr/bin/env python3

"""Pure state policy for proving Go2 mode/gait remain unchanged."""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class GaitContractResult:
    unchanged: bool
    reason: str
    baseline_mode: Optional[int]
    baseline_gait_type: Optional[int]
    current_mode: Optional[int]
    current_gait_type: Optional[int]


class GaitStateContract:
    """Latch a baseline and permanently fault on a mode or gait transition."""

    def __init__(
        self,
        *,
        expected_mode: int = -1,
        expected_gait_type: int = -1,
        timeout_sec: float = 0.30,
    ) -> None:
        if isinstance(expected_mode, bool) or not isinstance(expected_mode, int):
            raise ValueError("expected_mode must be an integer")
        if isinstance(expected_gait_type, bool) or not isinstance(expected_gait_type, int):
            raise ValueError("expected_gait_type must be an integer")
        if expected_mode < -1 or expected_gait_type < -1:
            raise ValueError("expected mode/gait must be -1 or nonnegative")
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be finite and positive")

        self.expected_mode = expected_mode
        self.expected_gait_type = expected_gait_type
        self.timeout_sec = timeout_sec
        self.baseline_mode: Optional[int] = None
        self.baseline_gait_type: Optional[int] = None
        self.current_mode: Optional[int] = None
        self.current_gait_type: Optional[int] = None
        self.last_status_sec: Optional[float] = None
        self.latched_fault: Optional[str] = None

    def observe(self, mode: int, gait_type: int, now_sec: float) -> GaitContractResult:
        if (
            isinstance(mode, bool)
            or not isinstance(mode, int)
            or isinstance(gait_type, bool)
            or not isinstance(gait_type, int)
            or mode < 0
            or gait_type < 0
            or not math.isfinite(now_sec)
        ):
            self._latch_fault("invalid_status")
            return self.evaluate(now_sec)

        if self.last_status_sec is not None and now_sec < self.last_status_sec:
            self._latch_fault("time_regression")

        self.current_mode = mode
        self.current_gait_type = gait_type
        self.last_status_sec = now_sec
        if self.baseline_mode is None:
            self.baseline_mode = self.expected_mode if self.expected_mode >= 0 else mode
        if self.baseline_gait_type is None:
            self.baseline_gait_type = (
                self.expected_gait_type
                if self.expected_gait_type >= 0
                else gait_type
            )

        if mode != self.baseline_mode:
            self._latch_fault("mode_changed")
        elif gait_type != self.baseline_gait_type:
            self._latch_fault("gait_changed")
        return self.evaluate(now_sec)

    def evaluate(self, now_sec: float) -> GaitContractResult:
        reason = self.latched_fault
        if reason is None:
            if self.last_status_sec is None:
                reason = "no_status"
            elif not math.isfinite(now_sec) or now_sec < self.last_status_sec:
                reason = "invalid_time"
            elif now_sec - self.last_status_sec > self.timeout_sec:
                reason = "stale_status"
            else:
                reason = "stable"
        return GaitContractResult(
            unchanged=reason == "stable",
            reason=reason,
            baseline_mode=self.baseline_mode,
            baseline_gait_type=self.baseline_gait_type,
            current_mode=self.current_mode,
            current_gait_type=self.current_gait_type,
        )

    def _latch_fault(self, reason: str) -> None:
        if self.latched_fault is None:
            self.latched_fault = reason
