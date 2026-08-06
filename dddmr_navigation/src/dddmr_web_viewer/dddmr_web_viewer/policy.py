"""Pure validation and preview-authorization policy for the web bridge."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass
class PreviewAuthorization:
    """Track whether a successfully previewed goal may be executed once."""

    max_age_sec: float
    _recorded_at_ns: Optional[int] = None
    _used: bool = False

    def record(self, now_ns: int) -> None:
        """Record a newly completed path preview."""
        self._recorded_at_ns = now_ns
        self._used = False

    def clear(self) -> None:
        """Invalidate the current preview."""
        self._recorded_at_ns = None
        self._used = False

    def authorize(self, now_ns: int) -> Tuple[bool, str]:
        """Consume one fresh preview authorization."""
        if self._recorded_at_ns is None:
            return False, "no completed preview is available"
        if self._used:
            return False, "the preview was already executed"
        age_sec = max(0.0, (now_ns - self._recorded_at_ns) / 1_000_000_000.0)
        if age_sec > self.max_age_sec:
            return False, "the preview expired; plan the goal again"
        self._used = True
        return True, "preview authorized"


def quaternion_is_valid(x: float, y: float, z: float, w: float) -> bool:
    """Return whether the values form a finite, approximately unit quaternion."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        return False
    norm_squared = sum(value * value for value in values)
    return abs(norm_squared - 1.0) <= 0.1


def position_is_finite(x: float, y: float, z: float) -> bool:
    """Return whether a Cartesian position contains only finite values."""
    return all(math.isfinite(value) for value in (x, y, z))
