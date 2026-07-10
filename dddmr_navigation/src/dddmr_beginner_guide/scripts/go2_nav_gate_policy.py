#!/usr/bin/env python3

import math
from typing import Optional


TRACKING_STATE = "TRACKING"


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
