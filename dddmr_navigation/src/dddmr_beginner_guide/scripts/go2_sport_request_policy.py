#!/usr/bin/env python3

"""Pure normal-gait contract for Go2 navigation Sport requests.

The navigation adapter is deliberately limited to Move and StopMove.  Gait,
posture, recovery-stand, and mode-changing Sport APIs are outside this
contract.  Keeping this module free of ROS imports makes the contract usable
from unit tests and evidence-audit tools.
"""

from dataclasses import dataclass
import json
import math
import re
from typing import Iterable, Optional, Tuple


MOVE_API_ID = 1008
STOP_MOVE_API_ID = 1003
NORMAL_GAIT_NAVIGATION_API_IDS = frozenset((MOVE_API_ID, STOP_MOVE_API_ID))


@dataclass(frozen=True)
class SportRequestRecord:
    request_id: int
    api_id: int


@dataclass(frozen=True)
class SportRequestAudit:
    allowed: bool
    reason: str
    request_count: int
    api_ids: Tuple[int, ...]


def validate_navigation_sport_request(api_id: int, parameter: str) -> None:
    """Raise ``ValueError`` unless a request obeys the normal-gait contract."""

    if isinstance(api_id, bool) or not isinstance(api_id, int):
        raise ValueError("Sport api_id must be an integer")
    if api_id not in NORMAL_GAIT_NAVIGATION_API_IDS:
        raise ValueError(
            "Sport api_id=%d is outside the normal-gait navigation contract" % api_id
        )
    if not isinstance(parameter, str):
        raise ValueError("Sport request parameter must be a string")

    if api_id == STOP_MOVE_API_ID:
        if parameter:
            raise ValueError("StopMove parameter must be empty")
        return

    try:
        payload = json.loads(parameter)
    except (json.JSONDecodeError, TypeError) as exc:
        raise ValueError("Move parameter must be valid JSON") from exc
    if not isinstance(payload, dict) or set(payload) != {"x", "y", "z"}:
        raise ValueError("Move parameter must contain exactly x, y, and z")
    for axis in ("x", "y", "z"):
        value = payload[axis]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("Move %s must be numeric" % axis)
        if not math.isfinite(float(value)):
            raise ValueError("Move %s must be finite" % axis)


def parse_ros2_request_echo(text: str) -> Tuple[SportRequestRecord, ...]:
    """Extract Unitree request identities from ``ros2 topic echo`` output."""

    if not isinstance(text, str):
        raise ValueError("request echo must be text")

    records = []
    for block in re.split(r"\n---\s*(?:\n|$)", text):
        request_match = re.search(
            r"^\s*id:\s*(-?\d+)\s*$", block, flags=re.MULTILINE
        )
        api_match = re.search(
            r"^\s*api_id:\s*(-?\d+)\s*$", block, flags=re.MULTILINE
        )
        if request_match is None and api_match is None:
            continue
        if request_match is None or api_match is None:
            raise ValueError("request echo contains an incomplete identity block")
        records.append(
            SportRequestRecord(
                request_id=int(request_match.group(1)),
                api_id=int(api_match.group(1)),
            )
        )
    return tuple(records)


def audit_navigation_request_records(
    records: Iterable[SportRequestRecord],
    *,
    minimum_request_id: Optional[int] = None,
    require_move: bool = False,
    require_stop: bool = False,
) -> SportRequestAudit:
    """Check that observed requests cannot change posture, mode, or gait."""

    selected = tuple(
        record
        for record in records
        if minimum_request_id is None or record.request_id > minimum_request_id
    )
    api_ids = tuple(record.api_id for record in selected)
    unexpected = sorted(set(api_ids) - NORMAL_GAIT_NAVIGATION_API_IDS)
    if unexpected:
        return SportRequestAudit(
            False,
            "unexpected_api_ids=" + ",".join(str(value) for value in unexpected),
            len(selected),
            api_ids,
        )
    if require_move and MOVE_API_ID not in api_ids:
        return SportRequestAudit(False, "missing_move", len(selected), api_ids)
    if require_stop and STOP_MOVE_API_ID not in api_ids:
        return SportRequestAudit(False, "missing_stopmove", len(selected), api_ids)
    if not selected:
        return SportRequestAudit(False, "no_requests", 0, ())
    return SportRequestAudit(True, "normal_gait_api_contract", len(selected), api_ids)


def audit_ros2_request_echo(
    text: str,
    *,
    minimum_request_id: Optional[int] = None,
    require_move: bool = False,
    require_stop: bool = False,
) -> SportRequestAudit:
    return audit_navigation_request_records(
        parse_ros2_request_echo(text),
        minimum_request_id=minimum_request_id,
        require_move=require_move,
        require_stop=require_stop,
    )
