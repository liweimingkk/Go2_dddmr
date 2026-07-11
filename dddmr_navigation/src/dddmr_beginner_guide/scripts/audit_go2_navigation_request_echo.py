#!/usr/bin/env python3

"""Offline audit for the normal-gait Go2 navigation Sport API contract."""

import argparse
from pathlib import Path
import sys

from go2_sport_request_policy import audit_ros2_request_echo


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fail unless every observed Go2 navigation Sport request is "
            "Move(1008) or StopMove(1003)."
        )
    )
    parser.add_argument("request_echo", type=Path)
    parser.add_argument(
        "--minimum-request-id",
        type=int,
        default=None,
        help="only audit identities greater than this value",
    )
    parser.add_argument("--require-move", action="store_true")
    parser.add_argument("--require-stop", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        text = args.request_echo.read_text(encoding="utf-8")
        audit = audit_ros2_request_echo(
            text,
            minimum_request_id=args.minimum_request_id,
            require_move=args.require_move,
            require_stop=args.require_stop,
        )
    except (OSError, UnicodeError, ValueError) as exc:
        print("NORMAL_GAIT_API_AUDIT=FAIL", file=sys.stderr)
        print("NORMAL_GAIT_API_REASON=%s" % str(exc), file=sys.stderr)
        return 1

    print("NORMAL_GAIT_API_AUDIT=%s" % ("PASS" if audit.allowed else "FAIL"))
    print("NORMAL_GAIT_API_REASON=%s" % audit.reason)
    print("NORMAL_GAIT_API_REQUEST_COUNT=%d" % audit.request_count)
    print(
        "NORMAL_GAIT_API_IDS=%s"
        % ",".join(str(api_id) for api_id in sorted(set(audit.api_ids)))
    )
    return 0 if audit.allowed else 1


if __name__ == "__main__":
    raise SystemExit(main())
