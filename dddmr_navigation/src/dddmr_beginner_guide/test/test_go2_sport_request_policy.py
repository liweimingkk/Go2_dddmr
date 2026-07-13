#!/usr/bin/env python3

import json
import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))

from go2_sport_request_policy import (  # noqa: E402
    MOVE_API_ID,
    STOP_MOVE_API_ID,
    audit_ros2_request_echo,
    parse_ros2_request_echo,
    validate_navigation_sport_request,
)


def echo_block(request_id: int, api_id: int) -> str:
    return (
        "header:\n"
        "  identity:\n"
        f"    id: {request_id}\n"
        f"    api_id: {api_id}\n"
        "parameter: ''\n"
        "---\n"
    )


class NavigationSportRequestPolicyTest(unittest.TestCase):
    def test_move_and_stopmove_are_valid(self):
        validate_navigation_sport_request(
            MOVE_API_ID,
            json.dumps({"x": 0.2, "y": 0.0, "z": -0.1}),
        )
        validate_navigation_sport_request(STOP_MOVE_API_ID, "")

    def test_mode_or_gait_api_is_rejected(self):
        for api_id in (1001, 1002, 1004, 1016, 1020, 2000):
            with self.subTest(api_id=api_id):
                with self.assertRaisesRegex(ValueError, "outside"):
                    validate_navigation_sport_request(api_id, "")

    def test_move_payload_is_exact_and_finite(self):
        invalid_payloads = (
            "",
            "{}",
            '{"x":0.1,"y":0.0}',
            '{"x":0.1,"y":0.0,"z":0.0,"gait":1}',
            '{"x":true,"y":0.0,"z":0.0}',
            '{"x":NaN,"y":0.0,"z":0.0}',
        )
        for payload in invalid_payloads:
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    validate_navigation_sport_request(MOVE_API_ID, payload)

    def test_stopmove_payload_must_be_empty(self):
        with self.assertRaisesRegex(ValueError, "empty"):
            validate_navigation_sport_request(STOP_MOVE_API_ID, "{}")

    def test_echo_parser_and_allowed_sequence(self):
        text = echo_block(101, MOVE_API_ID) + echo_block(102, STOP_MOVE_API_ID)
        records = parse_ros2_request_echo(text)
        self.assertEqual([(r.request_id, r.api_id) for r in records], [(101, 1008), (102, 1003)])
        audit = audit_ros2_request_echo(text, require_move=True, require_stop=True)
        self.assertTrue(audit.allowed)
        self.assertEqual(audit.reason, "normal_gait_api_contract")

    def test_echo_audit_rejects_any_unexpected_api(self):
        text = (
            echo_block(101, MOVE_API_ID)
            + echo_block(102, 1002)
            + echo_block(103, STOP_MOVE_API_ID)
        )
        audit = audit_ros2_request_echo(text, require_move=True, require_stop=True)
        self.assertFalse(audit.allowed)
        self.assertEqual(audit.reason, "unexpected_api_ids=1002")

    def test_id_filter_does_not_hide_global_audit_events(self):
        text = echo_block(10, 1002) + echo_block(101, MOVE_API_ID) + echo_block(102, STOP_MOVE_API_ID)
        global_audit = audit_ros2_request_echo(text, require_stop=True)
        self.assertFalse(global_audit.allowed)
        owned_audit = audit_ros2_request_echo(
            text,
            minimum_request_id=100,
            require_move=True,
            require_stop=True,
        )
        self.assertTrue(owned_audit.allowed)

    def test_missing_required_requests_fail(self):
        move_only = audit_ros2_request_echo(
            echo_block(1, MOVE_API_ID), require_stop=True
        )
        self.assertEqual(move_only.reason, "missing_stopmove")
        empty = audit_ros2_request_echo("", require_stop=False)
        self.assertEqual(empty.reason, "no_requests")

    def test_incomplete_echo_identity_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "incomplete"):
            parse_ros2_request_echo("header:\n  identity:\n    api_id: 1008\n---\n")


if __name__ == "__main__":
    unittest.main()
