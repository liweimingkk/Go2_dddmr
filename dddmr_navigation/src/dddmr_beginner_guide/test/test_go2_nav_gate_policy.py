#!/usr/bin/env python3

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))

from go2_nav_gate_policy import localization_block_reason  # noqa: E402


class LocalizationGatePolicyTest(unittest.TestCase):
    def test_optional_localization_never_blocks(self):
        self.assertIsNone(localization_block_reason(False, None, None, 0.5))

    def test_missing_and_stale_status_block(self):
        self.assertEqual(
            localization_block_reason(True, None, None, 0.5),
            "localization_no_status",
        )
        self.assertEqual(
            localization_block_reason(True, "TRACKING", 0.6, 0.5),
            "localization_status_stale",
        )

    def test_only_fresh_tracking_allows_motion(self):
        self.assertIsNone(
            localization_block_reason(True, "TRACKING", 0.1, 0.5)
        )
        self.assertEqual(
            localization_block_reason(True, "LOCALIZING", 0.1, 0.5),
            "localization_localizing",
        )
        self.assertEqual(
            localization_block_reason(True, "LOST", 0.1, 0.5),
            "localization_lost",
        )


if __name__ == "__main__":
    unittest.main()
