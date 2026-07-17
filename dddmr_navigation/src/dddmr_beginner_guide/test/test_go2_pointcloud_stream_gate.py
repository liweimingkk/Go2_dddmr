#!/usr/bin/env python3

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))

from go2_pointcloud_stream_gate import (  # noqa: E402
    POINTCLOUD2_TYPE,
    StreamThresholds,
    evaluate_stream,
)


class PointCloudStreamGateTest(unittest.TestCase):
    def setUp(self):
        self.thresholds = StreamThresholds(
            window_sec=3.0,
            min_samples=20,
            min_rate_hz=7.0,
            max_header_gap_sec=0.25,
            max_receive_gap_sec=0.20,
        )

    def evaluate(
        self,
        receipt_times,
        header_times=None,
        *,
        evaluation_time=3.05,
        publisher_count=1,
        topic_types=(POINTCLOUD2_TYPE,),
    ):
        if header_times is None:
            header_times = receipt_times
        return evaluate_stream(
            receipt_times=receipt_times,
            header_stamps_ns=[
                1_000_000_000 + int(value * 1_000_000_000)
                for value in header_times
            ],
            evaluation_time=evaluation_time,
            publisher_count=publisher_count,
            topic_types=topic_types,
            thresholds=self.thresholds,
        )

    def test_stable_ten_hz_stream_passes(self):
        times = [index / 10.0 for index in range(31)]
        result = self.evaluate(times)
        self.assertTrue(result.passed, result.reasons)
        self.assertAlmostEqual(result.receive_rate_hz, 10.0)
        self.assertAlmostEqual(result.header_rate_hz, 10.0)

    def test_no_samples_fails_closed(self):
        result = self.evaluate([], evaluation_time=8.0, publisher_count=0, topic_types=())
        self.assertFalse(result.passed)
        self.assertIn("no_samples", result.reasons)

    def test_insufficient_and_slow_samples_fail(self):
        times = [index / 2.0 for index in range(7)]
        result = self.evaluate(times)
        self.assertFalse(result.passed)
        self.assertTrue(any(reason.startswith("sample_count=") for reason in result.reasons))
        self.assertTrue(any(reason.startswith("receive_rate_hz=") for reason in result.reasons))

    def test_repeated_or_regressing_header_stamp_fails(self):
        receipts = [index / 10.0 for index in range(31)]
        for changed_stamp in (0.9, 0.8):
            with self.subTest(changed_stamp=changed_stamp):
                headers = list(receipts)
                headers[10] = changed_stamp
                result = self.evaluate(receipts, headers)
                self.assertFalse(result.passed)
                self.assertIn("header_stamps_not_strictly_increasing", result.reasons)

    def test_large_header_gap_fails_even_when_receive_rate_is_healthy(self):
        receipts = [index / 10.0 for index in range(31)]
        headers = list(receipts)
        for index in range(11, len(headers)):
            headers[index] += 0.20
        result = self.evaluate(receipts, headers)
        self.assertFalse(result.passed)
        self.assertTrue(any(reason.startswith("max_header_gap=") for reason in result.reasons))

    def test_large_receive_gap_fails_even_when_average_rate_is_healthy(self):
        receipts = [index / 10.0 for index in range(31)]
        for index in range(11, len(receipts)):
            receipts[index] += 0.15
        headers = [index / 10.0 for index in range(31)]
        result = self.evaluate(receipts, headers, evaluation_time=3.20)
        self.assertFalse(result.passed)
        self.assertGreater(result.receive_rate_hz, self.thresholds.min_rate_hz)
        self.assertTrue(any(reason.startswith("max_receive_gap=") for reason in result.reasons))

    def test_stale_tail_fails_after_an_initial_burst(self):
        times = [index / 10.0 for index in range(31)]
        result = self.evaluate(times, evaluation_time=3.60)
        self.assertFalse(result.passed)
        self.assertTrue(any(reason.startswith("tail_age=") for reason in result.reasons))

    def test_wrong_publisher_count_or_type_fails(self):
        times = [index / 10.0 for index in range(31)]
        duplicate = self.evaluate(times, publisher_count=2)
        self.assertFalse(duplicate.passed)
        self.assertTrue(any(reason.startswith("publisher_count=") for reason in duplicate.reasons))

        wrong_type = self.evaluate(times, topic_types=("std_msgs/msg/String",))
        self.assertFalse(wrong_type.passed)
        self.assertTrue(any(reason.startswith("topic_types=") for reason in wrong_type.reasons))


if __name__ == "__main__":
    unittest.main()
