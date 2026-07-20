#!/usr/bin/env python3

import pathlib
import subprocess
import tempfile
import unittest


WORKSPACE = pathlib.Path(__file__).resolve().parents[3]
CHECK_SCRIPT = WORKSPACE / "scripts" / "check_go2_dds_receive_buffers.sh"
DDS_SETUP = WORKSPACE / "scripts" / "setup_go2_dds_env.sh"


class Go2DdsReceiveBuffersTest(unittest.TestCase):
    def run_check(self, rmem_max: str, rmem_default: str):
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = pathlib.Path(temporary_directory)
            max_path = directory / "rmem_max"
            default_path = directory / "rmem_default"
            max_path.write_text(rmem_max, encoding="utf-8")
            default_path.write_text(rmem_default, encoding="utf-8")
            return subprocess.run(
                [
                    str(CHECK_SCRIPT),
                    "--rmem-max-path",
                    str(max_path),
                    "--rmem-default-path",
                    str(default_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

    def test_sufficient_buffers_pass(self):
        result = self.run_check("16777216\n", "33554432\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("GO2_DDS_RECEIVE_BUFFER_CHECK=PASS", result.stdout)

    def test_small_max_buffer_fails_with_remediation(self):
        result = self.run_check("212992\n", "16777216\n")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("net.core.rmem_max=16777216", result.stderr)
        self.assertIn("No ROS process or physical motion output was started", result.stderr)

    def test_small_default_buffer_fails(self):
        result = self.run_check("16777216\n", "212992\n")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("net.core.rmem_default=16777216", result.stderr)

    def test_malformed_kernel_value_fails_closed(self):
        result = self.run_check("invalid\n", "16777216\n")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("is not a positive integer", result.stderr)

    def test_missing_kernel_path_fails_closed(self):
        result = subprocess.run(
            [
                str(CHECK_SCRIPT),
                "--rmem-max-path",
                "/definitely/missing/rmem_max",
                "--rmem-default-path",
                "/definitely/missing/rmem_default",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("cannot read net.core.rmem_max", result.stderr)

    def test_legacy_environment_path_override_is_ignored(self):
        script = CHECK_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("GO2_DDS_RMEM_MAX_PATH", script)
        self.assertNotIn("GO2_DDS_RMEM_DEFAULT_PATH", script)

    def test_cyclone_receive_minimum_is_numeric(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=lo; "
            f"source {DDS_SETUP}; "
            "printf '%s' \"${CYCLONEDDS_URI}\""
        )
        result = subprocess.run(
            ["bash", "-c", command],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(
            '<SocketReceiveBufferSize min="16MiB" max="16MiB" />',
            result.stdout,
        )
        self.assertNotIn('<SocketReceiveBufferSize min="default"', result.stdout)


if __name__ == "__main__":
    unittest.main()
