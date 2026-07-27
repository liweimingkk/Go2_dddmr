#!/usr/bin/env python3

import os
import pathlib
import subprocess
import tempfile
import unittest


WORKSPACE = pathlib.Path(__file__).resolve().parents[3]
CHECK_SCRIPT = WORKSPACE / "scripts" / "check_go2_dds_receive_buffers.sh"
DDS_SETUP = WORKSPACE / "scripts" / "setup_go2_dds_env.sh"
DOCKER_WRAPPER = WORKSPACE / "scripts" / "dddmr_docker_go2_xt16.sh"
MOUTH_MAPPING_WRAPPER = (
    WORKSPACE / "scripts" / "run_go2_xt16_mouth_mapping_save_to_nav.sh"
)


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

    def test_cyclone_receive_minimum_accepts_explicit_kernel_default(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=lo; "
            "GO2_DDS_RCVBUF_MIN=default; "
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
            '<SocketReceiveBufferSize min="default" max="16MiB" />',
            result.stdout,
        )

    def run_wrapper_with_fake_docker(self, platform: str):
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = pathlib.Path(temporary_directory)
            fake_bin = directory / "bin"
            fake_bin.mkdir()
            fake_docker = fake_bin / "docker"
            fake_docker.write_text(
                "#!/usr/bin/env bash\n"
                "if [[ \"$1\" == image && \"$2\" == inspect ]]; then\n"
                "  exit 0\n"
                "fi\n"
                "printf '%s\\n' \"$@\"\n",
                encoding="utf-8",
            )
            fake_docker.chmod(0o755)
            environment = os.environ.copy()
            environment.update(
                {
                    "PATH": f"{fake_bin}:{environment['PATH']}",
                    "DDDMR_PLATFORM": platform,
                    "DDDMR_DOCKER_RUNTIME": "none",
                    "DDDMR_BAGS_DIR": str(directory / "bags"),
                    "GO2_NET_IFACE": "lo",
                }
            )
            return subprocess.run(
                [str(DOCKER_WRAPPER), "preflight", "--samples", "1"],
                check=False,
                capture_output=True,
                text=True,
                env=environment,
            )

    def test_orin_wrapper_uses_kernel_receive_default(self):
        result = self.run_wrapper_with_fake_docker("orin-jp5")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("GO2_DDS_RCVBUF_MIN=default", result.stdout)

    def test_x64_wrapper_keeps_strict_receive_minimum(self):
        result = self.run_wrapper_with_fake_docker("x64")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("GO2_DDS_RCVBUF_MIN=16MiB", result.stdout)

    def test_mouth_mapping_containers_source_unitree_dds_overlay(self):
        script = MOUTH_MAPPING_WRAPPER.read_text(encoding="utf-8")
        overlay_then_config = (
            "if [[ -f /opt/unitree_ros2/setup.bash ]]; then\n"
            "  source /opt/unitree_ros2/setup.bash\n"
            "fi\n"
            "source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh"
        )
        self.assertEqual(script.count(overlay_then_config), 5)


if __name__ == "__main__":
    unittest.main()
