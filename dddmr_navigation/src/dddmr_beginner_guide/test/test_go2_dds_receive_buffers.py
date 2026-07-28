#!/usr/bin/env python3

import os
import pathlib
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as element_tree


WORKSPACE = pathlib.Path(__file__).resolve().parents[3]
CHECK_SCRIPT = WORKSPACE / "scripts" / "check_go2_dds_receive_buffers.sh"
DDS_SETUP = WORKSPACE / "scripts" / "setup_go2_dds_env.sh"
DOCKER_WRAPPER = WORKSPACE / "scripts" / "dddmr_docker_go2_xt16.sh"
MOUTH_MAPPING_WRAPPER = (
    WORKSPACE / "scripts" / "run_go2_xt16_mouth_mapping_save_to_nav.sh"
)
GO2_LAUNCH_FILES = (
    WORKSPACE
    / "src"
    / "dddmr_beginner_guide"
    / "launch"
    / "go2_xt16_navigation.launch",
    WORKSPACE
    / "src"
    / "dddmr_lego_loam"
    / "lego_loam_bor"
    / "launch"
    / "lego_loam_go2_xt16_live.launch",
    WORKSPACE
    / "src"
    / "dddmr_lego_loam"
    / "lego_loam_bor"
    / "launch"
    / "lego_loam_go2_xt16_mouth.launch",
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
            "unset GO2_DDS_RCVBUF_MIN; "
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

    def test_cyclone_supports_explicit_additional_interfaces(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=eth0; "
            "GO2_DDS_EXTRA_IFACES='wlan0,eth0 wlan0'; "
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
        self.assertEqual(
            result.stdout.count('<NetworkInterface name="eth0"'), 1
        )
        self.assertEqual(
            result.stdout.count('<NetworkInterface name="wlan0"'), 1
        )

    def test_cyclone_rejects_unsafe_interface_names(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=eth0; "
            "GO2_DDS_EXTRA_IFACES='wlan0<invalid'; "
            f"source {DDS_SETUP}"
        )
        result = subprocess.run(
            ["bash", "-c", command],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsupported characters", result.stderr)

    def test_cyclone_supports_static_discovery_peers(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=lo; "
            "GO2_DDS_PEERS='192.168.123.18,192.168.123.18'; "
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
        self.assertEqual(
            result.stdout.count('<Peer Address="192.168.123.18"'), 1
        )
        self.assertIn("<ParticipantIndex>auto</ParticipantIndex>", result.stdout)

    def test_cyclone_rejects_unsafe_discovery_peers(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=lo; "
            "GO2_DDS_PEERS='192.168.123.18<invalid'; "
            f"source {DDS_SETUP}"
        )
        result = subprocess.run(
            ["bash", "-c", command],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsupported characters", result.stderr)

    def test_cyclone_rejects_invalid_participant_index(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=lo; "
            "GO2_DDS_PARTICIPANT_INDEX='-1'; "
            f"source {DDS_SETUP}"
        )
        result = subprocess.run(
            ["bash", "-c", command],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("GO2_DDS_PARTICIPANT_INDEX", result.stderr)

    def run_wrapper_with_fake_docker(
        self, platform: str, extra_interfaces: str = ""
    ):
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
            for inherited_name in (
                "DDDMR_BASE_IMAGE",
                "DDDMR_BUILD_BASE",
                "DDDMR_IMAGE",
                "DDDMR_INSTALL_BASE",
                "DDDMR_LOG_BASE",
                "DDDMR_ROS_DISTRO",
                "GO2_DDS_RCVBUF_MIN",
                "GO2_DDS_EXTRA_IFACES",
            ):
                environment.pop(inherited_name, None)
            environment.update(
                {
                    "PATH": f"{fake_bin}:{environment['PATH']}",
                    "DDDMR_PLATFORM": platform,
                    "DDDMR_DOCKER_RUNTIME": "none",
                    "DDDMR_BAGS_DIR": str(directory / "bags"),
                    "GO2_NET_IFACE": "lo",
                    "GO2_DDS_EXTRA_IFACES": extra_interfaces,
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

    def test_wrapper_forwards_additional_interfaces(self):
        result = self.run_wrapper_with_fake_docker("orin-jp5", "wlan0")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("GO2_DDS_EXTRA_IFACES=wlan0", result.stdout)

    def test_mouth_mapping_containers_source_unitree_dds_overlay(self):
        script = MOUTH_MAPPING_WRAPPER.read_text(encoding="utf-8")
        overlay_then_config = (
            "if [[ -f /opt/unitree_ros2/setup.bash ]]; then\n"
            "  source /opt/unitree_ros2/setup.bash\n"
            "fi\n"
            "source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh"
        )
        self.assertEqual(script.count(overlay_then_config), 5)

    def test_mouth_mapping_containers_forward_additional_interfaces(self):
        script = MOUTH_MAPPING_WRAPPER.read_text(encoding="utf-8")
        forwarded_environment = (
            '-e "GO2_DDS_EXTRA_IFACES=${GO2_DDS_EXTRA_IFACES_VALUE}"'
        )
        self.assertEqual(script.count(forwarded_environment), 3)

    def test_go2_static_transforms_use_foxy_compatible_arguments(self):
        publisher_count = 0
        for launch_file in GO2_LAUNCH_FILES:
            root = element_tree.parse(launch_file).getroot()
            publishers = root.findall(
                ".//node[@exec='static_transform_publisher']"
            )
            publisher_count += len(publishers)
            for publisher in publishers:
                arguments = publisher.attrib["args"]
                self.assertNotIn("--", arguments, launch_file)
        self.assertEqual(publisher_count, 8)


if __name__ == "__main__":
    unittest.main()
