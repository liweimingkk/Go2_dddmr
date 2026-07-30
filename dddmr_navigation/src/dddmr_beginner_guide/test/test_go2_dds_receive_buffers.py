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
NAVIGATION_TEST_WRAPPER = (
    WORKSPACE / "scripts" / "run_go2_xt16_navigation_test.sh"
)
LAPTOP_NAVIGATION_RVIZ_LAUNCHER = (
    WORKSPACE / "scripts" / "run_go2_xt16_laptop_navigation_rviz.sh"
)
LAPTOP_NAVIGATION_RVIZ_CONFIG = (
    WORKSPACE
    / "src"
    / "dddmr_beginner_guide"
    / "rviz"
    / "go2_xt16_navigation_laptop.rviz"
)
LAPTOP_MAPPING_RVIZ_LAUNCHER = (
    WORKSPACE / "scripts" / "run_go2_xt16_laptop_mapping_rviz.sh"
)
LAPTOP_MAPPING_RVIZ_CONFIG = (
    WORKSPACE
    / "src"
    / "dddmr_lego_loam"
    / "lego_loam_bor"
    / "rviz"
    / "go2_xt16_mapping_laptop.rviz"
)
MOUTH_MAPPING_CONFIG = (
    WORKSPACE
    / "src"
    / "dddmr_lego_loam"
    / "lego_loam_bor"
    / "config"
    / "loam_go2_xt16_mouth_config.yaml"
)
MOUTH_MAPPING_LAUNCH = (
    WORKSPACE
    / "src"
    / "dddmr_lego_loam"
    / "lego_loam_bor"
    / "launch"
    / "lego_loam_go2_xt16_mouth.launch"
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
    def test_auto_interface_uses_device_owning_local_dds_address(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            fake_ip = pathlib.Path(temporary_directory) / "ip"
            fake_ip.write_text(
                "#!/usr/bin/env bash\n"
                "if [[ \"$1 $2\" == 'route get' ]]; then\n"
                "  printf '%s\\n' "
                "'local 192.168.123.18 dev lo src 192.168.123.18'\n"
                "elif [[ \"$1 $2 $3\" == '-o -4 address' ]]; then\n"
                "  printf '%s\\n' "
                "'2: eth0 inet 192.168.123.18/24 scope global eth0'\n"
                "fi\n",
                encoding="utf-8",
            )
            fake_ip.chmod(0o755)
            command = (
                "set -u; "
                "unset GO2_NET_IFACE; "
                "GO2_DDS_IP=192.168.123.18; "
                f"source {DDS_SETUP}; "
                "printf '%s' \"${GO2_NET_IFACE}\""
            )
            environment = os.environ.copy()
            environment["PATH"] = (
                f"{temporary_directory}:{environment['PATH']}"
            )
            result = subprocess.run(
                ["bash", "-c", command],
                check=False,
                capture_output=True,
                text=True,
                env=environment,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "eth0")

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
            "GO2_DDS_PRIMARY_ADDRESS=192.168.123.18; "
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

    def test_cyclone_pins_robot_topics_to_primary_address_with_extra_iface(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=eth0; "
            "GO2_DDS_EXTRA_IFACES='wlan0'; "
            "GO2_DDS_PRIMARY_ADDRESS=192.168.123.18; "
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
            '<NetworkPartition Name="go2_primary" '
            'Address="192.168.123.18" />',
            result.stdout,
        )
        self.assertIn(
            'DCPSPartitionTopic="*.rt/utlidar/*" '
            'NetworkPartition="go2_primary"',
            result.stdout,
        )
        self.assertIn(
            'DCPSPartitionTopic="*.rt/uslam/*" '
            'NetworkPartition="go2_primary"',
            result.stdout,
        )
        element_tree.fromstring(result.stdout)

    def test_cyclone_does_not_add_network_partition_for_single_iface(self):
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
        self.assertNotIn("<Partitioning>", result.stdout)

    def test_cyclone_rejects_invalid_primary_address(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=eth0; "
            "GO2_DDS_EXTRA_IFACES='wlan0'; "
            "GO2_DDS_PRIMARY_ADDRESS='192.168.123.999'; "
            f"source {DDS_SETUP}"
        )
        result = subprocess.run(
            ["bash", "-c", command],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("must be an IPv4 address", result.stderr)

    def test_cyclone_rejects_unsafe_robot_topic_pattern(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=eth0; "
            "GO2_DDS_EXTRA_IFACES='wlan0'; "
            "GO2_DDS_PRIMARY_ADDRESS=192.168.123.18; "
            "GO2_DDS_ROBOT_TOPIC_PATTERNS='rt/utlidar/*<invalid'; "
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
        self.assertIn(
            "<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>",
            result.stdout,
        )

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

    def test_cyclone_rejects_unsafe_max_auto_participant_index(self):
        command = (
            "set -u; "
            "GO2_NET_IFACE=lo; "
            "GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX=121; "
            f"source {DDS_SETUP}"
        )
        result = subprocess.run(
            ["bash", "-c", command],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(
            "GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX",
            result.stderr,
        )

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
                "GO2_DDS_PRIMARY_ADDRESS",
                "GO2_DDS_ROBOT_TOPIC_PATTERNS",
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
        self.assertEqual(
            script.count(
                '-e "GO2_DDS_PRIMARY_ADDRESS='
                '${GO2_DDS_PRIMARY_ADDRESS_VALUE}"'
            ),
            3,
        )
        self.assertEqual(
            script.count(
                '-e "GO2_DDS_ROBOT_TOPIC_PATTERNS='
                '${GO2_DDS_ROBOT_TOPIC_PATTERNS_VALUE}"'
            ),
            3,
        )

    def test_navigation_container_forwards_additional_interfaces(self):
        script = NAVIGATION_TEST_WRAPPER.read_text(encoding="utf-8")
        self.assertEqual(
            script.count(
                '-e "GO2_DDS_EXTRA_IFACES='
                '${GO2_DDS_EXTRA_IFACES_VALUE}"'
            ),
            1,
        )
        self.assertEqual(
            script.count(
                '-e "GO2_DDS_PRIMARY_ADDRESS='
                '${GO2_DDS_PRIMARY_ADDRESS_VALUE}"'
            ),
            1,
        )
        self.assertEqual(
            script.count(
                '-e "GO2_DDS_ROBOT_TOPIC_PATTERNS='
                '${GO2_DDS_ROBOT_TOPIC_PATTERNS_VALUE}"'
            ),
            1,
        )
        self.assertIn(
            'GO2_DDS_EXTRA_IFACES="${GO2_DDS_EXTRA_IFACES_VALUE}"',
            script,
        )

    def test_navigation_supports_sudo_docker_on_orin(self):
        script = NAVIGATION_TEST_WRAPPER.read_text(encoding="utf-8")
        self.assertIn(
            'DDDMR_DOCKER_USE_SUDO_VALUE="${DDDMR_DOCKER_USE_SUDO:-0}"',
            script,
        )
        self.assertIn(
            'docker() {\n      sudo docker "$@"\n    }',
            script,
        )
        self.assertIn(
            '[[ "${mode}" == "stop" && -z "${DDDMR_DOCKER_USE_SUDO+x}"',
            script,
        )
        self.assertIn(
            'log "Docker socket is not writable; using sudo for --stop."',
            script,
        )
        self.assertIn("require_docker_access", script)
        self.assertIn(
            "Cannot access the Docker daemon. Set "
            "DDDMR_DOCKER_USE_SUDO=1 or run with Docker socket access.",
            script,
        )

    def test_navigation_uses_platform_specific_runtime_defaults(self):
        script = NAVIGATION_TEST_WRAPPER.read_text(encoding="utf-8")
        self.assertIn(
            'DEFAULT_IMAGE="dddmr_go2_xt16:orin-jp5.1.1"',
            script,
        )
        self.assertIn('DEFAULT_ROS_DISTRO="foxy"', script)
        self.assertIn(
            'DEFAULT_INSTALL_BASE=".docker_go2_xt16_orin_install"',
            script,
        )
        self.assertIn('DEFAULT_GO2_DDS_RCVBUF_MIN="default"', script)
        self.assertIn('DOCKER_RUN_ARGS+=(--runtime nvidia)', script)
        self.assertIn(
            '-e "GO2_DDS_RCVBUF_MIN=${GO2_DDS_RCVBUF_MIN_VALUE}"',
            script,
        )
        self.assertIn(
            '-e "GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX='
            '${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX_VALUE}"',
            script,
        )
        self.assertNotIn("/opt/ros/humble/setup.bash", script)
        overlay_then_config = (
            "if [[ -f /opt/unitree_ros2/setup.bash ]]; then\n"
            "  source /opt/unitree_ros2/setup.bash\n"
            "fi\n"
            "source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh"
        )
        self.assertEqual(script.count(overlay_then_config), 3)
        self.assertNotIn("ros2 topic echo --once --field", script)
        self.assertIn("--no-arr --no-str", script)
        self.assertIn("non-empty /map1/mapground consumption", script)
        self.assertIn("non-empty /map1/planning_ground consumption", script)
        self.assertIn("weighted planning-ground publication", script)
        log_waiter = script.split(
            "wait_for_container_log_pattern() {", 1
        )[1].split("\n}\n\ncheck_odom_sync_runtime()", 1)[0]
        self.assertIn('docker logs "${CONTAINER_NAME}"', log_waiter)
        self.assertIn("grep -m 1 -E", log_waiter)
        self.assertNotIn("docker logs --follow", log_waiter)
        self.assertNotIn("docker logs --tail 2000", log_waiter)

    def test_navigation_live_adapter_uses_available_unitree_overlay(self):
        script = NAVIGATION_TEST_WRAPPER.read_text(encoding="utf-8")
        adapter = script.split("start_live_adapter() {", 1)[1].split(
            "\n}\n\nprint_status()", 1
        )[0]
        self.assertIn(
            "if [[ -f /opt/unitree_ros2/setup.bash ]]; then\n"
            "  source /opt/unitree_ros2/setup.bash\n"
            "elif [[ -f /root/dddmr_navigation/"
            ".unitree_msg_ws/install/setup.bash ]]; then\n"
            "  source /root/dddmr_navigation/"
            ".unitree_msg_ws/install/setup.bash\n"
            "else\n",
            adapter,
        )
        self.assertNotIn(
            "test -f /root/dddmr_navigation/"
            ".unitree_msg_ws/install/setup.bash",
            adapter,
        )
        self.assertIn(
            "python3 -c 'from unitree_api.msg import Request'",
            adapter,
        )

    def test_laptop_navigation_rviz_uses_standard_goal_tools(self):
        config = LAPTOP_NAVIGATION_RVIZ_CONFIG.read_text(encoding="utf-8")
        self.assertNotIn("dddmr_rviz_default_plugins", config)
        self.assertIn("Class: rviz_default_plugins/SetInitialPose", config)
        self.assertIn("Class: rviz_default_plugins/SetGoal", config)
        self.assertIn("Value: initial_3d_pose", config)
        self.assertIn("Value: goal_pose_3d", config)

    def test_laptop_navigation_rviz_launcher_is_wifi_scoped(self):
        launcher = LAPTOP_NAVIGATION_RVIZ_LAUNCHER.read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'ORIN_OPERATOR_IP="${GO2_ORIN_OPERATOR_IP:-192.168.50.1}"',
            launcher,
        )
        self.assertIn(
            'export GO2_DDS_EXTRA_IFACES="${GO2_LAPTOP_DDS_EXTRA_IFACES:-}"',
            launcher,
        )
        self.assertIn("go2_xt16_navigation_laptop.rviz", launcher)
        self.assertIn("2D Goal Pose -> /goal_pose_3d", launcher)

    def test_laptop_mapping_rviz_is_read_only_and_bandwidth_bounded(self):
        config = LAPTOP_MAPPING_RVIZ_CONFIG.read_text(encoding="utf-8")
        self.assertIn("Value: /lego_loam_map", config)
        self.assertIn("Value: /lego_loam_ground", config)
        self.assertIn("Value: /mouth_ground_cloud", config)
        self.assertIn("Value: /key_poses", config)
        self.assertIn("Value: /pose_graph", config)
        self.assertNotIn("Value: /lidar_points", config)
        self.assertNotIn("rviz_default_plugins/SetGoal", config)
        self.assertNotIn("rviz_default_plugins/SetInitialPose", config)
        self.assertNotIn("rviz_default_plugins/PublishPoint", config)
        self.assertNotIn("/cmd_vel", config)
        self.assertNotIn("/api/sport/request", config)

    def test_laptop_mapping_rviz_launcher_checks_live_map_stream(self):
        launcher = LAPTOP_MAPPING_RVIZ_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn(
            'ORIN_OPERATOR_IP="${GO2_ORIN_OPERATOR_IP:-192.168.50.1}"',
            launcher,
        )
        self.assertIn(
            "REQUIRED_TOPICS=(/lego_loam_map /lego_loam_ground)",
            launcher,
        )
        self.assertIn("ros2 topic info", launcher)
        self.assertIn("--no-daemon", launcher)
        self.assertIn("--field width", launcher)
        self.assertIn("GO2_DDS_EXTRA_IFACES=wlan0", launcher)
        self.assertIn("go2_xt16_mapping_laptop.rviz", launcher)
        self.assertNotIn("ros2 topic pub", launcher)

    def test_live_mouth_mapping_uses_receipt_time_sync(self):
        config = MOUTH_MAPPING_CONFIG.read_text(encoding="utf-8")
        self.assertIn('mouth_sync_mode: "receipt_time"', config)

    def test_mouth_mapping_standardizes_odom_with_measured_offset(self):
        script = MOUTH_MAPPING_WRAPPER.read_text(encoding="utf-8")
        self.assertIn("standardize_odom:=true", script)
        self.assertIn(
            "odom_topic:=/dddmr_go2/robot_odom_standard", script
        )

        root = element_tree.parse(MOUTH_MAPPING_LAUNCH).getroot()
        standardizer = root.find(
            ".//node[@exec='go2_odom_standardizer']"
        )
        self.assertIsNotNone(standardizer)
        offset = standardizer.find(
            "./param[@name='stamp_time_offset_sec']"
        )
        self.assertIsNotNone(offset)
        self.assertEqual(offset.attrib["value"], "$(var odom_time_offset_sec)")

    def test_mouth_ground_sample_check_is_foxy_compatible(self):
        script = MOUTH_MAPPING_WRAPPER.read_text(encoding="utf-8")
        self.assertNotIn("ros2 topic echo --once --field", script)
        self.assertIn("--qos-reliability reliable --no-arr --no-str", script)
        self.assertIn("width:", script)

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
