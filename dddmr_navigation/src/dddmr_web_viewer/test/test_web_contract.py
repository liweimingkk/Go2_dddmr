"""Static contract tests for browser safety and DDDMR topic integration."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_web_application_uses_bridge_topics_and_has_no_velocity_publisher():
    source = (PACKAGE_ROOT / "web" / "main.js").read_text(encoding="utf-8")

    assert "/dddmr_web/preview_goal" in source
    assert "/dddmr_web/execute_preview" in source
    assert "/dddmr_web/cancel_navigation" in source
    assert "/dddmr_web/initial_pose" in source
    assert "cmd_vel" not in source
    assert "/api/sport/request" not in source


def test_launch_keeps_navigation_execution_disabled_by_default():
    source = (
        PACKAGE_ROOT / "launch" / "go2_xt16_web_viewer.launch.py"
    ).read_text(encoding="utf-8")

    assert '"allow_navigation_execution"' in source
    assert 'default_value="false"' in source
    assert 'default_value="127.0.0.1"' in source
