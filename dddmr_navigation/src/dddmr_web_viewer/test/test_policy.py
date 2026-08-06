"""Tests for fail-closed web navigation policy."""

from dddmr_web_viewer.policy import (
    PreviewAuthorization,
    position_is_finite,
    quaternion_is_valid,
)


def test_preview_authorization_requires_completed_preview():
    gate = PreviewAuthorization(max_age_sec=30.0)

    allowed, reason = gate.authorize(1_000_000_000)

    assert not allowed
    assert "no completed preview" in reason


def test_preview_authorization_is_fresh_and_one_shot():
    gate = PreviewAuthorization(max_age_sec=30.0)
    gate.record(1_000_000_000)

    assert gate.authorize(20_000_000_000)[0]
    allowed, reason = gate.authorize(21_000_000_000)

    assert not allowed
    assert "already executed" in reason


def test_preview_authorization_expires():
    gate = PreviewAuthorization(max_age_sec=30.0)
    gate.record(1_000_000_000)

    allowed, reason = gate.authorize(31_000_000_001)

    assert not allowed
    assert "expired" in reason


def test_pose_components_must_be_finite_and_unit_length():
    assert position_is_finite(1.0, 2.0, 3.0)
    assert not position_is_finite(float("nan"), 2.0, 3.0)
    assert quaternion_is_valid(0.0, 0.0, 0.0, 1.0)
    assert not quaternion_is_valid(0.0, 0.0, 0.0, 0.0)
