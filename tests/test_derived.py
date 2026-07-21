"""Derived channels must emit valid Foxglove well-known schema payloads."""

from __future__ import annotations

import json
import math

import pytest

from mavlink_foxglove import derived
from mavlink_foxglove.dialect import load_dialect

NS = 1_700_000_000_123_456_789


@pytest.fixture(scope="module")
def common():
    return load_dialect("common")


def test_global_position_becomes_location_fix(common):
    msg = common.MAVLink_global_position_int_message(
        1000, int(37.7749e7), int(-122.4194e7), 100_000, 50_000, 0, 0, 0, 0
    )
    (item,) = derived.convert("GLOBAL_POSITION_INT", msg, NS)
    assert item.schema_name == "foxglove.LocationFix"
    assert item.payload["latitude"] == pytest.approx(37.7749)
    assert item.payload["longitude"] == pytest.approx(-122.4194)
    assert item.payload["altitude"] == pytest.approx(100.0)  # mm -> m
    json.dumps(item.payload, allow_nan=False)


def test_attitude_becomes_frame_transform(common):
    msg = common.MAVLink_attitude_message(1000, 0.0, 0.0, math.pi / 2, 0.0, 0.0, 0.0)
    (item,) = derived.convert("ATTITUDE", msg, NS)
    assert item.schema_name == "foxglove.FrameTransform"
    rotation = item.payload["rotation"]
    # 90 degree yaw -> (0, 0, sin(45deg), cos(45deg))
    assert rotation["z"] == pytest.approx(math.sqrt(0.5))
    assert rotation["w"] == pytest.approx(math.sqrt(0.5))


def test_quaternion_is_normalized():
    q = derived.euler_to_quaternion(0.3, -0.2, 1.1)
    norm = math.sqrt(sum(component**2 for component in q.values()))
    assert norm == pytest.approx(1.0)


def test_statustext_becomes_log(common):
    msg = common.MAVLink_statustext_message(4, b"low battery", 0, 0)
    (item,) = derived.convert("STATUSTEXT", msg, NS)
    assert item.schema_name == "foxglove.Log"
    assert item.payload["message"] == "low battery"
    assert item.payload["level"] == 3  # MAV_SEVERITY_WARNING -> WARNING


def test_nan_attitude_is_skipped(common):
    """Non-finite input must not produce a payload with NaN in it."""
    msg = common.MAVLink_attitude_message(
        1000, float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0
    )
    assert derived.convert("ATTITUDE", msg, NS) == []


def test_unregistered_message_produces_nothing(common):
    msg = common.MAVLink_heartbeat_message(1, 1, 0, 0, 0, 3)
    assert derived.convert("HEARTBEAT", msg, NS) == []


def test_converter_errors_are_contained():
    """A malformed message degrades the derived topic only, never the bridge."""
    assert derived.convert("ATTITUDE", object(), NS) == []
