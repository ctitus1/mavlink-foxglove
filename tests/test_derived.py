"""Derived channels must emit valid Foxglove well-known schema payloads."""

from __future__ import annotations

import json
import math

import pytest

from mavlink_foxglove import derived
from mavlink_foxglove.derived import DerivedPublisher
from mavlink_foxglove.dialect import load_dialect

NS = 1_700_000_000_123_456_789


@pytest.fixture(scope="module")
def common():
    return load_dialect("common")


@pytest.fixture
def publisher():
    return DerivedPublisher()


def _received(msg, system=1, component=1):
    """Stamp a synthetic message with the header a wire-decoded one carries."""
    msg._header = type(
        "Header",
        (),
        {"srcSystem": system, "srcComponent": component, "seq": 0, "msgId": type(msg).id},
    )()
    return msg


# -- coordinate conventions ----------------------------------------------


def test_ned_translation_becomes_enu():
    """10m North, 5m East, 3m Down -> ENU x=East, y=North, z=Up."""
    assert derived.ned_to_enu(10.0, 5.0, 3.0) == {"x": 5.0, "y": 10.0, "z": -3.0}


@pytest.mark.parametrize(
    "yaw_ned,expected_heading",
    [
        (0.0, (0.0, 1.0)),            # North -> +Y in ENU
        (math.pi / 2, (1.0, 0.0)),    # East  -> +X in ENU
        (math.pi, (0.0, -1.0)),       # South -> -Y in ENU
        (-math.pi / 2, (-1.0, 0.0)),  # West  -> -X in ENU
    ],
)
def test_yaw_converts_from_ned_to_enu(yaw_ned, expected_heading):
    """A NED heading must point the right way once rendered in Foxglove's ENU.

    Rotating the body +X axis by the converted quaternion should give the
    expected ENU direction; this catches a mirrored or offset yaw.
    """
    q = derived.attitude_ned_to_enu_quaternion(0.0, 0.0, yaw_ned)
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    # Rotate the unit vector (1,0,0) by the quaternion.
    heading_x = 1 - 2 * (y * y + z * z)
    heading_y = 2 * (x * y + z * w)
    assert heading_x == pytest.approx(expected_heading[0], abs=1e-9)
    assert heading_y == pytest.approx(expected_heading[1], abs=1e-9)


def test_level_attitude_is_upright():
    """Level flight must not roll the model over (the Z-up/Z-down trap)."""
    q = derived.attitude_ned_to_enu_quaternion(0.0, 0.0, 0.0)
    # Rotate body +Z (up in FLU) and confirm it still points up in ENU.
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    up_z = 1 - 2 * (x * x + y * y)
    assert up_z == pytest.approx(1.0, abs=1e-9)


def test_pitch_up_raises_the_nose():
    """MAVLink pitch is positive nose-up; that must survive the conversion."""
    q = derived.attitude_ned_to_enu_quaternion(0.0, 0.2, 0.0)
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    # Body +X (forward) should gain a positive Z component in ENU.
    forward_z = 2 * (x * z - y * w)
    assert forward_z > 0


def test_quaternion_is_normalized():
    q = derived.euler_to_quaternion(0.3, -0.2, 1.1)
    norm = math.sqrt(sum(component**2 for component in q.values()))
    assert norm == pytest.approx(1.0)


# -- pose fusion ----------------------------------------------------------


def test_pose_fuses_attitude_and_position(common, publisher):
    """The transform must carry both halves, not whichever arrived last."""
    position = _received(
        common.MAVLink_local_position_ned_message(0, 10.0, 5.0, -3.0, 0, 0, 0)
    )
    (item,) = publisher.convert("LOCAL_POSITION_NED", position, NS)
    assert item.topic_suffix == "pose"
    assert item.payload["translation"] == {"x": 5.0, "y": 10.0, "z": 3.0}

    attitude = _received(
        common.MAVLink_attitude_message(0, 0.0, 0.0, math.pi / 2, 0, 0, 0)
    )
    (item,) = publisher.convert("ATTITUDE", attitude, NS)
    # Position from the earlier message must be retained.
    assert item.payload["translation"] == {"x": 5.0, "y": 10.0, "z": 3.0}
    assert item.payload["rotation"]["w"] == pytest.approx(1.0, abs=1e-9)


def test_pose_is_tracked_per_vehicle(common, publisher):
    """Two vehicles must not overwrite each other's pose."""
    a = _received(
        common.MAVLink_local_position_ned_message(0, 1.0, 0.0, 0.0, 0, 0, 0), system=1
    )
    b = _received(
        common.MAVLink_local_position_ned_message(0, 99.0, 0.0, 0.0, 0, 0, 0), system=2
    )
    publisher.convert("LOCAL_POSITION_NED", a, NS)
    publisher.convert("LOCAL_POSITION_NED", b, NS)

    (again,) = publisher.convert(
        "ATTITUDE",
        _received(common.MAVLink_attitude_message(0, 0.0, 0.0, 0.0, 0, 0, 0), system=1),
        NS,
    )
    assert again.payload["translation"]["y"] == pytest.approx(1.0)


def test_attitude_alone_still_publishes_a_pose(common, publisher):
    """A vehicle sending no local position must still appear, at the origin."""
    (item,) = publisher.convert(
        "ATTITUDE",
        _received(common.MAVLink_attitude_message(0, 0.0, 0.0, 0.0, 0, 0, 0)),
        NS,
    )
    assert item.payload["translation"] == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert item.payload["parent_frame_id"] == "map"
    assert item.payload["child_frame_id"] == "base_link"


def test_nan_pose_input_is_skipped(common, publisher):
    """Non-finite input must not corrupt the stored pose or emit NaN."""
    good = _received(
        common.MAVLink_local_position_ned_message(0, 7.0, 0.0, 0.0, 0, 0, 0)
    )
    publisher.convert("LOCAL_POSITION_NED", good, NS)

    bad = _received(
        common.MAVLink_attitude_message(0, float("nan"), 0.0, 0.0, 0, 0, 0)
    )
    assert publisher.convert("ATTITUDE", bad, NS) == []

    # The previously good position must be intact.
    (item,) = publisher.convert("LOCAL_POSITION_NED", good, NS)
    assert item.payload["translation"]["y"] == pytest.approx(7.0)


# -- location fixes -------------------------------------------------------


def test_global_position_becomes_location_fix(common, publisher):
    msg = _received(
        common.MAVLink_global_position_int_message(
            1000, int(37.7749e7), int(-122.4194e7), 100_000, 50_000, 0, 0, 0, 0
        )
    )
    (item,) = publisher.convert("GLOBAL_POSITION_INT", msg, NS)
    assert item.schema_name == "foxglove.LocationFix"
    assert item.payload["latitude"] == pytest.approx(37.7749)
    assert item.payload["longitude"] == pytest.approx(-122.4194)
    assert item.payload["altitude"] == pytest.approx(100.0)  # mm -> m
    json.dumps(item.payload, allow_nan=False)


def test_home_position_uses_its_own_field_names(common, publisher):
    """HOME_POSITION spells them latitude/longitude, not lat/lon."""
    msg = _received(
        common.MAVLink_home_position_message(
            int(37.5e7), int(-122.5e7), 25_000,
            0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0, 0,
        )
    )
    (item,) = publisher.convert("HOME_POSITION", msg, NS)
    assert item.topic_suffix == "home_location"
    assert item.payload["latitude"] == pytest.approx(37.5)
    assert item.payload["altitude"] == pytest.approx(25.0)


def test_ekf_origin_becomes_location_fix(common, publisher):
    msg = _received(
        common.MAVLink_gps_global_origin_message(int(37.5e7), int(-122.5e7), 25_000, 0)
    )
    (item,) = publisher.convert("GPS_GLOBAL_ORIGIN", msg, NS)
    assert item.topic_suffix == "ekf_origin_location"
    assert item.payload["latitude"] == pytest.approx(37.5)


def test_statustext_becomes_log(common, publisher):
    msg = _received(common.MAVLink_statustext_message(4, b"low battery", 0, 0))
    (item,) = publisher.convert("STATUSTEXT", msg, NS)
    assert item.schema_name == "foxglove.Log"
    assert item.payload["message"] == "low battery"
    assert item.payload["level"] == 3  # MAV_SEVERITY_WARNING -> WARNING


def test_unregistered_message_produces_nothing(common, publisher):
    msg = _received(common.MAVLink_heartbeat_message(1, 1, 0, 0, 0, 3))
    assert publisher.convert("HEARTBEAT", msg, NS) == []


def test_converter_errors_are_contained(publisher):
    """A malformed message degrades the derived topic only, never the bridge."""
    assert publisher.convert("ATTITUDE", object(), NS) == []
    assert publisher.convert("GLOBAL_POSITION_INT", object(), NS) == []
