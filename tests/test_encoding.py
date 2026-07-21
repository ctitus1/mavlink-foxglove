"""Encoding must always produce strictly-valid JSON, for any message."""

from __future__ import annotations

import json

import pytest

from mavlink_foxglove.dialect import load_dialect, message_classes, message_name
from mavlink_foxglove.encoding import encode_message, enum_label, sanitize, topic_for
from mavlink_foxglove.schema import META_KEY, field_specs

NS = 1_700_000_000_123_456_789


@pytest.fixture(scope="module")
def common():
    return load_dialect("common")


def _received(msg, system=1, component=1):
    """Stamp a synthetic message with the header a wire-decoded one would carry."""
    msg._header = type(
        "Header",
        (),
        {
            "srcSystem": system,
            "srcComponent": component,
            "seq": 5,
            "msgId": msg.get_msgId() if hasattr(msg, "_header") else type(msg).id,
        },
    )()
    return msg


@pytest.mark.parametrize(
    "value,expected",
    [
        (float("nan"), None),
        (float("inf"), None),
        (float("-inf"), None),
        (1.5, 1.5),
        (b"abc\x00def", "abc"),
        ("text\x00pad", "text"),
        ([1.0, float("nan")], [1.0, None]),
    ],
)
def test_sanitize(value, expected):
    assert sanitize(value) == expected


def test_sanitize_survives_invalid_utf8():
    assert isinstance(sanitize(b"\xff\xfe"), str)


def test_enum_label(common):
    assert enum_label(common, "GPS_FIX_TYPE", 3) == "GPS_FIX_TYPE_3D_FIX"


def test_enum_label_unknown_value_is_none(common):
    """A vehicle on a newer dialect must not break encoding."""
    assert enum_label(common, "GPS_FIX_TYPE", 9999) is None
    assert enum_label(common, "NO_SUCH_ENUM", 1) is None


def test_encode_puts_fields_at_top_level(common):
    msg = _received(common.MAVLink_attitude_message(1000, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0))
    out = encode_message(msg, common, NS)
    assert out["roll"] == pytest.approx(0.1)
    assert out[META_KEY]["system_id"] == 1
    assert out[META_KEY]["receive_timestamp"]["sec"] == NS // 1_000_000_000


def test_encode_nan_becomes_null_and_json_is_valid(common):
    msg = _received(
        common.MAVLink_attitude_message(1000, float("nan"), 0.2, 0.3, 0.0, 0.0, 0.0)
    )
    out = encode_message(msg, common, NS)
    assert out["roll"] is None
    # allow_nan=False is what the bridge uses; bare NaN would raise here.
    json.loads(json.dumps(out, allow_nan=False))


def test_encode_adds_enum_names(common):
    msg = _received(
        common.MAVLink_gps_raw_int_message(0, 3, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0)
    )
    out = encode_message(msg, common, NS)
    assert out["fix_type"] == 3
    assert out["fix_type_enum"] == "GPS_FIX_TYPE_3D_FIX"


def test_topic_template(common):
    msg = _received(
        common.MAVLink_attitude_message(0, 0, 0, 0, 0, 0, 0), system=2, component=42
    )
    topic = topic_for(msg, "/mavlink/{system_id}/{component_id}/{message}")
    assert topic == "/mavlink/2/42/ATTITUDE"


def _synthesize(msg_class):
    """Build an instance of any message class with awkward but legal values."""
    fill = {
        "uint8_t": 7, "uint8_t_mavlink_version": 3, "int8_t": -7,
        "uint16_t": 1234, "int16_t": -1234,
        "uint32_t": 123456, "int32_t": -123456,
        "uint64_t": 9_007_199_254_740_993, "int64_t": -9_007_199_254_740_993,
        "double": 2.25, "char": "x",
    }
    args = []
    for _name, mav_type, length in field_specs(msg_class):
        if length and mav_type == "char":
            # Generated constructors NUL-split char[N] as bytes, even though
            # the resulting attribute is a str.
            args.append(b"abc"[:length])
        elif length:
            args.append([fill.get(mav_type, 0.0)] * length)
        elif mav_type == "float":
            # Every float field gets NaN: the harshest case for JSON validity.
            args.append(float("nan"))
        else:
            args.append(fill.get(mav_type, 0))
    return msg_class(*args)


@pytest.mark.parametrize("dialect_name", ["common", "ardupilotmega"])
def test_every_message_encodes_to_valid_json(dialect_name):
    """The robustness guarantee: no message type may emit invalid JSON."""
    dialect = load_dialect(dialect_name)
    for msg_class in message_classes(dialect):
        msg = _received(_synthesize(msg_class))
        out = encode_message(msg, dialect, NS)
        # allow_nan=False makes non-compliant JSON a hard failure.
        encoded = json.dumps(out, allow_nan=False)
        assert json.loads(encoded)[META_KEY]["system_id"] == 1, message_name(msg_class)
