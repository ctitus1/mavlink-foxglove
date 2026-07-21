"""Schema generation must hold for every message in every supported dialect."""

from __future__ import annotations

import json

import pytest

from mavlink_foxglove.dialect import load_dialect, message_classes, message_name
from mavlink_foxglove.schema import META_KEY, field_schema, message_schema

DIALECTS = ["common", "ardupilotmega"]


@pytest.fixture(scope="module")
def common():
    return load_dialect("common")


def test_float_fields_allow_null(common):
    """NaN encodes to null, so the schema must permit null."""
    schema = message_schema(common.MAVLink_attitude_message)
    assert schema["properties"]["roll"]["type"] == ["number", "null"]


def test_char_array_is_a_string(common):
    schema = message_schema(common.MAVLink_statustext_message)
    text = schema["properties"]["text"]
    assert text["type"] == "string"
    assert text["maxLength"] == 50


def test_numeric_array_is_an_array():
    node = field_schema("float", array_length=4, unit=None, enum=None)
    assert node["type"] == "array"
    assert node["items"]["type"] == ["number", "null"]
    assert node["maxItems"] == 4


def test_enum_companion_field(common):
    schema = message_schema(common.MAVLink_gps_raw_int_message, enum_names=True)
    assert schema["properties"]["fix_type"]["type"] == "integer"
    assert schema["properties"]["fix_type_enum"]["type"] == ["string", "null"]


def test_bitmask_field_gets_an_array_companion(common):
    """Bitmask fields need a list of flags, not a single symbolic name."""
    schema = message_schema(common.MAVLink_heartbeat_message)
    assert "base_mode_enum" not in schema["properties"]
    flags = schema["properties"]["base_mode_flags"]
    assert flags["type"] == "array"
    assert flags["items"]["type"] == "string"
    # Non-bitmask enum fields in the same message keep the singular form.
    assert schema["properties"]["system_status_enum"]["type"] == ["string", "null"]


def test_enum_companion_can_be_disabled(common):
    schema = message_schema(common.MAVLink_gps_raw_int_message, enum_names=False)
    assert "fix_type_enum" not in schema["properties"]


def test_units_are_documented(common):
    schema = message_schema(common.MAVLink_gps_raw_int_message)
    assert "degE7" in schema["properties"]["lat"]["description"]


def test_64bit_precision_is_flagged(common):
    schema = message_schema(common.MAVLink_system_time_message)
    assert "2^53" in schema["properties"]["time_unix_usec"]["description"]


@pytest.mark.parametrize("dialect_name", DIALECTS)
def test_every_message_yields_serializable_schema(dialect_name):
    """The whole point: no message type may break schema generation."""
    dialect = load_dialect(dialect_name)
    classes = message_classes(dialect)
    assert len(classes) > 100, "dialect looks empty"

    for msg_class in classes:
        schema = message_schema(msg_class)
        # Must survive JSON serialization -- this is what goes on the wire.
        json.dumps(schema)

        assert schema["type"] == "object"
        assert schema["title"] == message_name(msg_class)
        assert META_KEY in schema["properties"]

        for name in msg_class.fieldnames:
            assert name in schema["properties"], f"{schema['title']}.{name} missing"


def test_metadata_orderings_are_untangled(common):
    """Regression guard for pymavlink's two conflicting field orderings.

    PARAM_REQUEST_READ is the canonical trap: fieldtypes (fieldnames order) has
    `char` at index 2 = param_id, but array_lengths (wire order) has that 16 at
    index 3. Reading both with one index turns param_id into a scalar char and
    param_index into a 16-element array.
    """
    schema = message_schema(common.MAVLink_param_request_read_message)
    assert schema["properties"]["param_id"]["type"] == "string"
    assert schema["properties"]["param_id"]["maxLength"] == 16
    assert schema["properties"]["param_index"]["type"] == "integer"

    # GPS_RAW_INT: fix_type is uint8 at a different index in each ordering.
    gps = message_schema(common.MAVLink_gps_raw_int_message)
    assert gps["properties"]["fix_type"]["type"] == "integer"
    assert gps["properties"]["lat"]["type"] == "integer"


@pytest.mark.parametrize("dialect_name", DIALECTS)
def test_every_char_field_is_typed_as_a_string(dialect_name):
    """No char[N] field may be mistyped, in any message of any dialect."""
    dialect = load_dialect(dialect_name)
    checked = 0
    for msg_class in message_classes(dialect):
        schema = message_schema(msg_class)
        for index, name in enumerate(msg_class.fieldnames):
            if msg_class.fieldtypes[index] == "char":
                node = schema["properties"][name]
                assert node["type"] == "string", f"{message_name(msg_class)}.{name}"
                checked += 1
    assert checked > 30, "expected many char fields to be covered"
