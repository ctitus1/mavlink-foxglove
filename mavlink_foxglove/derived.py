"""Derived channels using Foxglove's well-known schemas.

The generic MAVLink channels are complete but untyped as far as Foxglove is
concerned, so they only drive the Raw Message, Table and Plot panels. Foxglove
recognises a small set of schema *names* and wires them into richer panels --
Map, 3D and Log. This module republishes a handful of MAVLink messages under
those names so those panels work out of the box.

Each converter is a pure function of ``(message, receive_time_ns)``, which
keeps them trivially unit-testable and makes adding a new one a local change.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Iterable

#: MAV_SEVERITY (0 = emergency .. 7 = debug) -> foxglove.LogLevel
#: (1 = debug, 2 = info, 3 = warning, 4 = error, 5 = fatal).
_SEVERITY_TO_LOG_LEVEL = {0: 5, 1: 5, 2: 4, 3: 4, 4: 3, 5: 2, 6: 2, 7: 1}

_TIMESTAMP_SCHEMA = {
    "type": "object",
    "properties": {"sec": {"type": "integer"}, "nsec": {"type": "integer"}},
}
_VECTOR3_SCHEMA = {
    "type": "object",
    "properties": {
        "x": {"type": "number"},
        "y": {"type": "number"},
        "z": {"type": "number"},
    },
}
_QUATERNION_SCHEMA = {
    "type": "object",
    "properties": {
        "x": {"type": "number"},
        "y": {"type": "number"},
        "z": {"type": "number"},
        "w": {"type": "number"},
    },
}

LOCATION_FIX_SCHEMA = {
    "type": "object",
    "title": "foxglove.LocationFix",
    "properties": {
        "timestamp": _TIMESTAMP_SCHEMA,
        "frame_id": {"type": "string"},
        "latitude": {"type": "number"},
        "longitude": {"type": "number"},
        "altitude": {"type": "number"},
        "position_covariance": {"type": "array", "items": {"type": "number"}},
        "position_covariance_type": {"type": "integer"},
    },
}

FRAME_TRANSFORM_SCHEMA = {
    "type": "object",
    "title": "foxglove.FrameTransform",
    "properties": {
        "timestamp": _TIMESTAMP_SCHEMA,
        "parent_frame_id": {"type": "string"},
        "child_frame_id": {"type": "string"},
        "translation": _VECTOR3_SCHEMA,
        "rotation": _QUATERNION_SCHEMA,
    },
}

LOG_SCHEMA = {
    "type": "object",
    "title": "foxglove.Log",
    "properties": {
        "timestamp": _TIMESTAMP_SCHEMA,
        "level": {"type": "integer"},
        "message": {"type": "string"},
        "name": {"type": "string"},
        "file": {"type": "string"},
        "line": {"type": "integer"},
    },
}


@dataclass(frozen=True)
class DerivedMessage:
    """One derived publication: where it goes, its schema, and its payload."""

    topic_suffix: str
    schema_name: str
    schema: dict[str, Any]
    payload: dict[str, Any]


def _timestamp(ns: int) -> dict[str, int]:
    return {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000}


def _finite(value: Any) -> float | None:
    """Return ``value`` as a float, or ``None`` if it is not finite."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def euler_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> dict[str, float]:
    """Convert MAVLink's ZYX intrinsic Euler angles (radians) to a quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def _location_fix(
    msg: Any, ns: int, *, suffix: str, frame_id: str, alt_field: str
) -> Iterable[DerivedMessage]:
    """Shared builder for the lat/lon/alt-in-integer-units MAVLink messages."""
    lat, lon = _finite(msg.lat), _finite(msg.lon)
    if lat is None or lon is None:
        return ()
    altitude = _finite(getattr(msg, alt_field, None))
    return (
        DerivedMessage(
            topic_suffix=suffix,
            schema_name="foxglove.LocationFix",
            schema=LOCATION_FIX_SCHEMA,
            payload={
                "timestamp": _timestamp(ns),
                "frame_id": frame_id,
                "latitude": lat * 1e-7,
                "longitude": lon * 1e-7,
                # MAVLink reports these altitudes in millimetres.
                "altitude": (altitude or 0.0) / 1000.0,
                "position_covariance": [0.0] * 9,
                "position_covariance_type": 0,  # UNKNOWN
            },
        ),
    )


def from_global_position_int(msg: Any, ns: int) -> Iterable[DerivedMessage]:
    return _location_fix(msg, ns, suffix="location", frame_id="wgs84", alt_field="alt")


def from_gps_raw_int(msg: Any, ns: int) -> Iterable[DerivedMessage]:
    return _location_fix(msg, ns, suffix="gps_location", frame_id="wgs84", alt_field="alt")


def from_attitude(msg: Any, ns: int) -> Iterable[DerivedMessage]:
    roll, pitch, yaw = _finite(msg.roll), _finite(msg.pitch), _finite(msg.yaw)
    if roll is None or pitch is None or yaw is None:
        return ()
    return (
        DerivedMessage(
            topic_suffix="attitude_transform",
            schema_name="foxglove.FrameTransform",
            schema=FRAME_TRANSFORM_SCHEMA,
            payload={
                "timestamp": _timestamp(ns),
                "parent_frame_id": "map",
                "child_frame_id": "base_link",
                "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation": euler_to_quaternion(roll, pitch, yaw),
            },
        ),
    )


def from_statustext(msg: Any, ns: int) -> Iterable[DerivedMessage]:
    text = getattr(msg, "text", "")
    if isinstance(text, bytes):
        text = text.decode("utf-8", errors="replace")
    return (
        DerivedMessage(
            topic_suffix="log",
            schema_name="foxglove.Log",
            schema=LOG_SCHEMA,
            payload={
                "timestamp": _timestamp(ns),
                "level": _SEVERITY_TO_LOG_LEVEL.get(getattr(msg, "severity", 6), 0),
                "message": text.split("\x00", 1)[0],
                "name": "mavlink",
                "file": "",
                "line": 0,
            },
        ),
    )


#: MAVLink message name -> converter. Adding an entry is all it takes to add a
#: new derived topic.
CONVERTERS: dict[str, Callable[[Any, int], Iterable[DerivedMessage]]] = {
    "GLOBAL_POSITION_INT": from_global_position_int,
    "GPS_RAW_INT": from_gps_raw_int,
    "ATTITUDE": from_attitude,
    "STATUSTEXT": from_statustext,
}


def convert(message_name: str, msg: Any, receive_time_ns: int) -> list[DerivedMessage]:
    """Apply the converter registered for ``message_name``, if any.

    Conversion failures are swallowed: a malformed or unexpected message must
    degrade the derived topic only, never the generic channel.
    """
    converter = CONVERTERS.get(message_name)
    if converter is None:
        return []
    try:
        return list(converter(msg, receive_time_ns))
    except Exception:  # noqa: BLE001 - derived topics are strictly best-effort
        return []
