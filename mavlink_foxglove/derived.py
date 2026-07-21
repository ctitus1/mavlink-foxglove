"""Derived channels using Foxglove's well-known schemas.

The generic MAVLink channels are complete but untyped as far as Foxglove is
concerned, so they only drive the Raw Message, Table and Plot panels. Foxglove
recognises a small set of schema *names* and wires them into richer panels --
Map, 3D and Log. This module republishes a handful of MAVLink messages under
those names so those panels work out of the box.

Two conventions matter here, and getting either wrong produces a picture that
looks plausible but is wrong:

**Coordinate frame.** MAVLink is NED (North-East-Down) with an FRD body frame,
the aerospace convention. Foxglove's 3D panel is Z-up, matching ROS REP-103 ENU
(East-North-Up) with an FLU body frame. Publishing NED directly puts the vehicle
underground and inverts its attitude, so everything here is converted to ENU.
The raw NED values remain untouched on the generic `LOCAL_POSITION_NED` channel.

**Pose is split across two messages.** Position arrives in LOCAL_POSITION_NED
and orientation in ATTITUDE, at independent rates. A transform needs both, so
:class:`DerivedPublisher` keeps the last-known half per vehicle and republishes
a fused ``map`` -> ``base_link`` transform whenever either updates. Publishing
two competing transforms for the same frame pair would make the vehicle jump
between the origin and its true position.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

#: MAV_SEVERITY (0 = emergency .. 7 = debug) -> foxglove.LogLevel
#: (1 = debug, 2 = info, 3 = warning, 4 = error, 5 = fatal).
_SEVERITY_TO_LOG_LEVEL = {0: 5, 1: 5, 2: 4, 3: 4, 4: 3, 5: 2, 6: 2, 7: 1}

#: The world frame every derived transform is expressed in. It is the vehicle's
#: EKF/local origin -- which for most flights is where it was armed -- not a
#: geographic datum.
WORLD_FRAME = "map"
BODY_FRAME = "base_link"

_IDENTITY_ROTATION = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
_ZERO_TRANSLATION = {"x": 0.0, "y": 0.0, "z": 0.0}

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


# -- coordinate conversion ------------------------------------------------


def ned_to_enu(north: float, east: float, down: float) -> dict[str, float]:
    """Convert a NED position to ENU, which is what Foxglove's 3D panel expects."""
    return {"x": east, "y": north, "z": -down}


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> dict[str, float]:
    """Convert ZYX intrinsic Euler angles (radians) to a quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def attitude_ned_to_enu_quaternion(
    roll: float, pitch: float, yaw: float
) -> dict[str, float]:
    """Convert MAVLink NED/FRD attitude to an ENU/FLU quaternion.

    The standard aerospace-to-robotics conversion: roll is unchanged, pitch
    negates, and yaw is measured counter-clockwise from East rather than
    clockwise from North. A vehicle heading North (yaw=0) therefore points along
    +Y in ENU, which is what the 3D panel draws as North.
    """
    return euler_to_quaternion(roll, -pitch, math.pi / 2 - yaw)


# -- stateless converters -------------------------------------------------


def _location_fix(
    msg: Any,
    ns: int,
    *,
    suffix: str,
    alt_field: str,
    lat_field: str = "lat",
    lon_field: str = "lon",
) -> Iterable[DerivedMessage]:
    """Shared builder for the lat/lon/alt-in-integer-units MAVLink messages.

    Field names differ between messages (``lat``/``lon`` on GPS_RAW_INT,
    ``latitude``/``longitude`` on HOME_POSITION), so callers name them
    explicitly. All of them use degE7 and millimetres.
    """
    lat = _finite(getattr(msg, lat_field, None))
    lon = _finite(getattr(msg, lon_field, None))
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
                "frame_id": "wgs84",
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
    return _location_fix(msg, ns, suffix="location", alt_field="alt")


def from_gps_raw_int(msg: Any, ns: int) -> Iterable[DerivedMessage]:
    return _location_fix(msg, ns, suffix="gps_location", alt_field="alt")


def from_home_position(msg: Any, ns: int) -> Iterable[DerivedMessage]:
    """Home / launch point, so the Map panel can show it alongside the vehicle."""
    return _location_fix(
        msg, ns, suffix="home_location", alt_field="altitude",
        lat_field="latitude", lon_field="longitude",
    )


def from_gps_global_origin(msg: Any, ns: int) -> Iterable[DerivedMessage]:
    """The EKF origin that all local NED positions are measured from."""
    return _location_fix(
        msg, ns, suffix="ekf_origin_location", alt_field="altitude",
        lat_field="latitude", lon_field="longitude",
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


#: Stateless MAVLink message name -> converter. Adding an entry is all it takes
#: to add a new derived topic.
CONVERTERS: dict[str, Callable[[Any, int], Iterable[DerivedMessage]]] = {
    "GLOBAL_POSITION_INT": from_global_position_int,
    "GPS_RAW_INT": from_gps_raw_int,
    "HOME_POSITION": from_home_position,
    "GPS_GLOBAL_ORIGIN": from_gps_global_origin,
    "STATUSTEXT": from_statustext,
}

#: Messages that contribute one half of the fused vehicle pose.
POSE_MESSAGES = frozenset({"ATTITUDE", "LOCAL_POSITION_NED"})


@dataclass
class _Pose:
    """Last-known pose halves for one vehicle, in Foxglove's ENU frame."""

    translation: dict[str, float] = field(
        default_factory=lambda: dict(_ZERO_TRANSLATION)
    )
    rotation: dict[str, float] = field(
        default_factory=lambda: dict(_IDENTITY_ROTATION)
    )


class DerivedPublisher:
    """Converts MAVLink messages into Foxglove well-known schema publications.

    Holds the small amount of per-vehicle state needed to fuse position and
    orientation, which MAVLink sends in separate messages, into a single
    transform. Stateless conversions are delegated to :data:`CONVERTERS`.
    """

    def __init__(self) -> None:
        self._poses: dict[tuple[int, int], _Pose] = {}

    def convert(self, message_name: str, msg: Any, receive_time_ns: int) -> list[DerivedMessage]:
        """Derive whatever Foxglove-native messages ``msg`` supports.

        Conversion failures are swallowed: a malformed or unexpected message
        must degrade the derived topic only, never the generic channel.
        """
        try:
            if message_name in POSE_MESSAGES:
                return self._update_pose(message_name, msg, receive_time_ns)
            converter = CONVERTERS.get(message_name)
            return list(converter(msg, receive_time_ns)) if converter else []
        except Exception:  # noqa: BLE001 - derived topics are strictly best-effort
            return []

    def _update_pose(
        self, message_name: str, msg: Any, ns: int
    ) -> list[DerivedMessage]:
        """Fold one pose half into the vehicle's pose and emit the fused result."""
        key = (msg.get_srcSystem(), msg.get_srcComponent())
        pose = self._poses.setdefault(key, _Pose())

        if message_name == "ATTITUDE":
            roll, pitch, yaw = _finite(msg.roll), _finite(msg.pitch), _finite(msg.yaw)
            if roll is None or pitch is None or yaw is None:
                return []
            pose.rotation = attitude_ned_to_enu_quaternion(roll, pitch, yaw)
        else:  # LOCAL_POSITION_NED
            north, east, down = _finite(msg.x), _finite(msg.y), _finite(msg.z)
            if north is None or east is None or down is None:
                return []
            pose.translation = ned_to_enu(north, east, down)

        return [
            DerivedMessage(
                topic_suffix="pose",
                schema_name="foxglove.FrameTransform",
                schema=FRAME_TRANSFORM_SCHEMA,
                payload={
                    "timestamp": _timestamp(ns),
                    "parent_frame_id": WORLD_FRAME,
                    "child_frame_id": BODY_FRAME,
                    "translation": dict(pose.translation),
                    "rotation": dict(pose.rotation),
                },
            )
        ]
