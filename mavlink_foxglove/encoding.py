"""Convert pymavlink message objects into JSON-safe dictionaries.

The tricky parts of this conversion, and why they matter:

* **NaN / Infinity** appear routinely in MAVLink floats (unknown or unset
  values) but have no JSON representation. ``json.dumps`` emits bare ``NaN``
  tokens by default, which is invalid JSON and rejected by strict parsers, so
  they are mapped to ``null``.
* **char[N] fields** arrive as ``str`` on modern pymavlink and ``bytes`` on
  older releases, in both cases NUL-padded.
* **bytes payloads** are not always valid UTF-8 (e.g. tunnelled binary), so
  decoding must never raise.
"""

from __future__ import annotations

import math
from types import ModuleType
from typing import Any

from .dialect import message_name
from .schema import META_KEY


def _clean_float(value: float) -> float | None:
    """Map non-finite floats to ``None`` so the result is valid JSON."""
    return value if math.isfinite(value) else None


def _clean_text(value: bytes | str) -> str:
    """Decode a MAVLink char[N] field to a trimmed string, never raising."""
    if isinstance(value, bytes):
        value = value.decode("utf-8", errors="replace")
    # char[N] fields are NUL-padded to their full width.
    return value.split("\x00", 1)[0]


def sanitize(value: Any) -> Any:
    """Recursively convert a pymavlink field value into a JSON-safe value."""
    if isinstance(value, float):
        return _clean_float(value)
    if isinstance(value, (bytes, bytearray)):
        return _clean_text(bytes(value))
    if isinstance(value, str):
        return _clean_text(value)
    if isinstance(value, (list, tuple)):
        return [sanitize(item) for item in value]
    # bool is a subclass of int and both are already JSON-safe.
    if isinstance(value, (int, bool)) or value is None:
        return value
    return str(value)


def enum_label(dialect: ModuleType, enum_name: str, value: Any) -> str | None:
    """Resolve a numeric enum value to its symbolic MAVLink name.

    Returns ``None`` for unknown values rather than raising, so a vehicle
    running a newer dialect than the bridge cannot break encoding.
    """
    entries = dialect.enums.get(enum_name)
    if not entries or not isinstance(value, int):
        return None
    entry = entries.get(value)
    return getattr(entry, "name", None) if entry is not None else None


def encode_message(
    msg: Any,
    dialect: ModuleType,
    receive_time_ns: int,
    enum_names: bool = True,
) -> dict[str, Any]:
    """Render a pymavlink message as a JSON-safe dict matching its schema.

    The MAVLink fields sit at the top level (so Foxglove plot paths read as
    ``/topic.roll``) with routing metadata nested under ``_meta``.
    """
    msg_class = type(msg)
    enums = getattr(msg_class, "fieldenums_by_name", {}) or {}

    out: dict[str, Any] = {
        META_KEY: {
            "system_id": msg.get_srcSystem(),
            "component_id": msg.get_srcComponent(),
            "sequence": msg.get_seq(),
            "message_id": msg.get_msgId(),
            "receive_timestamp": {
                "sec": receive_time_ns // 1_000_000_000,
                "nsec": receive_time_ns % 1_000_000_000,
            },
        }
    }

    for name in msg_class.fieldnames:
        raw = getattr(msg, name, None)
        out[name] = sanitize(raw)
        if enum_names and name in enums:
            out[f"{name}_enum"] = enum_label(dialect, enums[name], raw)

    return out


def topic_for(msg: Any, template: str) -> str:
    """Render the configured topic template for a received message."""
    return template.format(
        system_id=msg.get_srcSystem(),
        component_id=msg.get_srcComponent(),
        message=message_name(msg),
    )
