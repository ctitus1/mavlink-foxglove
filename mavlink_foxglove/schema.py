"""Generate JSON Schemas from pymavlink message class metadata.

Each MAVLink message class carries enough reflection data (`fieldnames`,
`fieldtypes`, `array_lengths`, `fieldunits_by_name`, `fieldenums_by_name`) to
derive a JSON Schema mechanically, so the bridge supports every message in a
dialect -- including ones added after this code was written -- without a
hand-maintained table.

Important pymavlink trap: the two metadata lists use *different* orderings.

* ``fieldtypes`` is indexed in ``fieldnames`` order (declaration order).
* ``array_lengths`` is indexed in ``ordered_fieldnames`` order (wire order,
  which sorts fields by descending size).

These coincide for many messages, which is what makes the bug so easy to ship:
PARAM_REQUEST_READ, for example, has ``fieldnames`` index 2 = ``param_id``
(char[16]) but ``array_lengths`` puts that 16 at index 3. Reading both lists
with one index silently turns a string field into a scalar and vice versa.
:func:`field_specs` resolves each list against its own ordering.
"""

from __future__ import annotations

from types import ModuleType
from typing import Any

from .dialect import message_name

#: MAVLink scalar type -> JSON Schema type.
_INT_TYPES = frozenset(
    {
        "uint8_t", "int8_t", "uint8_t_mavlink_version",
        "uint16_t", "int16_t",
        "uint32_t", "int32_t",
        "uint64_t", "int64_t",
    }
)
_FLOAT_TYPES = frozenset({"float", "double"})

#: Beyond 2^53 a JSON number cannot round-trip through a IEEE-754 double, which
#: is what every JSON parser in the Foxglove stack uses.
_LOSSY_TYPES = frozenset({"uint64_t", "int64_t"})

#: Envelope key holding per-message routing metadata. MAVLink field names are
#: always lowercase alphanumerics, so a leading underscore cannot collide.
META_KEY = "_meta"

_META_SCHEMA: dict[str, Any] = {
    "type": "object",
    "description": "Bridge-added envelope: MAVLink routing and receipt metadata.",
    "properties": {
        "system_id": {"type": "integer", "description": "Source MAVLink system ID."},
        "component_id": {"type": "integer", "description": "Source MAVLink component ID."},
        "sequence": {"type": "integer", "description": "Sender's packet sequence number."},
        "message_id": {"type": "integer", "description": "MAVLink message ID."},
        "receive_timestamp": {
            "type": "object",
            "description": "Host time the packet was parsed.",
            "properties": {
                "sec": {"type": "integer"},
                "nsec": {"type": "integer"},
            },
        },
    },
}


def _scalar_schema(mav_type: str) -> dict[str, Any]:
    """JSON Schema node for a single (non-array) MAVLink value."""
    if mav_type in _FLOAT_TYPES:
        # NaN/Infinity are not representable in JSON, so encoding.py maps them
        # to null. The schema must permit that or strict validators reject it.
        return {"type": ["number", "null"]}
    if mav_type in _INT_TYPES:
        node: dict[str, Any] = {"type": "integer"}
        if mav_type in _LOSSY_TYPES:
            node["description"] = (
                "64-bit integer; values above 2^53 lose precision when decoded "
                "as a JSON number."
            )
        return node
    # 'char' and anything a future dialect introduces degrade to a string.
    return {"type": "string"}


def _annotate(node: dict[str, Any], extra: str) -> None:
    """Append ``extra`` to a schema node's description."""
    existing = node.get("description")
    node["description"] = f"{existing} {extra}" if existing else extra


def field_schema(
    mav_type: str,
    array_length: int,
    unit: str | None = None,
    enum: str | None = None,
) -> dict[str, Any]:
    """Build the JSON Schema node for one MAVLink field."""
    if array_length and mav_type == "char":
        # char[N] is a fixed-width NUL-padded string, not an array of numbers.
        node = {"type": "string", "maxLength": array_length}
    elif array_length:
        node = {
            "type": "array",
            "items": _scalar_schema(mav_type),
            "maxItems": array_length,
        }
    else:
        node = _scalar_schema(mav_type)

    notes = []
    if unit:
        notes.append(f"Units: {unit}.")
    if enum:
        notes.append(f"Enum: {enum}.")
    if notes:
        _annotate(node, " ".join(notes))
    return node


def field_specs(msg_class: type) -> list[tuple[str, str, int]]:
    """Return ``(name, mav_type, array_length)`` for each field of ``msg_class``.

    This is the single place that untangles pymavlink's two field orderings
    (see the module docstring); everything else should consume this.
    """
    array_lengths = getattr(msg_class, "array_lengths", []) or []
    ordered = getattr(msg_class, "ordered_fieldnames", msg_class.fieldnames)
    # array_lengths follows wire order, so resolve it to names first.
    lengths_by_name = {
        name: array_lengths[index]
        for index, name in enumerate(ordered)
        if index < len(array_lengths)
    }
    return [
        (name, msg_class.fieldtypes[index], lengths_by_name.get(name, 0))
        for index, name in enumerate(msg_class.fieldnames)
    ]


def message_schema(msg_class: type, enum_names: bool = True) -> dict[str, Any]:
    """Derive a JSON Schema for a pymavlink message class.

    When ``enum_names`` is set, each enum-typed field gains a ``<field>_enum``
    string companion carrying the symbolic name, which makes Foxglove's table
    and state-transition panels readable.
    """
    units = getattr(msg_class, "fieldunits_by_name", {}) or {}
    enums = getattr(msg_class, "fieldenums_by_name", {}) or {}

    properties: dict[str, Any] = {META_KEY: _META_SCHEMA}
    for name, mav_type, length in field_specs(msg_class):
        enum = enums.get(name)
        properties[name] = field_schema(mav_type, length, units.get(name), enum)

        if enum_names and enum:
            properties[f"{name}_enum"] = {
                "type": ["string", "null"],
                "description": f"Symbolic name of {name} from enum {enum}.",
            }

    return {
        "type": "object",
        "title": message_name(msg_class),
        "description": (getattr(msg_class, "description", "") or "").strip() or None,
        "properties": properties,
    }


def build_schemas(
    dialect: ModuleType, enum_names: bool = True
) -> dict[str, dict[str, Any]]:
    """Pre-compute schemas for every message in ``dialect``, keyed by message name."""
    from .dialect import message_classes

    return {
        message_name(cls): message_schema(cls, enum_names)
        for cls in message_classes(dialect)
    }
