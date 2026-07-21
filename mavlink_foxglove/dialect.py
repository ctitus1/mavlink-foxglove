"""Resolution of MAVLink message definitions (the "dialect").

This module is the *single* place the bridge learns what messages exist and
what fields they have. Everything downstream -- schema generation, encoding,
derived topics -- is driven purely by reflection over whatever this returns, so
pointing the bridge at a different definition set changes nothing else.

That matters for version pinning. Two axes are supported today:

* ``dialect`` -- which XML definition set (``common`` for PX4, ``ardupilotmega``
  for ArduPilot, ``development`` for work-in-progress messages, ...).
* ``wire_version`` -- MAVLink 1 vs MAVLink 2 framing, which also selects the
  ``v10``/``v20`` generated module tree.

A third axis, pinning to an *older revision* of a dialect's definitions, is not
wired to a flag yet; the hook for it is :func:`load_dialect`, which need only
return a module exposing ``mavlink_map`` and ``enums``. Generating such a module
from an archived XML with ``pymavlink.generator.mavgen`` and importing it here
is enough -- no other module needs to change.
"""

from __future__ import annotations

import importlib
import os
from types import ModuleType
from typing import Any

#: PX4 speaks stock common.xml, so it is the default rather than a vendor set.
DEFAULT_DIALECT = "common"
DEFAULT_WIRE_VERSION = 2

_SUPPORTED_WIRE_VERSIONS = {1: "v10", 2: "v20"}


def load_dialect(name: str, wire_version: int = DEFAULT_WIRE_VERSION) -> ModuleType:
    """Import a generated MAVLink dialect module.

    Args:
        name: Dialect name, e.g. ``"common"`` or ``"ardupilotmega"``.
        wire_version: MAVLink wire protocol major version (1 or 2).
    """
    package = _SUPPORTED_WIRE_VERSIONS.get(wire_version)
    if package is None:
        raise ValueError(
            f"Unsupported MAVLink wire version {wire_version!r}; "
            f"expected one of {sorted(_SUPPORTED_WIRE_VERSIONS)}"
        )
    try:
        return importlib.import_module(f"pymavlink.dialects.{package}.{name}")
    except ImportError as exc:
        raise ValueError(
            f"Unknown MAVLink dialect {name!r} for wire version {wire_version}"
        ) from exc


def configure_mavutil(name: str, wire_version: int = DEFAULT_WIRE_VERSION) -> None:
    """Point ``pymavlink.mavutil`` at the same dialect the bridge decoded against.

    ``mavutil`` selects its wire protocol from the ``MAVLINK20`` environment
    variable at ``set_dialect`` time, so the variable must be set first. Without
    this, connections would decode against pymavlink's default dialect while the
    schemas describe ours.
    """
    if wire_version == 2:
        os.environ["MAVLINK20"] = "1"
    else:
        os.environ.pop("MAVLINK20", None)

    from pymavlink import mavutil

    mavutil.set_dialect(name)


def message_classes(dialect: ModuleType) -> list[type]:
    """Every ``MAVLink_*_message`` class defined by ``dialect``, ordered by msgid."""
    return [dialect.mavlink_map[msgid] for msgid in sorted(dialect.mavlink_map)]


def message_name(msg: Any) -> str:
    """Return a message's MAVLink name across pymavlink versions.

    ``.name`` is deprecated in pymavlink >= 2.4.31 in favour of ``.msgname``,
    but older releases only have ``.name``.
    """
    name = getattr(msg, "msgname", None)
    return name if name is not None else msg.name
