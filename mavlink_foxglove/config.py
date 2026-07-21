"""Runtime configuration, resolved from CLI arguments and environment variables.

Every option has an ``MAVLINK_FOXGLOVE_``-prefixed environment variable so the
container can be configured without overriding the image entrypoint.
"""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass

DEFAULT_MAVLINK_URL = "udpin:0.0.0.0:14445"
DEFAULT_WS_HOST = "0.0.0.0"
DEFAULT_WS_PORT = 8765
#: PX4 speaks stock common.xml; ArduPilot users should pass --dialect ardupilotmega.
DEFAULT_DIALECT = "common"
DEFAULT_WIRE_VERSION = 2

#: Convenience presets mapping an autopilot to its dialect. `ardupilotmega`
#: is a strict superset of `common`, so it also decodes PX4 traffic -- useful
#: when one bridge must serve a mixed fleet.
AUTOPILOT_DIALECTS = {
    "px4": "common",
    "ardupilot": "ardupilotmega",
    "both": "ardupilotmega",
    "all": "all",
}
DEFAULT_AUTOPILOT = "px4"
DEFAULT_TOPIC_TEMPLATE = "/mavlink/{system_id}/{component_id}/{message}"
DEFAULT_QUEUE_SIZE = 10_000

_ENV_PREFIX = "MAVLINK_FOXGLOVE_"


@dataclass(frozen=True)
class Config:
    """Fully resolved bridge configuration."""

    mavlink_url: str = DEFAULT_MAVLINK_URL
    dialect: str = DEFAULT_DIALECT
    #: MAVLink wire protocol major version (1 or 2).
    wire_version: int = DEFAULT_WIRE_VERSION
    ws_host: str = DEFAULT_WS_HOST
    ws_port: int = DEFAULT_WS_PORT
    topic_template: str = DEFAULT_TOPIC_TEMPLATE
    queue_size: int = DEFAULT_QUEUE_SIZE
    #: Emit ``<field>_enum`` string companions for enum-typed fields.
    enum_names: bool = True
    #: Publish Foxglove well-known schemas (LocationFix/FrameTransform/Log).
    derived_topics: bool = True
    #: Advertise every message in the dialect at startup instead of on first sight.
    advertise_all: bool = False
    #: Send a MAVLink heartbeat so the autopilot keeps streaming to us.
    send_heartbeat: bool = True
    log_level: str = "INFO"


def _env(name: str) -> str | None:
    return os.environ.get(_ENV_PREFIX + name)


def _env_bool(name: str, default: bool) -> bool:
    raw = _env(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


def _env_int(name: str, default: int) -> int:
    raw = _env(name)
    return default if raw is None else int(raw)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mavlink-foxglove",
        description="Expose every MAVLink message as a Foxglove WebSocket channel.",
    )
    parser.add_argument(
        "--mavlink-url",
        default=_env("MAVLINK_URL") or DEFAULT_MAVLINK_URL,
        help="pymavlink connection string, e.g. udpin:0.0.0.0:14445, "
        "udpout:host:14550, tcp:host:5760, /dev/ttyACM0 (default: %(default)s)",
    )
    parser.add_argument(
        "--autopilot",
        choices=sorted(AUTOPILOT_DIALECTS),
        default=_env("AUTOPILOT") or DEFAULT_AUTOPILOT,
        help="Autopilot preset selecting a sensible dialect; overridden by an "
        "explicit --dialect (default: %(default)s)",
    )
    parser.add_argument(
        "--dialect",
        default=_env("DIALECT"),
        help="MAVLink dialect / message definition set, overriding --autopilot. "
        "'common' suits PX4, 'ardupilotmega' ArduPilot, 'development' adds "
        "work-in-progress messages (default: chosen by --autopilot)",
    )
    parser.add_argument(
        "--wire-version",
        type=int,
        choices=(1, 2),
        default=_env_int("WIRE_VERSION", DEFAULT_WIRE_VERSION),
        help="MAVLink wire protocol major version; use 1 for legacy vehicles "
        "(default: %(default)s)",
    )
    parser.add_argument(
        "--ws-host",
        default=_env("WS_HOST") or DEFAULT_WS_HOST,
        help="Foxglove WebSocket bind address (default: %(default)s)",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=_env_int("WS_PORT", DEFAULT_WS_PORT),
        help="Foxglove WebSocket port (default: %(default)s)",
    )
    parser.add_argument(
        "--topic-template",
        default=_env("TOPIC_TEMPLATE") or DEFAULT_TOPIC_TEMPLATE,
        help="Topic naming template; supports {system_id}, {component_id}, "
        "{message} (default: %(default)s)",
    )
    parser.add_argument(
        "--queue-size",
        type=int,
        default=_env_int("QUEUE_SIZE", DEFAULT_QUEUE_SIZE),
        help="Max buffered messages before the oldest are dropped "
        "(default: %(default)s)",
    )
    parser.add_argument(
        "--log-level",
        default=_env("LOG_LEVEL") or "INFO",
        help="Python logging level (default: %(default)s)",
    )

    _add_bool_flag(
        parser, "enum-names", _env_bool("ENUM_NAMES", True),
        "add <field>_enum string companions for enum-typed fields",
    )
    _add_bool_flag(
        parser, "derived-topics", _env_bool("DERIVED_TOPICS", True),
        "publish Foxglove well-known schemas for map/3D/log panels",
    )
    _add_bool_flag(
        parser, "advertise-all", _env_bool("ADVERTISE_ALL", False),
        "advertise every dialect message at startup rather than on first sight",
    )
    _add_bool_flag(
        parser, "send-heartbeat", _env_bool("SEND_HEARTBEAT", True),
        "send a 1 Hz GCS heartbeat back to the vehicle",
    )
    return parser


def _add_bool_flag(
    parser: argparse.ArgumentParser, name: str, default: bool, help_text: str
) -> None:
    """Register a matched ``--flag`` / ``--no-flag`` pair."""
    dest = name.replace("-", "_")
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        f"--{name}", dest=dest, action="store_true", default=default,
        help=f"{help_text} (default: {default})",
    )
    group.add_argument(f"--no-{name}", dest=dest, action="store_false", help=argparse.SUPPRESS)


def load_config(argv: list[str] | None = None) -> Config:
    """Resolve configuration from ``argv`` (falling back to environment defaults)."""
    values = vars(build_parser().parse_args(argv))
    # --dialect is the precise knob; --autopilot is the friendly preset behind it.
    autopilot = values.pop("autopilot")
    if not values.get("dialect"):
        values["dialect"] = AUTOPILOT_DIALECTS[autopilot]
    return Config(**values)
