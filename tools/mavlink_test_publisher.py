#!/usr/bin/env python3
"""Send synthetic MAVLink traffic at the bridge.

Two modes:

* ``--mode all`` (default) walks every message class in the dialect and sends
  one synthetic instance of each. This is the robustness check: it proves the
  bridge can advertise and encode every topic the dialect defines, not just the
  handful a particular vehicle happens to emit.
* ``--mode telemetry`` streams a realistic ATTITUDE / GLOBAL_POSITION_INT /
  GPS_RAW_INT / STATUSTEXT / SYS_STATUS loop, for eyeballing live plots and the
  Map and 3D panels in Foxglove.

Edge cases are deliberately included: NaN and infinite floats, non-UTF-8 bytes
in char fields, and 64-bit values above 2^53.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

# Must be set before pymavlink resolves its dialect module.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from mavlink_foxglove.schema import field_specs  # noqa: E402

#: Values used to fill synthetic messages, chosen to include awkward cases.
_SCALAR_FILL = {
    "uint8_t": 7,
    "uint8_t_mavlink_version": 3,
    "int8_t": -7,
    "uint16_t": 1234,
    "int16_t": -1234,
    "uint32_t": 123456,
    "int32_t": -123456,
    # Above 2^53: exercises the JSON precision path.
    "uint64_t": 9_007_199_254_740_993,
    "int64_t": -9_007_199_254_740_993,
    "float": 1.5,
    "double": 2.25,
    "char": "x",
}


def _fill_value(mav_type: str, array_length: int, index: int):
    """Produce a plausible value for one field of a synthetic message."""
    if array_length and mav_type == "char":
        # Generated constructors NUL-split char[N] as bytes, even though the
        # resulting attribute is a str.
        return b"test-string"[:array_length]
    if array_length:
        base = _SCALAR_FILL.get(mav_type, 0)
        return [base] * array_length
    if mav_type == "float":
        # Rotate through the non-finite values MAVLink uses for "unknown".
        return [1.5, float("nan"), float("inf"), -float("inf")][index % 4]
    return _SCALAR_FILL.get(mav_type, 0)


def synthesize(msg_class: type):
    """Build a synthetic instance of ``msg_class``, or ``None`` if unbuildable."""
    # field_specs untangles pymavlink's two conflicting metadata orderings.
    args = [
        _fill_value(mav_type, length, index)
        for index, (_name, mav_type, length) in enumerate(field_specs(msg_class))
    ]
    try:
        return msg_class(*args)
    except Exception as exc:  # noqa: BLE001 - report and skip, don't abort the sweep
        print(f"  ! cannot synthesize {msg_class.__name__}: {exc}", file=sys.stderr)
        return None


def send_all(conn, dialect, delay: float) -> tuple[int, int]:
    """Send one synthetic instance of every message in the dialect."""
    sent = skipped = 0
    for msgid in sorted(dialect.mavlink_map):
        msg_class = dialect.mavlink_map[msgid]
        msg = synthesize(msg_class)
        if msg is None:
            skipped += 1
            continue
        try:
            conn.mav.send(msg)
            sent += 1
        except Exception as exc:  # noqa: BLE001 - packing can reject odd values
            print(f"  ! cannot send {msg_class.__name__}: {exc}", file=sys.stderr)
            skipped += 1
        if delay:
            time.sleep(delay)
    return sent, skipped


#: Home / EKF origin for the synthetic flight, over San Francisco.
_HOME_LAT, _HOME_LON, _HOME_ALT_MM = 37.7749, -122.4194, 100_000


def send_telemetry(conn, duration: float, rate: float) -> int:
    """Stream a realistic telemetry loop for ``duration`` seconds.

    Emits everything the README's quick-start layout needs: a fused pose (from
    LOCAL_POSITION_NED + ATTITUDE), a global fix, home and EKF origin, and
    altitude.
    """
    sent = 0
    period = 1.0 / rate
    start = time.time()

    # Home and EKF origin are sent once up front, as a real vehicle does.
    conn.mav.home_position_send(
        int(_HOME_LAT * 1e7), int(_HOME_LON * 1e7), _HOME_ALT_MM,
        0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0, 0,
    )
    conn.mav.gps_global_origin_send(
        int(_HOME_LAT * 1e7), int(_HOME_LON * 1e7), _HOME_ALT_MM, 0
    )
    sent += 2

    while time.time() - start < duration:
        t = time.time() - start
        boot_ms = int(t * 1000)

        # A 20m circle at 15m altitude, expressed in NED relative to the EKF
        # origin -- this is what drives the 3D panel.
        north = 20.0 * math.cos(t * 0.5)
        east = 20.0 * math.sin(t * 0.5)
        down = -15.0
        conn.mav.local_position_ned_send(
            boot_ms, north, east, down,
            -10.0 * math.sin(t * 0.5), 10.0 * math.cos(t * 0.5), 0.0,
        )
        conn.mav.altitude_send(
            int(t * 1e6), 15.0, 115.0, 15.0, 15.0, 15.0, 15.0
        )
        sent += 2

        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_PX4,
            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        conn.mav.attitude_send(
            boot_ms,
            0.2 * math.sin(t),  # roll
            0.1 * math.cos(t),  # pitch
            # Yaw tracks the direction of travel around the circle.
            (t * 0.5 + math.pi / 2) % (2 * math.pi),
            0.0, 0.0, 0.0,
        )
        # The same circle in global coordinates, so the Map panel shows motion.
        # ~1e-5 degrees of latitude is about 1.1 m.
        conn.mav.global_position_int_send(
            boot_ms,
            int((_HOME_LAT + north * 9e-6) * 1e7),
            int((_HOME_LON + east * 1.1e-5) * 1e7),
            int(_HOME_ALT_MM + 15_000),  # alt (mm, AMSL)
            int(15_000),                 # relative_alt (mm)
            100, 0, 0,
            int(((t * 28.6) % 360) * 100),
        )
        conn.mav.gps_raw_int_send(
            int(t * 1e6),
            3,  # GPS_FIX_TYPE_3D_FIX -- exercises the enum companion field
            int((_HOME_LAT + north * 9e-6) * 1e7),
            int((_HOME_LON + east * 1.1e-5) * 1e7),
            int(_HOME_ALT_MM + 15_000),
            100, 100, 500, 0, 12,
        )
        conn.mav.sys_status_send(
            0, 0, 0, 250, 12000, 5000, 75, 0, 0, 0, 0, 0, 0,
        )
        conn.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            f"synthetic telemetry t={t:.1f}s".encode(),
        )
        sent += 6  # heartbeat, attitude, global position, gps, sys_status, statustext
        time.sleep(period)
    return sent


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url", default="udpout:127.0.0.1:14445",
        help="pymavlink connection string to send to (default: %(default)s)",
    )
    parser.add_argument("--dialect", default="common", help="MAVLink dialect (default: %(default)s)")
    parser.add_argument("--mode", choices=("all", "telemetry"), default="all")
    parser.add_argument("--duration", type=float, default=10.0, help="telemetry mode seconds")
    parser.add_argument("--rate", type=float, default=5.0, help="telemetry mode Hz")
    parser.add_argument(
        "--delay", type=float, default=0.002,
        help="seconds between messages in 'all' mode; keeps UDP buffers from overflowing",
    )
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=1)
    args = parser.parse_args()

    mavutil.set_dialect(args.dialect)
    dialect = mavutil.mavlink

    conn = mavutil.mavlink_connection(
        args.url, source_system=args.system_id, source_component=args.component_id,
        dialect=args.dialect,
    )
    print(f"Sending {args.mode} traffic to {args.url} (dialect={args.dialect})")

    if args.mode == "all":
        sent, skipped = send_all(conn, dialect, args.delay)
        print(f"Sent {sent} message types, skipped {skipped}")
    else:
        sent = send_telemetry(conn, args.duration, args.rate)
        print(f"Sent {sent} telemetry messages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
