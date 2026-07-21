#!/usr/bin/env python3
"""Smoke-test a *running* bridge over its real network interfaces.

Unlike the pytest suite, which runs the bridge in-process, this drives a
deployed instance (typically the Docker container) exactly as a vehicle and
Foxglove would: MAVLink in over UDP, Foxglove WebSocket protocol out.

It asserts that
  1. traffic produces advertised channels,
  2. every advertised schema is a legal JSON Schema document,
  3. live payloads validate against the schema advertised for their channel,
  4. no NaN/Infinity tokens reach the wire.

Exits non-zero on failure, so it works as a CI gate.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import struct
import subprocess
import sys
import time
from pathlib import Path

import jsonschema
import websockets

SUBPROTOCOL = "foxglove.websocket.v1"
OP_MESSAGE_DATA = 1
_MESSAGE_DATA_HEADER = struct.Struct("<BIQ")

_PUBLISHER = Path(__file__).with_name("mavlink_test_publisher.py")


def _publish(mode: str, url: str, dialect: str, extra: list[str] | None = None):
    """Run the test publisher as a subprocess."""
    return subprocess.run(
        [sys.executable, str(_PUBLISHER), "--url", url, "--mode", mode,
         "--dialect", dialect, *(extra or [])],
        capture_output=True, text=True, check=False,
    )


async def _drain_advertisements(ws, timeout: float) -> dict[int, dict]:
    """Collect channel advertisements until the server goes quiet."""
    channels: dict[int, dict] = {}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            raw = await asyncio.wait_for(ws.recv(), 1.0)
        except asyncio.TimeoutError:
            break
        if isinstance(raw, str):
            message = json.loads(raw)
            if message.get("op") == "advertise":
                for channel in message["channels"]:
                    channels[channel["id"]] = channel
    return channels


async def run(args: argparse.Namespace) -> int:
    send_url = f"udpout:{args.mavlink_host}:{args.mavlink_port}"

    # Advertisement is lazy, so the bridge must see traffic before it has
    # anything to advertise.
    result = _publish("all", send_url, args.dialect)
    print(result.stdout.strip() or result.stderr.strip())
    await asyncio.sleep(args.settle)

    async with websockets.connect(args.ws_url, subprotocols=[SUBPROTOCOL]) as ws:
        channels = await _drain_advertisements(ws, timeout=args.settle + 5)
        print(f"Advertised channels: {len(channels)}")
        if len(channels) < args.min_channels:
            print(
                f"FAIL: expected at least {args.min_channels} channels, "
                f"got {len(channels)}",
                file=sys.stderr,
            )
            return 1

        for channel in channels.values():
            jsonschema.Draft7Validator.check_schema(json.loads(channel["schema"]))
        print(f"All {len(channels)} advertised schemas are valid JSON Schema.")

        derived = sorted(
            c["topic"] for c in channels.values() if c["schemaName"].startswith("foxglove.")
        )
        print(f"Foxglove well-known channels: {derived or '(none yet)'}")

        subscriptions, mapping = [], {}
        for index, channel in enumerate(channels.values(), start=1):
            subscriptions.append({"id": index, "channelId": channel["id"]})
            mapping[index] = channel
        await ws.send(json.dumps({"op": "subscribe", "subscriptions": subscriptions}))

        # Stream realistic telemetry in the background while we validate.
        loop = asyncio.get_event_loop()
        telemetry = loop.run_in_executor(
            None, _publish, "telemetry", send_url, args.dialect,
            ["--duration", str(args.duration)],
        )

        seen: set[str] = set()
        validated = 0
        deadline = time.monotonic() + args.duration + 8
        while time.monotonic() < deadline and validated < args.min_messages:
            try:
                raw = await asyncio.wait_for(ws.recv(), 2.0)
            except asyncio.TimeoutError:
                continue
            if not isinstance(raw, bytes) or raw[0] != OP_MESSAGE_DATA:
                continue
            _op, sub_id, _ts = _MESSAGE_DATA_HEADER.unpack_from(raw, 0)
            text = raw[_MESSAGE_DATA_HEADER.size:].decode("utf-8")
            if "NaN" in text or "Infinity" in text:
                print(f"FAIL: non-JSON float token on the wire: {text[:200]}", file=sys.stderr)
                return 1
            channel = mapping[sub_id]
            jsonschema.validate(json.loads(text), json.loads(channel["schema"]))
            seen.add(channel["topic"])
            validated += 1

        await telemetry

        print(f"Validated {validated} live messages across {len(seen)} topics.")
        if validated < args.min_messages:
            print(
                f"FAIL: expected at least {args.min_messages} validated messages, "
                f"got {validated}",
                file=sys.stderr,
            )
            return 1

    print("\nSMOKE TEST PASSED")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ws-url", default="ws://127.0.0.1:8765")
    parser.add_argument("--mavlink-host", default="127.0.0.1")
    parser.add_argument("--mavlink-port", type=int, default=14445)
    parser.add_argument("--dialect", default="common")
    parser.add_argument("--duration", type=float, default=6.0, help="telemetry seconds")
    parser.add_argument("--settle", type=float, default=3.0, help="seconds to wait after the sweep")
    parser.add_argument("--min-channels", type=int, default=100)
    parser.add_argument("--min-messages", type=int, default=30)
    args = parser.parse_args()
    return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
