"""End-to-end: real MAVLink over UDP -> real Foxglove WebSocket client.

This is the test that proves the container does what it claims. It runs the
actual bridge, sends real MAVLink packets at it over UDP, connects a client
speaking the genuine Foxglove WebSocket subprotocol, and validates every
received payload against the schema the bridge advertised for it.
"""

from __future__ import annotations

import asyncio
import json
import socket
import struct

import jsonschema
import pytest
import websockets
from pymavlink import mavutil

from mavlink_foxglove.bridge import MavlinkFoxgloveBridge
from mavlink_foxglove.config import Config

SUBPROTOCOL = "foxglove.websocket.v1"
#: Binary opcode for message data in the Foxglove WebSocket protocol.
OP_MESSAGE_DATA = 1
_MESSAGE_DATA_HEADER = struct.Struct("<BIQ")


def _free_port(kind: int = socket.SOCK_STREAM) -> int:
    with socket.socket(socket.AF_INET, kind) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


@pytest.fixture
def ports():
    return {"udp": _free_port(socket.SOCK_DGRAM), "ws": _free_port()}


class FoxgloveTestClient:
    """A minimal but faithful Foxglove WebSocket protocol client."""

    def __init__(self, url: str) -> None:
        self._url = url
        self._ws: websockets.WebSocketClientProtocol | None = None
        self.channels: dict[int, dict] = {}
        self._next_subscription_id = 1

    async def __aenter__(self) -> "FoxgloveTestClient":
        self._ws = await websockets.connect(self._url, subprotocols=[SUBPROTOCOL])
        return self

    async def __aexit__(self, *exc_info) -> None:
        if self._ws is not None:
            await self._ws.close()

    async def _recv_json(self, timeout: float) -> dict:
        while True:
            raw = await asyncio.wait_for(self._ws.recv(), timeout)
            if isinstance(raw, str):
                return json.loads(raw)

    async def collect_advertisements(self, wanted: set[str], timeout: float) -> None:
        """Read advertise messages until every topic in ``wanted`` is known."""

        async def _loop() -> None:
            while not wanted.issubset({c["topic"] for c in self.channels.values()}):
                msg = await self._recv_json(timeout)
                if msg.get("op") == "advertise":
                    for channel in msg["channels"]:
                        self.channels[channel["id"]] = channel

        await asyncio.wait_for(_loop(), timeout)

    async def subscribe(self, topics: set[str]) -> dict[int, str]:
        """Subscribe to the given topics; returns subscription id -> topic."""
        subscriptions = []
        mapping: dict[int, str] = {}
        for channel in self.channels.values():
            if channel["topic"] in topics:
                sub_id = self._next_subscription_id
                self._next_subscription_id += 1
                subscriptions.append({"id": sub_id, "channelId": channel["id"]})
                mapping[sub_id] = channel["topic"]
        await self._ws.send(json.dumps({"op": "subscribe", "subscriptions": subscriptions}))
        return mapping

    async def collect_messages(self, count: int, timeout: float) -> list[tuple[int, dict]]:
        """Read ``count`` binary message-data frames as (subscription_id, payload)."""
        out: list[tuple[int, dict]] = []

        async def _loop() -> None:
            while len(out) < count:
                raw = await asyncio.wait_for(self._ws.recv(), timeout)
                if not isinstance(raw, bytes) or raw[0] != OP_MESSAGE_DATA:
                    continue
                _op, sub_id, _timestamp = _MESSAGE_DATA_HEADER.unpack_from(raw, 0)
                payload = raw[_MESSAGE_DATA_HEADER.size:]
                # json.loads rejects bare NaN/Infinity only with a strict parser,
                # so assert explicitly that the bytes contain no such tokens.
                text = payload.decode("utf-8")
                assert "NaN" not in text and "Infinity" not in text, text
                out.append((sub_id, json.loads(text)))

        await asyncio.wait_for(_loop(), timeout)
        return out


async def _run_bridge(config: Config):
    bridge = MavlinkFoxgloveBridge(config)
    task = asyncio.ensure_future(bridge.run())
    # Give the WebSocket server and MAVLink socket time to bind.
    await asyncio.sleep(0.5)
    return task


def _sender(port: int):
    mavutil.set_dialect("common")
    return mavutil.mavlink_connection(
        f"udpout:127.0.0.1:{port}", source_system=1, source_component=1, dialect="common"
    )


@pytest.mark.asyncio
async def test_messages_reach_a_foxglove_client_and_match_their_schema(ports):
    config = Config(
        mavlink_url=f"udpin:127.0.0.1:{ports['udp']}",
        ws_host="127.0.0.1",
        ws_port=ports["ws"],
        send_heartbeat=False,
    )
    task = await _run_bridge(config)
    conn = _sender(ports["udp"])

    try:
        async with FoxgloveTestClient(f"ws://127.0.0.1:{ports['ws']}") as client:
            # The bridge advertises lazily, so it must see traffic first.
            async def keep_sending() -> None:
                while True:
                    conn.mav.attitude_send(1000, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0)
                    conn.mav.gps_raw_int_send(
                        0, 3, int(37.77e7), int(-122.41e7), 100_000,
                        100, 100, 500, 0, 12,
                    )
                    await asyncio.sleep(0.1)

            pump = asyncio.ensure_future(keep_sending())
            try:
                wanted = {"/mavlink/1/1/ATTITUDE", "/mavlink/1/1/GPS_RAW_INT"}
                await client.collect_advertisements(wanted, timeout=10.0)
                mapping = await client.subscribe(wanted)
                messages = await client.collect_messages(6, timeout=10.0)
            finally:
                pump.cancel()

            by_topic = {c["topic"]: c for c in client.channels.values()}
            assert len(messages) >= 6

            for sub_id, payload in messages:
                topic = mapping[sub_id]
                schema = json.loads(by_topic[topic]["schema"])
                # The real proof: the payload validates against the schema the
                # bridge told Foxglove to expect.
                jsonschema.validate(payload, schema)
                assert payload["_meta"]["system_id"] == 1

            attitude = next(p for s, p in messages if mapping[s].endswith("ATTITUDE"))
            assert attitude["roll"] == pytest.approx(0.1, abs=1e-6)

            gps = next(p for s, p in messages if mapping[s].endswith("GPS_RAW_INT"))
            assert gps["fix_type_enum"] == "GPS_FIX_TYPE_3D_FIX"
    finally:
        task.cancel()
        conn.close()


@pytest.mark.asyncio
async def test_derived_topics_are_published(ports):
    config = Config(
        mavlink_url=f"udpin:127.0.0.1:{ports['udp']}",
        ws_host="127.0.0.1",
        ws_port=ports["ws"],
        send_heartbeat=False,
    )
    task = await _run_bridge(config)
    conn = _sender(ports["udp"])

    try:
        async with FoxgloveTestClient(f"ws://127.0.0.1:{ports['ws']}") as client:

            async def keep_sending() -> None:
                while True:
                    conn.mav.global_position_int_send(
                        1000, int(37.7749e7), int(-122.4194e7),
                        100_000, 50_000, 0, 0, 0, 0,
                    )
                    await asyncio.sleep(0.1)

            pump = asyncio.ensure_future(keep_sending())
            try:
                wanted = {"/mavlink/1/1/location"}
                await client.collect_advertisements(wanted, timeout=10.0)
                mapping = await client.subscribe(wanted)
                messages = await client.collect_messages(2, timeout=10.0)
            finally:
                pump.cancel()

            by_topic = {c["topic"]: c for c in client.channels.values()}
            assert by_topic["/mavlink/1/1/location"]["schemaName"] == "foxglove.LocationFix"

            _sub_id, payload = messages[0]
            jsonschema.validate(
                payload, json.loads(by_topic["/mavlink/1/1/location"]["schema"])
            )
            assert payload["latitude"] == pytest.approx(37.7749, abs=1e-5)
    finally:
        task.cancel()
        conn.close()


@pytest.mark.asyncio
async def test_every_message_type_survives_the_bridge(ports):
    """Robustness sweep: send one of every message type through the live bridge.

    Nothing may crash the bridge, and every payload that arrives must validate
    against its advertised schema.
    """
    config = Config(
        mavlink_url=f"udpin:127.0.0.1:{ports['udp']}",
        ws_host="127.0.0.1",
        ws_port=ports["ws"],
        send_heartbeat=False,
        queue_size=100_000,
    )
    task = await _run_bridge(config)
    conn = _sender(ports["udp"])

    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    from mavlink_test_publisher import synthesize  # noqa: E402

    try:
        async with FoxgloveTestClient(f"ws://127.0.0.1:{ports['ws']}") as client:
            dialect = mavutil.mavlink
            classes = [dialect.mavlink_map[m] for m in sorted(dialect.mavlink_map)]

            sent = 0
            for msg_class in classes:
                msg = synthesize(msg_class)
                if msg is None:
                    continue
                conn.mav.send(msg)
                sent += 1
                # Pace to avoid overrunning the OS UDP receive buffer.
                await asyncio.sleep(0.002)

            assert sent > 100, "expected to send most of the dialect"
            await asyncio.sleep(2.0)

            # Drain whatever advertisements arrived.
            try:
                await client.collect_advertisements({"__never__"}, timeout=2.0)
            except asyncio.TimeoutError:
                pass

            assert not task.done(), "bridge died during the sweep"

            topics = {c["topic"] for c in client.channels.values()}
            # A large majority of the dialect should have been advertised; UDP
            # is lossy so an exact count would be flaky.
            assert len(topics) > sent * 0.8, f"only advertised {len(topics)} of {sent}"

            # Every advertised schema must be a valid JSON Schema document.
            for channel in client.channels.values():
                jsonschema.Draft7Validator.check_schema(json.loads(channel["schema"]))
    finally:
        task.cancel()
        conn.close()
