"""MAVLink ingest: a blocking pymavlink reader bridged onto asyncio.

pymavlink's connection API is synchronous, so reading happens on a dedicated
daemon thread that hands messages to the event loop through a bounded queue.
The queue is bounded on purpose: a Foxglove client that stalls must not let the
bridge grow without limit, so the oldest messages are dropped and counted.
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from pymavlink import mavutil

from .dialect import message_name

logger = logging.getLogger(__name__)

#: Backoff between reconnect attempts when the link errors out.
_RECONNECT_DELAY_S = 1.0
#: How long a blocking read waits before looping to re-check the stop flag.
_RECV_TIMEOUT_S = 0.5
_HEARTBEAT_PERIOD_S = 1.0


@dataclass
class SourceStats:
    """Counters exposed for logging and Foxglove status messages."""

    received: int = 0
    dropped: int = 0
    bad_data: int = 0
    reconnects: int = 0
    messages_by_type: dict[str, int] = field(default_factory=dict)


@dataclass(frozen=True)
class Received:
    """A parsed MAVLink message paired with its host receive time."""

    message: Any
    receive_time_ns: int


class MavlinkSource:
    """Reads MAVLink from a pymavlink connection URL onto an asyncio queue."""

    def __init__(
        self,
        url: str,
        dialect_name: str,
        queue_size: int,
        send_heartbeat: bool = True,
    ) -> None:
        self._url = url
        self._dialect_name = dialect_name
        self._send_heartbeat = send_heartbeat
        self._queue: asyncio.Queue[Received] = asyncio.Queue(maxsize=queue_size)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._conn: Any = None
        self.stats = SourceStats()

    # -- lifecycle ---------------------------------------------------------

    def start(self, loop: asyncio.AbstractEventLoop) -> None:
        """Begin reading on a background thread, delivering onto ``loop``."""
        self._loop = loop
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="mavlink-reader", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Signal the reader thread to exit and wait briefly for it."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2 * _RECV_TIMEOUT_S + 1.0)
        self._close()

    async def get(self) -> Received:
        """Await the next received message."""
        return await self._queue.get()

    # -- reader thread -----------------------------------------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                if self._conn is None:
                    self._connect()
                self._read_loop()
            except Exception:  # noqa: BLE001 - a link fault must never kill the thread
                if self._stop.is_set():
                    break
                logger.exception("MAVLink link error; reconnecting in %.1fs", _RECONNECT_DELAY_S)
                self.stats.reconnects += 1
                self._close()
                self._stop.wait(_RECONNECT_DELAY_S)

    def _connect(self) -> None:
        logger.info("Connecting to MAVLink at %s", self._url)
        # dialect.configure_mavutil() has already pinned the wire version, so
        # mavutil resolves this name against the same generated module tree the
        # schemas were built from.
        self._conn = mavutil.mavlink_connection(
            self._url, dialect=self._dialect_name, autoreconnect=True
        )

    def _close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001 - best-effort cleanup
                logger.debug("Error closing MAVLink connection", exc_info=True)
            self._conn = None

    def _read_loop(self) -> None:
        next_heartbeat = 0.0
        while not self._stop.is_set():
            if self._send_heartbeat and time.monotonic() >= next_heartbeat:
                self._emit_heartbeat()
                next_heartbeat = time.monotonic() + _HEARTBEAT_PERIOD_S

            msg = self._conn.recv_match(blocking=True, timeout=_RECV_TIMEOUT_S)
            if msg is None:
                continue

            name = message_name(msg)
            if name == "BAD_DATA":
                # Framing errors and non-MAVLink noise on the wire; counted so
                # a misconfigured link is visible, but never forwarded.
                self.stats.bad_data += 1
                continue

            self.stats.received += 1
            self.stats.messages_by_type[name] = self.stats.messages_by_type.get(name, 0) + 1
            self._publish(Received(msg, time.time_ns()))

    def _emit_heartbeat(self) -> None:
        """Announce ourselves as a GCS so vehicles start/keep streaming telemetry."""
        try:
            self._conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
        except Exception:  # noqa: BLE001 - e.g. udpin with no peer yet
            logger.debug("Heartbeat send failed (no peer yet?)", exc_info=True)

    def _publish(self, item: Received) -> None:
        """Hand a message to the event loop, dropping the oldest when saturated."""
        loop = self._loop
        if loop is None or loop.is_closed():
            return
        loop.call_soon_threadsafe(self._enqueue, item)

    def _enqueue(self, item: Received) -> None:
        # Runs on the event loop thread, so queue access needs no extra locking.
        if self._queue.full():
            try:
                self._queue.get_nowait()
                self.stats.dropped += 1
            except asyncio.QueueEmpty:  # pragma: no cover - drained concurrently
                pass
        try:
            self._queue.put_nowait(item)
        except asyncio.QueueFull:  # pragma: no cover - drained concurrently
            self.stats.dropped += 1
