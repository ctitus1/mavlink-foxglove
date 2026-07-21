"""The bridge: MAVLink ingest -> Foxglove channels.

Wiring only. Each collaborator is independently testable:
:mod:`.source` produces messages, :mod:`.schema` describes them,
:mod:`.encoding` serialises them, :mod:`.channels` advertises them, and
:mod:`.derived` adds Foxglove-native republications.
"""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Any

from foxglove_websocket.server import FoxgloveServer

from . import derived
from .channels import ChannelRegistry
from .config import Config
from .dialect import configure_mavutil, load_dialect, message_classes, message_name
from .encoding import encode_message, topic_for
from .schema import message_schema
from .source import MavlinkSource

logger = logging.getLogger(__name__)

#: How often to log throughput statistics.
_STATS_INTERVAL_S = 10.0


class MavlinkFoxgloveBridge:
    """Serves every received MAVLink message as a Foxglove WebSocket channel."""

    def __init__(self, config: Config) -> None:
        self._config = config
        self._dialect = load_dialect(config.dialect, config.wire_version)
        configure_mavutil(config.dialect, config.wire_version)

        self._source = MavlinkSource(
            url=config.mavlink_url,
            dialect_name=config.dialect,
            queue_size=config.queue_size,
            send_heartbeat=config.send_heartbeat,
        )
        # Schemas are derived once per message class and reused across every
        # (system, component) that emits that message.
        self._schema_cache: dict[str, dict[str, Any]] = {}
        self._server: FoxgloveServer | None = None
        self._registry: ChannelRegistry | None = None

    async def run(self) -> None:
        """Run the bridge until cancelled."""
        config = self._config
        async with FoxgloveServer(
            config.ws_host,
            config.ws_port,
            "mavlink-foxglove",
            supported_encodings=["json"],
        ) as server:
            self._server = server
            registry = self._registry = ChannelRegistry(server)
            if config.advertise_all:
                await self._advertise_all(registry)

            self._source.start(asyncio.get_running_loop())
            logger.info(
                "Bridging %s (dialect=%s v%d) to ws://%s:%d",
                config.mavlink_url,
                config.dialect,
                config.wire_version,
                config.ws_host,
                config.ws_port,
            )
            try:
                await asyncio.gather(self._pump(registry), self._log_stats())
            finally:
                self._source.stop()

    # -- internals ---------------------------------------------------------

    def _schema_for(self, name: str, msg_class: type) -> dict[str, Any]:
        cached = self._schema_cache.get(name)
        if cached is None:
            cached = message_schema(msg_class, self._config.enum_names)
            self._schema_cache[name] = cached
        return cached

    async def _advertise_all(self, registry: ChannelRegistry) -> None:
        """Pre-advertise every dialect message for system/component 1.

        Useful when a Foxglove layout must resolve topics before the vehicle has
        actually sent them; off by default because most of the channels stay
        empty.
        """
        for msg_class in message_classes(self._dialect):
            name = message_name(msg_class)
            topic = self._config.topic_template.format(
                system_id=1, component_id=1, message=name
            )
            await registry.ensure(topic, name, self._schema_for(name, msg_class))
        logger.info("Pre-advertised %d channels", len(registry))

    async def _pump(self, registry: ChannelRegistry) -> None:
        """Forward messages from the source onto Foxglove channels forever."""
        while True:
            item = await self._source.get()
            try:
                await self._publish(registry, item.message, item.receive_time_ns)
            except Exception:  # noqa: BLE001 - one bad message must not stop the bridge
                logger.exception(
                    "Failed to publish %s", message_name(item.message)
                )

    async def _publish(
        self, registry: ChannelRegistry, msg: Any, receive_time_ns: int
    ) -> None:
        name = message_name(msg)
        topic = topic_for(msg, self._config.topic_template)

        channel_id = await registry.ensure(
            topic, name, self._schema_for(name, type(msg))
        )
        payload = encode_message(
            msg, self._dialect, receive_time_ns, self._config.enum_names
        )
        await self._send(channel_id, receive_time_ns, payload)

        if self._config.derived_topics:
            await self._publish_derived(registry, name, msg, topic, receive_time_ns)

    async def _publish_derived(
        self,
        registry: ChannelRegistry,
        name: str,
        msg: Any,
        source_topic: str,
        receive_time_ns: int,
    ) -> None:
        for item in derived.convert(name, msg, receive_time_ns):
            # Derived topics sit alongside their source, e.g.
            # /mavlink/1/1/ATTITUDE -> /mavlink/1/1/attitude_transform.
            topic = source_topic.rsplit("/", 1)[0] + "/" + item.topic_suffix
            channel_id = await registry.ensure(topic, item.schema_name, item.schema)
            await self._send(channel_id, receive_time_ns, item.payload)

    async def _send(
        self, channel_id: int, timestamp_ns: int, payload: dict[str, Any]
    ) -> None:
        # allow_nan=False turns any non-finite float that slipped past
        # encoding.sanitize into a loud error here, rather than silently
        # emitting invalid JSON (bare `NaN`) that Foxglove would reject.
        encoded = json.dumps(payload, allow_nan=False).encode("utf-8")
        await self._server.send_message(channel_id, timestamp_ns, encoded)

    async def _log_stats(self) -> None:
        """Periodically report throughput so a silent link is obvious."""
        while True:
            await asyncio.sleep(_STATS_INTERVAL_S)
            stats = self._source.stats
            logger.info(
                "received=%d types=%d channels=%d dropped=%d bad_data=%d reconnects=%d",
                stats.received,
                len(stats.messages_by_type),
                len(self._registry) if self._registry else 0,
                stats.dropped,
                stats.bad_data,
                stats.reconnects,
            )
