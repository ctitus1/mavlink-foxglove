"""Lazy advertisement of Foxglove channels.

Foxglove clients discover topics from server advertisements, so every distinct
``(topic, schema)`` pair must be registered exactly once before any message is
sent on it. A dialect defines a few hundred message types across an unknown
number of systems and components, so channels are created on first sight by
default -- mirroring how ``foxglove_bridge`` advertises the topics that
actually exist rather than every type that could exist.
"""

from __future__ import annotations

import json
import logging
from typing import Any

from foxglove_websocket.server import FoxgloveServer
from foxglove_websocket.types import ChannelId

logger = logging.getLogger(__name__)


def _serialize_schema(schema: dict[str, Any]) -> str:
    """JSON-encode a schema, dropping null-valued keys that clutter the UI."""
    return json.dumps({k: v for k, v in schema.items() if v is not None})


class ChannelRegistry:
    """Maps topic names to Foxglove channel IDs, advertising them on demand."""

    def __init__(self, server: FoxgloveServer) -> None:
        self._server = server
        self._channels: dict[str, ChannelId] = {}

    def __len__(self) -> int:
        return len(self._channels)

    @property
    def topics(self) -> list[str]:
        return sorted(self._channels)

    async def ensure(
        self, topic: str, schema_name: str, schema: dict[str, Any]
    ) -> ChannelId:
        """Return the channel ID for ``topic``, advertising it if it is new."""
        existing = self._channels.get(topic)
        if existing is not None:
            return existing

        channel_id = await self._server.add_channel(
            {
                "topic": topic,
                "encoding": "json",
                "schemaName": schema_name,
                "schema": _serialize_schema(schema),
                "schemaEncoding": "jsonschema",
            }
        )
        self._channels[topic] = channel_id
        logger.info("Advertised %s (%s) as channel %d", topic, schema_name, channel_id)
        return channel_id
