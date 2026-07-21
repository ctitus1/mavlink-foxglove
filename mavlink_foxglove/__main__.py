"""Console entrypoint: ``python -m mavlink_foxglove``."""

from __future__ import annotations

import asyncio
import logging
import signal
import sys

from .bridge import MavlinkFoxgloveBridge
from .config import Config, load_config


def configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
        stream=sys.stdout,
    )


async def _run(config: Config) -> None:
    """Run the bridge until SIGINT/SIGTERM, so containers stop cleanly."""
    bridge = MavlinkFoxgloveBridge(config)
    task = asyncio.ensure_future(bridge.run())

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, task.cancel)
        except NotImplementedError:  # pragma: no cover - non-POSIX platforms
            pass

    try:
        await task
    except asyncio.CancelledError:
        logging.getLogger(__name__).info("Shutting down")


def main(argv: list[str] | None = None) -> int:
    config = load_config(argv)
    configure_logging(config.log_level)
    try:
        asyncio.run(_run(config))
    except KeyboardInterrupt:  # pragma: no cover - handled via signals normally
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
