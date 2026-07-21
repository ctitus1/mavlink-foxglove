# Multi-stage build: dependencies are compiled in the builder and only the
# resulting site-packages are copied forward, keeping the runtime image small
# and free of build toolchains.
FROM python:3.12-slim AS builder

ENV PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

WORKDIR /build

# Copy only the dependency manifest first so this layer caches across source edits.
COPY requirements.txt ./
RUN python -m venv /opt/venv && \
    /opt/venv/bin/pip install --no-cache-dir -r requirements.txt


FROM python:3.12-slim AS runtime

ENV PATH="/opt/venv/bin:$PATH" \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

# Run as a non-root user: the bridge needs no privileges beyond two sockets.
RUN useradd --create-home --uid 10001 bridge

COPY --from=builder /opt/venv /opt/venv

WORKDIR /app
COPY mavlink_foxglove/ ./mavlink_foxglove/
COPY tools/ ./tools/

USER bridge

# 14445/udp: MAVLink in. 8765/tcp: Foxglove WebSocket out.
EXPOSE 14445/udp
EXPOSE 8765/tcp

# Verifies the WebSocket port is accepting connections.
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
    CMD python -c "import socket,sys; s=socket.create_connection(('127.0.0.1',8765),3); s.close()" || exit 1

ENTRYPOINT ["python", "-m", "mavlink_foxglove"]
