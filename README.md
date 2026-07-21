# mavlink-foxglove

[![CI](https://github.com/ctitus1/mavlink-foxglove/actions/workflows/ci.yml/badge.svg)](https://github.com/ctitus1/mavlink-foxglove/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A Docker container that listens for MAVLink telemetry and exposes **every**
MAVLink message as a live [Foxglove](https://foxglove.dev) topic — the same role
`foxglove_bridge` plays for ROS.

Point it at a UDP port, open `ws://localhost:8765` in Foxglove, and every
message your vehicle emits shows up as a typed, schema-described channel with no
per-message configuration.

```
        MAVLink/UDP                     Foxglove WebSocket Protocol
vehicle ────────────► :14445  [bridge]  :8765 ──────────────────────► Foxglove
```

## Quick start

```bash
docker compose up --build
```

Then in Foxglove: **Open connection → Foxglove WebSocket → `ws://localhost:8765`**.

Without compose:

```bash
docker build -t mavlink-foxglove .
docker run --rm \
  -p 127.0.0.1:14445:14445/udp \
  -p 127.0.0.1:8765:8765 \
  mavlink-foxglove
```

To point a vehicle or SITL at the bridge, forward telemetry to
`udp://127.0.0.1:14445`. For PX4 SITL:

```bash
# in PX4's pxh> console
mavlink start -u 14445 -r 50000 -o 14445
```

For ArduPilot SITL, add `--out=udp:127.0.0.1:14445` to `sim_vehicle.py`, and run
the bridge with `--autopilot ardupilot`.

## What you get

### Generic channels — one per message type

Every message becomes a topic named by `--topic-template`, which defaults to
`/mavlink/{system_id}/{component_id}/{message}`:

```
/mavlink/1/1/ATTITUDE
/mavlink/1/1/GLOBAL_POSITION_INT
/mavlink/1/1/BATTERY_STATUS
...
```

MAVLink fields sit at the **top level** of the payload so Foxglove plot paths
read naturally (`/mavlink/1/1/ATTITUDE.roll`). Routing metadata is nested under
`_meta`, which cannot collide with a MAVLink field name:

```json
{
  "_meta": {
    "system_id": 1, "component_id": 1, "sequence": 42, "message_id": 30,
    "receive_timestamp": { "sec": 1763000000, "nsec": 123456789 }
  },
  "time_boot_ms": 51230,
  "roll": 0.0187, "pitch": -0.0032, "yaw": 1.5708,
  "rollspeed": 0.001, "pitchspeed": 0.0, "yawspeed": 0.0
}
```

Each channel is advertised with a JSON Schema generated from pymavlink's own
field metadata, including **units** and **enum names** in field descriptions.

### Enum name companions

Enum-typed fields get a `<field>_enum` string alongside the raw number, so
tables and state-transition panels are readable:

```json
{ "fix_type": 3, "fix_type_enum": "GPS_FIX_TYPE_3D_FIX" }
```

Disable with `--no-enum-names`.

### Derived channels — Foxglove well-known schemas

Generic channels are complete but untyped as far as Foxglove is concerned, so
they only drive the Raw Message, Table and Plot panels. The bridge additionally
republishes a few messages under schema names Foxglove recognises, which lights
up the richer panels:

| Source MAVLink message | Derived topic         | Schema                 | Enables       |
| ---------------------- | --------------------- | ---------------------- | ------------- |
| `GLOBAL_POSITION_INT`  | `.../location`        | `foxglove.LocationFix` | Map panel     |
| `GPS_RAW_INT`          | `.../gps_location`    | `foxglove.LocationFix` | Map panel     |
| `ATTITUDE`             | `.../attitude_transform` | `foxglove.FrameTransform` | 3D panel |
| `STATUSTEXT`           | `.../log`             | `foxglove.Log`         | Log panel     |

Disable with `--no-derived-topics`. Adding a converter is a single entry in
`CONVERTERS` in `mavlink_foxglove/derived.py`.

## Configuration

Every option is available as a CLI flag and as an environment variable prefixed
with `MAVLINK_FOXGLOVE_`. Boolean flags take a `--no-` form.

| Flag | Env var | Default | Purpose |
| --- | --- | --- | --- |
| `--mavlink-url` | `MAVLINK_URL` | `udpin:0.0.0.0:14445` | pymavlink connection string |
| `--autopilot` | `AUTOPILOT` | `px4` | Preset: `px4`, `ardupilot`, `both`, `all` |
| `--dialect` | `DIALECT` | from `--autopilot` | Exact dialect, overrides the preset |
| `--wire-version` | `WIRE_VERSION` | `2` | MAVLink 1 or 2 framing |
| `--ws-host` | `WS_HOST` | `0.0.0.0` | WebSocket bind address |
| `--ws-port` | `WS_PORT` | `8765` | WebSocket port |
| `--topic-template` | `TOPIC_TEMPLATE` | `/mavlink/{system_id}/{component_id}/{message}` | Topic naming |
| `--queue-size` | `QUEUE_SIZE` | `10000` | Buffered messages before dropping oldest |
| `--enum-names` | `ENUM_NAMES` | `true` | Emit `<field>_enum` companions |
| `--derived-topics` | `DERIVED_TOPICS` | `true` | Publish Foxglove well-known schemas |
| `--advertise-all` | `ADVERTISE_ALL` | `false` | Advertise the whole dialect at startup |
| `--send-heartbeat` | `SEND_HEARTBEAT` | `true` | Send a 1 Hz GCS heartbeat |
| `--log-level` | `LOG_LEVEL` | `INFO` | Python log level |

### Autopilot selection

`--autopilot px4` (default) uses the stock `common` dialect, which is what PX4
speaks. `--autopilot ardupilot` uses `ardupilotmega`.

Because `ardupilotmega` is a strict superset of `common`, `--autopilot both` also
decodes PX4 traffic — use it for a mixed fleet on one bridge.

### Connection strings

`--mavlink-url` is passed to `pymavlink.mavutil.mavlink_connection`, so anything
it accepts works:

| Value | Meaning |
| --- | --- |
| `udpin:0.0.0.0:14445` | Listen for UDP (the default) |
| `udpout:192.168.1.10:14550` | Send to, and receive from, a fixed peer |
| `tcp:192.168.1.10:5760` | TCP client |
| `/dev/ttyACM0` | Serial (needs `--device` passthrough in Docker) |

## Message version pinning

Two axes are supported today:

* `--dialect` — which XML definition set.
* `--wire-version` — MAVLink 1 vs 2 framing, which also selects pymavlink's
  `v10`/`v20` generated module tree.

Pinning to an **older revision** of a dialect's definitions is not yet exposed as
a flag, but the design accommodates it: `mavlink_foxglove/dialect.py` is the only
module that resolves message definitions, and everything downstream works purely
by reflection over whatever it returns. Generating a module from an archived
`*.xml` with `pymavlink.generator.mavgen` and returning it from `load_dialect()`
is sufficient — no other module needs to change.

Pinning the `pymavlink` version in `requirements.txt` is the blunt instrument
that works today.

## Architecture

Each module has one job and is independently testable.

| Module | Responsibility |
| --- | --- |
| `config.py` | Resolve CLI flags and env vars into a frozen `Config` |
| `dialect.py` | The *only* place message definitions are resolved |
| `schema.py` | pymavlink message class → JSON Schema |
| `encoding.py` | pymavlink message object → JSON-safe dict |
| `channels.py` | Lazily advertise Foxglove channels, cache topic → channel ID |
| `source.py` | Blocking pymavlink reader on a thread → asyncio queue |
| `derived.py` | Pure converters to Foxglove well-known schemas |
| `bridge.py` | Wiring only |

### Notable implementation details

These are the things that make it robust across *all* messages rather than the
handful a typical vehicle sends.

**pymavlink uses two conflicting field orderings.** `fieldtypes` is indexed in
`fieldnames` (declaration) order, but `array_lengths` is indexed in
`ordered_fieldnames` (wire) order. They coincide for many messages, which makes
the bug easy to ship: `PARAM_REQUEST_READ` has `char[16] param_id` at
`fieldnames` index 2, but that `16` sits at `array_lengths` index 3. Reading both
with one index turns `param_id` into a scalar and `param_index` into a
16-element array. `schema.field_specs()` is the single place this is untangled,
and `tests/test_schema.py` pins it for every char field in every dialect.

**NaN and Infinity are not valid JSON.** MAVLink floats routinely carry them to
mean "unknown", and `json.dumps` emits bare `NaN` tokens by default, which
Foxglove rejects. `encoding.sanitize()` maps non-finite floats to `null`, float
schemas are typed `["number", "null"]` to match, and the bridge serialises with
`allow_nan=False` so any escapee is a loud error rather than silent corruption.

**64-bit integers lose precision.** JSON numbers are IEEE-754 doubles, so
`uint64`/`int64` values above 2^53 (e.g. `time_usec`) are not exact in Foxglove.
The generated schema flags affected fields in their description.

**`char[N]` fields are strings, not arrays**, and are NUL-padded. They arrive as
`str` on modern pymavlink and `bytes` on older releases, and are not always valid
UTF-8, so decoding uses `errors="replace"` and never raises.

**Backpressure is bounded.** A stalled Foxglove client must not let the bridge
grow without limit, so the queue drops oldest messages and counts them; drops
appear in the periodic stats line.

**Failures are contained.** A single bad message cannot stop the pump, a link
fault reconnects with backoff instead of killing the reader thread, and a
derived-topic converter error degrades only that derived topic.

## Development

```bash
pip install -e '.[dev]'
pytest
```

The suite has 38 tests in four files:

| File | Covers |
| --- | --- |
| `tests/test_schema.py` | Schema generation for **every message in every dialect**; the field-ordering regression |
| `tests/test_encoding.py` | JSON validity for **every message**, with NaN in every float field |
| `tests/test_derived.py` | Well-known schema converters, including non-finite input |
| `tests/test_integration.py` | Real UDP → real bridge → real Foxglove protocol client, validating payloads against advertised schemas |

`tests/test_integration.py` is the meaningful one: it runs the actual bridge,
speaks the genuine Foxglove WebSocket subprotocol, and validates every received
payload against the schema the bridge advertised for it.

> If you have ROS installed system-wide, its `launch_testing` pytest plugins are
> incompatible with modern pytest. `pyproject.toml` already disables them.

### Sending test traffic

`tools/mavlink_test_publisher.py` generates synthetic MAVLink:

```bash
# One instance of every message type in the dialect -- the robustness check.
python tools/mavlink_test_publisher.py --mode all

# A realistic moving-vehicle loop, for eyeballing plots, Map and 3D panels.
python tools/mavlink_test_publisher.py --mode telemetry --duration 60
```

Both default to `udpout:127.0.0.1:14445`. `--mode all` deliberately includes
NaN and infinite floats, non-ASCII bytes in char fields, and 64-bit values
above 2^53.

### Smoke-testing a running container

The pytest suite runs the bridge in-process. To validate a *deployed* instance
over its real network interfaces:

```bash
docker compose up -d
python tools/smoke_test.py
```

It sends the full dialect, checks every advertised schema is a legal JSON
Schema, validates live payloads against the schema advertised for their
channel, and fails if any `NaN`/`Infinity` token reaches the wire. It exits
non-zero on failure, which is how CI gates the image.

## Continuous integration

`.github/workflows/ci.yml` runs on every push and pull request:

* **`test`** — the pytest suite on Python 3.9, 3.10, 3.11 and 3.12.
* **`docker`** — builds the image, starts the container, runs
  `tools/smoke_test.py` against it, and fails if the container logged any
  error.

## Verified behaviour

Against the running container, driven by the test publisher over host UDP:

* **205 message types** received and advertised as channels, plus 4 derived
  channels — 209 total.
* **All 209 advertised schemas** validate as legal JSON Schema documents.
* **Live payloads validate against their own advertised schemas**, with no
  `NaN`/`Infinity` tokens on the wire.
* **Zero errors** in container logs across the full-dialect sweep;
  `dropped=0 bad_data=0 reconnects=0`.

Reproduce with `docker compose up -d`, then
`python tools/mavlink_test_publisher.py --mode all`.

## Limitations

* **Read-only.** The bridge subscribes to telemetry; it does not accept commands
  from Foxglove back to the vehicle.
* **JSON encoding**, not Protobuf — simpler and fully dynamic, at the cost of
  bandwidth and 64-bit integer precision.
* **Lazy advertisement.** A topic appears only once its first message arrives.
  Use `--advertise-all` if a saved layout needs to resolve topics up front.
* **`_meta.receive_timestamp` is host receive time**, not vehicle time. Vehicle
  timestamps remain available in their original fields (`time_boot_ms`,
  `time_usec`).

## License

[MIT](LICENSE).
