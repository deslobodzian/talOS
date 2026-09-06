# Studio IPC bridge

Build from the monorepo root using Bazel:

```sh
bazel build //studio/bridge:studio_bridge //studio/bridge:studio_mock
bazel test //studio/bridge:wire_test //studio/bridge:integration_test
```

The monorepo pins FlatBuffers, uWebSockets v20.51.0 and its uSockets
submodule. SSL and WebSocket compression are disabled for this local bridge.
The bridge itself never parses FlatBuffers.

Build `studio/dist` using the frontend instructions. In separate terminals:

```sh
bazel-bin/studio/bridge/studio_mock /talos_studio
bazel-bin/studio/bridge/studio_bridge --drop-newest-publisher /talos_studio 127.0.0.1 5801 5800 studio/dist
```

The positional arguments are topic, destination IPv4 address, UDP destination
port, HTTP/WebSocket port and static web directory. Open `http://localhost:5800`;
binary frames arrive at `/telemetry`. The mock emits deterministic 200 Hz data
with nanosecond timestamps and source sequence IDs. Sequence IDs advance even
when a full queue rejects a frame, exposing the gap downstream.

## Dedicated DROP_NEWEST publisher contract

Use a **dedicated telemetry topic**, with exactly one producer, 256 slots of
65,536 bytes, alignment 8, and `OverflowPolicy::DROP_NEWEST`. Each slot starts
with a four-byte little-endian FlatBuffers size prefix, followed by the `TLMS`
FlatBuffer. The bridge transmits prefix plus payload, excluding slot padding.
Datagrams are limited to 65,507 bytes including the prefix. Smaller frames are
recommended because large UDP datagrams fragment and loss discards the frame.

The required `--drop-newest-publisher` argument acknowledges a contract that
RTMS cannot verify: the overflow policy is not stored in shared memory. Merely
setting DROP_NEWEST in the reader does **not** change the producer's policy.
OVERWRITE_OLDEST and REMOVE_SLOWEST_READER are unsafe here: a producer could
modify slot bytes while a network send is using them. Never attach this bridge
to an existing control-loop topic that uses those policies.

The producer must treat BUFFER_FULL as a dropped telemetry frame and continue
the control loop without waiting. DROP_NEWEST protects unread bytes during the
reader callback; the cursor advances only after synchronous send calls return.
This avoids an application-side staging copy or deserialization. Kernel socket
buffers and uWebSockets backpressure buffers still copy bytes; this is not
kernel zero-copy networking.

The publisher must start first and remain alive while the bridge runs. Stop the
bridge before restarting/replacing the publisher. Graceful SIGINT/SIGTERM frees
the reader slot; a SIGKILL/crash can strand a reader and cause subsequent writes
to be dropped. Restart both processes after such a crash. Shared-memory names
are local and limited by RTMS to 30 characters after the initial slash.

UDP is nonblocking. Slow WebSocket clients skip new frames once buffered bytes
exceed 256 KiB, with a 512 KiB uWebSockets backpressure cap. The shutdown summary
reports malformed envelopes and rejected UDP/WebSocket sends. HTTP static file
reads are synchronous and capped at 32 MiB per file; serving large assets during
streaming can increase jitter. The server has no authentication or TLS and is
intended for the local robot/development network. The current POSIX daemon runs
on Linux/macOS; the desktop frontend has a separate cross-platform Rust path.
