# TalOS Controller Protocol

This package is the wire contract between the companion bridge
(`//talOS/bridge:hardware_node`) and the realtime controller gateway
(`talOS/hardware`, run by the RoboRIO program or `sim_gateway`).

The end-to-end [data-flow reference](../../docs/DATA_FLOW.md) records link
rates, exact payload/slot sizes, config chunking, and the current robot budget.

## Frames

Each UDP datagram contains one frame. `EncodeFrame` explicitly encodes
multi-byte header fields little-endian; native C++ layout is not the wire
layout. Ports, protocol version, header and payload sizes are recorded in the
[data-flow reference](../../docs/DATA_FLOW.md#udp-frame-and-payload-sizes).

| `FrameType` | Value | Direction | Payload |
|---|---|---|---|
| `kHardwareState` | 20 | gateway → companion | `hardware::State` (`talOS/hardware/messages.h`) |
| `kHardwareCommand` | 21 | companion → gateway | `hardware::Command` |
| `kHardwareConfig` | 22 | companion → gateway | one chunk of the serialized `hardware::Config` (`config_wire.h`) |
| `kHardwareConfigAck` | 23 | gateway → companion | `ConfigAckPayload` |
| `kDriverStation` | 24 | gateway → companion | Driver Station snapshot (`talOS/driver_station`) |
| `kClockProbe` | 25 | companion → gateway | empty; header send time echoed in a subsequent State |

Payload codecs live with the payload owners; this package only frames bytes.
The frame decoder rejects incompatible protocol versions before payload
decoding. Clock probes do not authorize actuation and work while disabled.

## Low-Latency UDP API

`RuntimeUdpPeer` is the hot-path socket wrapper for both sides:

- connected UDP, so the kernel filters packets from unexpected peers;
- non-blocking by default for control-loop polling;
- fixed TX/RX packet buffers, no allocation per send or receive;
- explicit sequence numbers and sequence-gap counters;
- version/header validation before payload interpretation;
- DSCP EF (`0xB8`) requested through `IP_TOS` where the OS honors it.

Use `MutablePayloadBuffer()` plus `SendBufferedFrame()` to build the payload
directly inside the outgoing datagram buffer, avoiding an extra copy.
`SendFrame()` copies from a caller buffer.

Loopback tests expecting a sent frame use the test-only
`udp_test_helpers.h` `testing::ReceiveFrame()` delivery barrier. It bounds
wall-clock waiting without advancing simulated time or resending, and returns
socket/decode errors immediately. Negative receive assertions still use
`TryReceive()` directly.

## Safety Boundary

The protocol carries commands; it does not make them safe. The gateway must
still enforce freshness, enable state, command bounds, and subsystem-specific
safety rules before any command reaches hardware.
