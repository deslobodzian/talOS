# TalOS Controller Protocol

This package is the wire contract between the companion bridge
(`//talOS/bridge:hardware_node`) and the realtime controller gateway
(`talOS/hardware`, run by the RoboRIO program or `sim_gateway`).

## Frames

All traffic is UDP: the gateway listens on port 5802 and the companion on
5803. Every datagram is one frame: a fixed 40 byte header
(`kFrameHeaderSize`) followed by an optional payload of up to
`kMaxPayloadSize` bytes, which is 1400: a 1440-byte frame fits inside the 1472
bytes a 1500-byte Ethernet MTU leaves after the IPv4 and UDP headers, and
`frame.h` has a static assertion to that effect. Multi-byte header fields are
encoded little-endian explicitly by `EncodeFrame`; the C++ struct layout is not
the wire layout.

| `FrameType` | Value | Direction | Payload |
|---|---|---|---|
| `kHardwareState` | 20 | gateway → companion | `hardware::State` (`talOS/hardware/messages.h`) |
| `kHardwareCommand` | 21 | companion → gateway | `hardware::Command` |
| `kHardwareConfig` | 22 | companion → gateway | one chunk of the serialized `hardware::Config` (`config_wire.h`) |
| `kHardwareConfigAck` | 23 | gateway → companion | `ConfigAckPayload` |
| `kDriverStation` | 24 | gateway → companion | Driver Station snapshot (`talOS/driver_station`) |

Payload codecs live with the payload owners; this package only frames bytes.

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

## Safety Boundary

The protocol carries commands; it does not make them safe. The gateway must
still enforce freshness, enable state, command bounds, and subsystem-specific
safety rules before any command reaches hardware.
