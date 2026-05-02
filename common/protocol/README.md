# TalOS Mac/Rio Protocol

This package is the shared wire contract between the companion `rio_client`
node and the roboRIO `systemcore-rio` gateway.

## Research:
[Low Latency C++ for HFT](https://stacygaudreau.com/blog/cpp/low-latency-cpp-for-hft-part3-network-programming/)

## Transports

- TCP port `5801`: startup/config negotiation.
- UDP port `5802`: runtime state, commands, heartbeats, and faults.

All traffic uses the same fixed 40 byte frame header followed by an optional
payload of up to 1200 bytes. Multi-byte header fields are encoded little-endian
explicitly by `talos::protocol::EncodeFrame`; the C++ struct layout is not used
as the wire layout.

## Startup Flow

1. Rio sends `FrameType::kHello` with `RioHello`.
2. Companion sends `FrameType::kManifest`.
3. Rio validates command/state/fault IDs and replies with
   `FrameType::kManifestAck` or `FrameType::kConfigError`.
4. Runtime UDP begins only after the manifest is accepted.

## Runtime Flow

- Rio publishes `FrameType::kRioState` at the configured control period.
- Companion publishes `FrameType::kRioCommand` at the configured command period.
- Either side may publish `FrameType::kHeartbeat` if no runtime frame was sent
  recently.
- Rio publishes `FrameType::kFault` for safety or protocol faults.

Runtime payloads start with `RuntimeValuesHeader`, followed by `SignalValue`
entries. The `manifest_id` in each runtime payload must match the accepted
manifest.

## Low-Latency UDP API

`RuntimeUdpPeer` is the shared hot-path socket wrapper for both sides:

- connected UDP, so the kernel filters packets from unexpected peers;
- non-blocking by default for control-loop polling;
- fixed TX/RX packet buffers, no allocation per send or receive;
- explicit sequence numbers and sequence-gap counters;
- version/header validation before payload interpretation;
- DSCP EF (`0xB8`) requested through `IP_TOS` where the OS honors it.

Use `MutablePayloadBuffer()` plus `SendBufferedFrame()` for the runtime hot
path. That lets the caller build the payload directly inside the outgoing UDP
datagram buffer, avoiding an extra userspace payload copy. `SendFrame()` remains
available for tests, config, and lower-rate convenience paths.

## WPILib Simulation Smoke Test

The robot simulation opens a small UDP endpoint in `SimulationInit()`:

- simulated Rio side: `127.0.0.1:5802`
- TalOS side: `127.0.0.1:5803`

Run the WPILib simulation from the `robot/` directory:

```sh
./gradlew simulateNative
```

Then, in another terminal from the repo root, run:

```sh
bazel run //common/protocol:sim_udp_smoke -- --iterations=100
```

The TalOS smoke tool sends `RioCommand` frames and expects `RioState` replies
from the WPILib simulation. This verifies that the shared UDP protocol works
between the simulated Rio process and a TalOS-side process before real hardware
is involved.

## Safety Boundary

The protocol carries commands; it does not make them safe. The Rio gateway must
still enforce freshness, enable state, command bounds, and subsystem-specific
safety rules before any command reaches hardware.
