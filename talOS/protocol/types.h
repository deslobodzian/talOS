#pragma once

#include <cstdint>

namespace talos::protocol {

// Bumped by any change to a payload codec's bytes or to what it accepts,
// even one that keeps every length: recorded sessions are decoded only under
// the version they were recorded with (docs/DATA_FLOW.md#protocol-version).
constexpr uint16_t kProtocolVersion = 10;
static_assert(kProtocolVersion == 10,
              "Update docs/DATA_FLOW.md and docs/DEPLOYMENT.md, and regenerate "
              "the wire_golden_test constants in talOS/hardware and "
              "talOS/driver_station (docs/DATA_FLOW.md#protocol-version)");
constexpr uint64_t kFrameMagic = 0x4F4952534F4C4154ull;  // "TALOSRIO" LE

enum class FrameType : uint16_t {
  kHardwareState = 20,
  kHardwareCommand = 21,
  kHardwareConfig = 22,
  kHardwareConfigAck = 23,
  kDriverStation = 24,
  // Empty probe; header monotonic_time_us is echoed by a later State.
  kClockProbe = 25,
};

// No header flag has defined semantics yet: every sender writes kFlagNone.
// ConfigAck is a frame type and config chunking is payload-level.
enum FrameFlags : uint16_t {
  kFlagNone = 0,
};

}  // namespace talos::protocol
