#pragma once

#include <cstdint>

namespace talos::protocol {

constexpr uint16_t kProtocolVersion = 3;
static_assert(kProtocolVersion == 3, "Update docs/DATA_FLOW.md");
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
