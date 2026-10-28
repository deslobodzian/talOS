#pragma once

#include <cstdint>

namespace talos::protocol {

constexpr uint16_t kProtocolVersion = 1;
constexpr uint64_t kFrameMagic = 0x4F4952534F4C4154ull;  // "TALOSRIO" LE

enum class FrameType : uint16_t {
  kHardwareState = 20,
  kHardwareCommand = 21,
  kHardwareConfig = 22,
  kHardwareConfigAck = 23,
  kDriverStation = 24,
};

enum FrameFlags : uint16_t {
  kFlagNone = 0,
  kFlagAckRequested = 1u << 0,
  kFlagAck = 1u << 1,
  kFlagFragment = 1u << 2,
};

}  // namespace talos::protocol
