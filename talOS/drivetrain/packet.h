#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "common/protocol/frame.h"

namespace talos::drive {
// Fixed-size local IPC envelope containing canonical hardware wire bytes.
// Zero-initialized unused bytes make recorded outputs byte-stable.
struct Packet {
  uint32_t size{};
  std::array<uint8_t, protocol::kMaxPayloadSize> data{};
  std::span<const uint8_t> bytes() const {
    return size <= data.size() ? std::span<const uint8_t>{data.data(), size}
                               : std::span<const uint8_t>{};
  }
};
inline constexpr const char* kStateTopic = "/hw/state";
inline constexpr const char* kCommandTopic = "/hw/cmd";
inline constexpr const char* kDriveRequestTopic = "/hw/req/drive";
inline constexpr const char* kTargetTopic = "/drivetrain/tgt";
inline constexpr const char* kDrivetrainStateTopic = "/drivetrain/state";
}  // namespace talos::drive
