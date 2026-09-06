#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "common/protocol/frame.h"

namespace talos::shooter {

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

inline constexpr const char* kHwStateTopic = "/hw/state";
inline constexpr const char* kShooterStateTopic = "/shooter/state";
inline constexpr const char* kShooterTargetTopic = "/shooter/tgt";
inline constexpr const char* kShooterRequestTopic = "/hw/req/shooter";

}  // namespace talos::shooter
