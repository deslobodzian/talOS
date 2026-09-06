#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "talOS/protocol/frame.h"

namespace talos::hardware {
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

// Topics the bridge owns. Subsystem nodes name their own request topic as
// kRequestTopicPrefix + subsystem name.
inline constexpr const char* kStateTopic = "/hw/state";
inline constexpr const char* kCommandTopic = "/hw/cmd";
inline constexpr const char* kDriverStationTopic = "/hw/ds";
inline constexpr const char* kRequestTopicPrefix = "/hw/req/";
}  // namespace talos::hardware
