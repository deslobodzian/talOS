#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

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

// The topics either end of the controller link is addressed by.
//
// `hw` is a reserved owner namespace rather than a node name (see
// talOS/introspection/names.h) because the publisher of these topics is not a
// talOS node. The bridge process holds the shared-memory ends, but it is only a
// courier: the state, the driver station bytes and the accepted commands all
// belong to the controller processor across the UDP link, which runs no event
// loop, claims no registry slot and has no name to put in a first segment.
// Naming the owner `hardware_node` would say the bridge authored what it
// merely relayed, and it would break every subscriber the day the courier is
// renamed or replaced by a second transport.
inline constexpr const char* kStateTopic = "/hw/state";
inline constexpr const char* kCommandTopic = "/hw/command";
inline constexpr const char* kDriverStationTopic = "/hw/state/driver_station";

// Subsystem nodes name their own request topic as kRequestTopicPrefix plus
// their own subsystem name -- the `[subsystems.<name>]` key, spelled exactly,
// with no abbreviation. The bridge derives the same string from the config, so
// the two ends cannot be spelled differently without the launcher noticing.
inline constexpr const char* kRequestTopicPrefix = "/hw/request/";

// The one place the request topic is spelled. Both ends call this -- the
// subsystem node for the topic it publishes, the bridge for the topic it reads
// -- so a rename is one edit and cannot leave one side behind.
inline std::string RequestTopic(std::string_view subsystem) {
  return std::string{kRequestTopicPrefix} + std::string{subsystem};
}

// A debug and legacy hook: a whole pre-merged Command, applied as-is, bypassing
// per-subsystem arbitration and timeout neutralization. Nothing in the tree
// publishes it, so the bridge's end of it is declared optional.
inline constexpr const char* kCommandOverrideTopic = "/hw/command/override";
}  // namespace talos::hardware
