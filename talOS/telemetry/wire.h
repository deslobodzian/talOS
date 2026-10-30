#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace talos::telemetry_wire {

// The telemetry wire contract, shared by the producer
// (//2026-robot/main_processor/telemetry), the Studio bridge
// (//studio/bridge) and the tests. See talOS/telemetry/README.md for the
// full protocol and the size/rate budget.
//
// RTMS slots hold a size-prefixed FlatBuffer (4-byte little-endian size,
// then the Frame buffer whose file identifier is "TLM2") at offset 0; the
// rest of the slot is unused padding. payload() returns the prefix plus
// buffer -- exactly what the bridge forwards on the WebSocket -- or an
// empty view when the slot does not hold a well-formed v2 frame header.

// A value-only frame (no dictionary) including its 4-byte prefix. 1200 B
// fits one 1500-byte-MTU Ethernet frame with IP/TCP/WebSocket overhead,
// and at the 100 Hz telemetry rate it is 1200 * 8 * 100 = 0.96 Mbit/s per
// client, under 25% of the 4 Mbit/s FRC radio floor (rule R704 budget).
inline constexpr std::size_t kMaxValueFrameBytes = 1200;

// Slot geometry for the /talos/telemetry byte topic. A dictionary frame
// (names, units, labels) is larger than a value frame but still bounded
// by one slot; 64 slots at 100 Hz absorb over half a second of bridge
// lag before DROP_NEWEST starts shedding frames.
inline constexpr std::size_t slot_bytes = 16384;
inline constexpr std::size_t slot_count = 64;

// Validates the slot header: a 4-byte little-endian FlatBuffers size
// prefix, a minimum FlatBuffer size (root offset + "TLM2" identifier),
// a size that fits the slot, and the TLM2 identifier itself. Full
// FlatBuffers verification is the consumer's job (the bridge does it
// when decoding for /telemetry.json); this stays cheap enough to run
// per frame on the forwarding path.
inline std::string_view payload(std::span<const std::byte> slot) {
  if (slot.size() < 4) return {};
  std::uint32_t n = 0;
  for (unsigned i = 0; i < 4; ++i) {
    n |= std::to_integer<std::uint32_t>(slot[i]) << (8 * i);
  }
  // n counts the FlatBuffer bytes after the prefix. The smallest Frame
  // buffer is the 4-byte root offset plus the 4-byte identifier.
  if (n < 8 || n > slot_bytes - 4 || n > slot.size() - 4) return {};
  // The file identifier sits right after the root-table offset:
  // slot bytes [8, 12) are buffer bytes [4, 8).
  if (slot[8] != std::byte{'T'} || slot[9] != std::byte{'L'} ||
      slot[10] != std::byte{'M'} || slot[11] != std::byte{'2'}) {
    return {};
  }
  return {reinterpret_cast<const char*>(slot.data()), n + 4};
}

}  // namespace talos::telemetry_wire
