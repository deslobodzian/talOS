#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace talos::telemetry_wire {

// The telemetry wire contract, shared by the producer
// (//talOS/telemetry:node), the Studio bridge
// (//studio/bridge) and the tests. See talOS/telemetry/README.md for the
// full protocol and the size/rate budget.
//
// RTMS slots hold a size-prefixed FlatBuffer (4-byte little-endian size,
// then the Frame buffer whose file identifier is "TLM3") at offset 0; the
// rest of the slot is unused padding. payload() returns the prefix plus
// buffer -- the frame as the Studio bridge decodes it -- or an empty view
// when the slot does not hold a well-formed v3 frame header.

// A value page (no dictionary) including its 4-byte prefix. A tick's values
// take as many pages as they need; each page, from the producer and from the
// Studio bridge to a client, fits one 1500-byte-MTU Ethernet frame with
// IP/TCP/WebSocket overhead. The bridge's egress budget, not this cap, bounds
// the radio rate (docs/DATA_FLOW.md).
inline constexpr std::size_t kMaxValueFrameBytes = 1200;
static_assert(kMaxValueFrameBytes == 1200,
              "telemetry value page limit changed; update docs/DATA_FLOW.md");

// Per carried sample: a uint32 channel index and a uint64 raw value. An
// absent channel costs less (its uint32 index alone).
inline constexpr std::size_t kSampleBytes = 12;
// Upper bound on a value page's bytes besides its samples and absent
// channels: header scalars, chassis, vtable, vector lengths and alignment
// (frames.h measures the real overhead; its test holds it under this).
inline constexpr std::size_t kValuePageOverheadBound = 200;

// The Studio bridge's per-client egress burst (studio/bridge EgressBudget)
// and the WebSocket header it charges per message: the bridge sends a client
// an update only when it can afford all of it at once, so an update larger
// than the burst could never be sent.
inline constexpr std::size_t kClientBurstBytes = 64000;
inline constexpr std::size_t kMaxWebSocketHeaderBytes = 10;

// The most channels a dictionary may hold: sized so that the largest update,
// a keyframe carrying every channel, fits the bridge's burst with all its
// page and WebSocket framing. The producer refuses a larger dictionary at
// construction, naming its largest sources.
inline constexpr std::size_t kMaxChannels = 2048;
inline constexpr std::size_t kMinSamplesPerPage =
    (kMaxValueFrameBytes - kValuePageOverheadBound) / kSampleBytes;
inline constexpr std::size_t kMaxUpdateBytes =
    (kMaxChannels + kMinSamplesPerPage - 1) / kMinSamplesPerPage *
    (kMaxValueFrameBytes + kMaxWebSocketHeaderBytes);
static_assert(kMaxUpdateBytes <= kClientBurstBytes,
              "a keyframe of kMaxChannels must fit the Studio bridge's "
              "egress burst; update docs/DATA_FLOW.md with either");

// Slot geometry for the /talos/telemetry byte topic. One slot holds one
// value page or one dictionary page; a dictionary takes as many pages as it
// needs, each at most one slot, so one channel's entry (name, unit, labels)
// must fit one slot. 64 slots hold at least 0.3 s of a two-page tick at
// 100 Hz, which a reader that falls further behind loses (the writer
// overwrites; the reader is told how many it missed).
inline constexpr std::size_t slot_bytes = 16384;
inline constexpr std::size_t slot_count = 64;
inline constexpr std::size_t kMaxDictionaryPageBytes = slot_bytes;
// A page, prefix included, is what one slot holds (payload() checks it).
static_assert(kMaxDictionaryPageBytes <= slot_bytes &&
                  kMaxValueFrameBytes <= slot_bytes,
              "a telemetry page must fit one /talos/telemetry slot");

// Dictionary limits: each channel name, unit and enum label is at most
// kMaxDictionaryStringBytes, and an enum channel carries at most
// kMaxEnumLabels labels. Studio's decoder (studio/src/core.ts) accepts
// exactly these, and the producer (node.h) never emits past them: one
// channel past them would blank the whole stream.
inline constexpr std::size_t kMaxDictionaryStringBytes = 256;
inline constexpr std::size_t kMaxEnumLabels = 1024;
static_assert(kMaxDictionaryStringBytes == 256 && kMaxEnumLabels == 1024,
              "telemetry dictionary limits changed; update docs/DATA_FLOW.md "
              "and studio/src/core.ts");

// Validates the slot header: a 4-byte little-endian FlatBuffers size
// prefix, a minimum FlatBuffer size (root offset + "TLM3" identifier),
// a size that fits the slot, and the TLM3 identifier itself. Full
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
      slot[10] != std::byte{'M'} || slot[11] != std::byte{'3'}) {
    return {};
  }
  return {reinterpret_cast<const char*>(slot.data()), n + 4};
}

}  // namespace talos::telemetry_wire
