#pragma once

// The schema identity of a topic: what every endpoint records when it
// registers, what the shared-memory segment header stores, what describe and
// the graph lint compare, and what a .tlog carries so log_dump can decode it.
//
//   name  "control.VelocityTarget": the fully-qualified schema name, or a
//         framework layout's name (below)
//   hash  Descriptor<M>::kHash (schema_hash.h), or a framework layout's hash
//   bfbs  the reflection.Schema bytes that declare `name`; empty for a layout
//         with no FlatBuffers schema
//
// Two endpoints agree on a topic exactly when their names and hashes agree;
// the bfbs bytes describe the type and are not compared (a comment change
// moves them, not the hash).
//
// Framework layouts, the topics that do not carry one talos_messages struct:
//
//   talos.Bytes                  a byte topic (Node::publish_bytes): opaque
//   talos.hardware.Packet        talOS/hardware/packet.h, the gateway link
//   talos.TargetEnvelope<NAME>   talOS/devices/target.h around struct NAME:
//                                its bfbs is NAME's
//
// Their hashes are FNV-1a 64 (the schema hash's function) of a canonical
// text: "talos.Bytes", "talos.hardware.Packet <sizeof>", and
// "talos.TargetEnvelope <inner hash as 16 lowercase hex digits>".

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include "talOS/schema/descriptor.h"

namespace talos::schema {

// Longest schema name a segment header or a log records.
inline constexpr std::size_t kMaxSchemaName = 127;

struct Identity {
  std::string name;
  std::uint64_t hash = 0;
  std::string bfbs;

  // Name and hash; see above.
  bool operator==(const Identity& other) const {
    return name == other.name && hash == other.hash;
  }
};

// FNV-1a 64, as flatbuffers::HashFnv1a<uint64_t> and schema_hash.h use.
inline constexpr std::uint64_t kFnvOffsetBasis = 0xcbf29ce484222645ULL;
inline constexpr std::uint64_t kFnvPrime = 0x100000001b3ULL;

constexpr std::uint64_t fnv1a64(std::string_view text,
                                std::uint64_t hash = kFnvOffsetBasis) {
  for (const char c : text) {
    hash ^= static_cast<std::uint8_t>(c);
    hash *= kFnvPrime;
  }
  return hash;
}

// `value` in decimal, continuing `hash`.
constexpr std::uint64_t fnv1a64_decimal(std::uint64_t value,
                                        std::uint64_t hash) {
  char digits[20]{};
  std::size_t count = 0;
  do {
    digits[count++] = static_cast<char>('0' + value % 10);
    value /= 10;
  } while (value != 0);
  while (count > 0) {
    hash = fnv1a64(std::string_view{&digits[--count], 1}, hash);
  }
  return hash;
}

// `value` as 16 lowercase hex digits, continuing `hash`.
constexpr std::uint64_t fnv1a64_hex(std::uint64_t value, std::uint64_t hash) {
  constexpr std::string_view kHex = "0123456789abcdef";
  for (int shift = 60; shift >= 0; shift -= 4) {
    hash = fnv1a64(kHex.substr((value >> shift) & 0xF, 1), hash);
  }
  return hash;
}

inline constexpr std::string_view kBytesName = "talos.Bytes";
inline constexpr std::string_view kPacketName = "talos.hardware.Packet";
inline constexpr std::string_view kEnvelopePrefix = "talos.TargetEnvelope<";

constexpr std::uint64_t BytesHash() { return fnv1a64(kBytesName); }

constexpr std::uint64_t PacketHash(std::size_t bytes) {
  return fnv1a64_decimal(bytes, fnv1a64(" ", fnv1a64(kPacketName)));
}

constexpr std::uint64_t EnvelopeHash(std::uint64_t inner) {
  return fnv1a64_hex(inner, fnv1a64("talos.TargetEnvelope "));
}

// "talos.TargetEnvelope<" + Descriptor<M>::kName + ">", with static storage.
template <class M>
struct EnvelopeName {
  static constexpr std::string_view kInner = Descriptor<M>::kName;
  static constexpr std::size_t kSize = kEnvelopePrefix.size() + kInner.size() + 1;
  static constexpr std::array<char, kSize> kStorage = [] {
    std::array<char, kSize> out{};
    std::size_t at = 0;
    for (const char c : kEnvelopePrefix) out[at++] = c;
    for (const char c : kInner) out[at++] = c;
    out[at] = '>';
    return out;
  }();
  static constexpr std::string_view kValue{kStorage.data(), kSize};
};

// The identity of a described type M.
template <Described M>
Identity Of() {
  const std::span<const std::uint8_t> bfbs = Descriptor<M>::kBfbs;
  return Identity{std::string{Descriptor<M>::kName}, Descriptor<M>::kHash,
                  std::string{reinterpret_cast<const char*>(bfbs.data()),
                              bfbs.size()}};
}

inline Identity Bytes() { return Identity{std::string{kBytesName}, BytesHash(), {}}; }

// A TargetEnvelope around `message` (talOS/devices/target.h).
inline Identity Envelope(const Identity& message) {
  return Identity{std::string{kEnvelopePrefix} + message.name + ">",
                  EnvelopeHash(message.hash), message.bfbs};
}

// The struct an envelope name wraps ("control.X" for
// "talos.TargetEnvelope<control.X>"), or empty when `name` is not one.
constexpr std::string_view EnvelopeInner(std::string_view name) {
  if (!name.starts_with(kEnvelopePrefix) || !name.ends_with('>')) return {};
  return name.substr(kEnvelopePrefix.size(),
                     name.size() - kEnvelopePrefix.size() - 1);
}

// "control.VelocityTarget (0x0123456789abcdef)", for error messages.
inline std::string Describe(std::string_view name, std::uint64_t hash) {
  constexpr std::string_view kHex = "0123456789abcdef";
  std::string out{name};
  out += " (0x";
  for (int shift = 60; shift >= 0; shift -= 4) {
    out += kHex[(hash >> shift) & 0xF];
  }
  out += ')';
  return out;
}

inline std::string Describe(const Identity& identity) {
  return Describe(identity.name, identity.hash);
}

}  // namespace talos::schema
