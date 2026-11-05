#pragma once

// One message's bytes as text, from the topic's schema identity
// (talOS/schema/identity.h) alone: what log_dump prints for every record.
//
//   a talos_messages struct    {position_m: 1.5, mode: TRACKING, gains: [1, 2]}
//                              field names in declaration order, read with
//                              flatbuffers/reflection.h from the identity's
//                              bfbs; enums print their value's name
//   talos.TargetEnvelope<M>    {mode_since_us: 1200, active: true,
//                               message: {...M...}}
//   a layout with no bfbs      bytes=1436 hex=0a01ff... (talos.Bytes,
//   (byte topics, the          talos.hardware.Packet): the payload without
//   hardware packet)           its trailing zeros, the first kMaxHexBytes
//                              of it in hex
//
// Never throws: a payload of the wrong size, or a bfbs that does not name
// the struct, prints why in place of the fields.

#include <cstddef>
#include <span>
#include <string>

#include "talOS/schema/identity.h"

namespace talos::schema {

inline constexpr std::size_t kMaxHexBytes = 64;

std::string DecodeToText(const Identity& schema,
                         std::span<const std::byte> payload);

}  // namespace talos::schema
