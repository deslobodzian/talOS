#pragma once

// Safe, non-privileged thunks behind the templates in talOS/devices/target.h
// (Target, TargetLane, OperatorArming). Privileged device plumbing
// (HardwareAccess, including note_live_target and device_id, and
// DriverStationAccess) lives in talOS/devices/access.h behind the restricted
// //talOS/devices:access target so robot node code cannot include or link it.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "talOS/schema/identity.h"

namespace talos {
class Node;
}  // namespace talos

namespace talos::detail {

// Target and TargetLane (target.h) over message bytes, so the C ABI drives
// the same code. Indexes address this node's targets and lanes.
struct TargetAccess {
  // Subscribes to every lane of the node's policy and the Driver Station.
  // `bytes` and `alignment` are the message's (sizeof, alignof) and
  // `message` its schema: the lanes carry schema::Envelope(message).
  static std::size_t open_target(Node& node, std::size_t bytes,
                                 std::size_t alignment,
                                 const schema::Identity& message);
  // The selected message (`bytes` long), or null; `source` is the selected
  // source's index in target_sources, or -1.
  static const void* target_get(Node& node, std::size_t target,
                                std::int32_t& source);
  // The policy's sources in subscription order.
  static std::span<const std::string> target_sources(Node& node,
                                                     std::size_t target);
  // The message size the target was opened with.
  static std::size_t target_bytes(Node& node, std::size_t target);

  // Subscribes to the Driver Station, whose generation lane_send stamps.
  // With `arming` (an index from open_arming on this node), lane_send
  // publishes a release instead while that arming is not armed.
  static std::size_t open_lane(Node& node, std::string_view subsystem,
                               std::string_view source, std::size_t bytes,
                               std::size_t alignment,
                               const schema::Identity& message,
                               std::optional<std::size_t> arming = {});
  // Whether the envelope went out active: false when the transport dropped
  // it or the lane's arming held it to a release.
  static bool lane_send(Node& node, std::size_t lane, const void* message,
                        std::size_t bytes);
  static bool lane_release(Node& node, std::size_t lane);

  // OperatorArming (target.h): subscribes to the Driver Station.
  static std::size_t open_arming(Node& node);
  // OperatorArming::evaluate, armed and disarm.
  static bool arming_evaluate(Node& node, std::size_t arming, bool neutral);
  static bool arming_armed(Node& node, std::size_t arming);
  static void arming_disarm(Node& node, std::size_t arming);
};

// sizeof(TargetEnvelope<M>) for a message of `bytes` and `alignment`: the
// 16-byte header plus `bytes`. Throws std::invalid_argument unless alignment
// is a power of two at most 8 that divides bytes.
std::size_t TargetEnvelopeBytes(std::size_t bytes, std::size_t alignment);
// Writes the envelope: the header (mode_since_us, active, zeros), then the
// message (zeros when `message` is null). `out` holds
// TargetEnvelopeBytes(bytes, alignment).
void EncodeTargetEnvelope(std::uint64_t mode_since_us, bool active,
                          const void* message, std::size_t bytes,
                          std::size_t alignment, std::span<std::byte> out);
// An envelope's header, as EncodeTargetEnvelope wrote it.
struct TargetEnvelopeHeader {
  std::uint64_t mode_since_us = 0;
  bool active = false;
};
TargetEnvelopeHeader DecodeTargetEnvelopeHeader(const std::byte* envelope);

}  // namespace talos::detail
