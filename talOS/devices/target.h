#pragma once

// Target lanes: what a subsystem is asked to do, from several producers at
// once, resolved inside the subsystem itself.
//
//   // in Wrist's constructor; lanes /wrist/target/<source> from the policy
//   talos::Target<control::PositionTarget> target_{node};
//   // in update()
//   if (const control::PositionTarget* t = target_.get()) {
//     motor_.set_motion_magic(t->position_rot());
//   }
//
//   // in a producer (auto routine, auto-align, operator interface)
//   talos::TargetLane<control::PositionTarget> lane_{node, "wrist", "auto"};
//   lane_.send(control::PositionTarget{0.25, 0.0});
//   lane_.release();  // nothing to say: the next lane in priority order wins
//
// Each producer writes its own lane, /<sub>/target/<source>, carrying a
// TargetEnvelope<M>. The policy (which lanes count in which Driver Station
// mode, highest priority first) is config: [robot.targets] for every
// subsystem, a subsystem's [targets] for itself. Target<M>::get() returns the
// message of the first lane in the current mode's list whose latest envelope
//   - is active (the producer did not release it),
//   - is fresh (younger than [robot] command_timeout_us), and
//   - arrived at or after the mode began (the first Driver Station packet in
//     it), so a lane from an earlier mode never drives this one,
// and only while robot_enabled(node). Otherwise it returns null: neutral.
// Target subscribes without a callback, so the node keeps draining right
// before its timer; selection runs on every get().

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "talOS/devices/access.h"
#include "talOS/node/node.h"

namespace talos {

// What a lane carries: active (0: this producer has nothing to say), padding
// to alignof(M), then M; alignof(M) + sizeof(M) bytes.
template <Message M>
struct TargetEnvelope {
  std::uint8_t active{0};
  M message{};
};

namespace detail {
// What TargetAccess assumes of the envelope, checked by Target and
// TargetLane for each M.
template <Message M>
inline constexpr bool kTargetEnvelopeLayout =
    sizeof(TargetEnvelope<M>) == alignof(M) + sizeof(M) &&
    offsetof(TargetEnvelope<M>, message) == alignof(M);
}  // namespace detail

// The lanes of this node's target policy, resolved: see the top of this
// file. A cheap copyable value; take it in the constructor.
template <Message M>
class Target {
  static_assert(detail::kTargetEnvelopeLayout<M>);

 public:
  Target() = default;
  explicit Target(Node& node)
      : node_{&node},
        index_{detail::TargetAccess::open_target(node, sizeof(M), alignof(M))} {
  }

  // The selected lane's message, or null: robot disabled, Driver Station
  // stale or in a mode with no live lane.
  const M* get() const {
    std::int32_t source = -1;
    return static_cast<const M*>(
        detail::TargetAccess::target_get(*node_, index_, source));
  }
  // The selected lane's source ("teleop", "auto", ...), or "" when get()
  // is null.
  std::string_view source() const {
    std::int32_t source = -1;
    detail::TargetAccess::target_get(*node_, index_, source);
    if (source < 0) return {};
    return detail::TargetAccess::target_sources(
        *node_, index_)[static_cast<std::size_t>(source)];
  }

 private:
  Node* node_{nullptr};
  std::size_t index_{0};
};

// One producer's lane, /<subsystem>/target/<source>.
template <Message M>
class TargetLane {
  static_assert(detail::kTargetEnvelopeLayout<M>);

 public:
  TargetLane() = default;
  TargetLane(Node& node, std::string_view subsystem, std::string_view source)
      : node_{&node},
        index_{detail::TargetAccess::open_lane(node, subsystem, source,
                                               sizeof(M), alignof(M))} {}

  // Asks for `message` (active = 1). Republish every period: a lane older
  // than [robot] command_timeout_us is stale.
  bool send(const M& message) {
    return detail::TargetAccess::lane_send(*node_, index_, &message, sizeof(M));
  }
  // Nothing to say (active = 0): the consumer falls through to its next
  // lane at once.
  bool release() { return detail::TargetAccess::lane_release(*node_, index_); }

 private:
  Node* node_{nullptr};
  std::size_t index_{0};
};

}  // namespace talos
