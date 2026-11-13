#pragma once

// Target lanes: what a subsystem is asked to do, from several producers at
// once, resolved inside the subsystem itself.
//
//   // in Wrist's constructor; lanes /wrist/target/<source> from the policy
//   talos::Target<control::PositionTarget> target_{node};
//   // in update(); motor_ needs a position frame (position_frame =
//   // "absolute" on a remote CANcoder, or a rezero first: see
//   // docs/NODE_DEVELOPMENT_GUIDE.md, "Rezero and position frames")
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
//   - carries the consumer's current mode generation,
// and only while robot_enabled(node). Otherwise it returns null: neutral.
//
// The mode generation is the invariant that a target issued under an earlier
// Driver Station enable or mode never actuates. The sampler on the far side of
// the link (the RoboRIO program, SimHardware) stamps every Driver Station
// packet with mode_since_us, the sample time of the first packet in the
// current mode (driver_station::ModeTracker); a disable, re-enable, mode
// switch, e-stop or re-attach starts a new one. TargetLane::send stamps the
// producer's own view of it into the envelope: the generation of the packet
// its node drained last, or 0 while that view is stale, disabled or in no
// mode. Target accepts an envelope only when that stamp is nonzero and equal
// to the generation of the packet the consumer drained last. Receipt time
// would not do: a target queued before a mode change and the packet that
// changed the mode can land in one drain and share a timestamp. A Driver
// Station that drops and comes back in the same mode keeps its generation, so
// a producer that kept talking drives again as soon as its lane is fresh.
//
// Target subscribes without a callback, so the node keeps draining right
// before its timer; selection runs on every get().
//
// The generation keeps an old target from driving a new enable; it does not
// stop a producer from turning a joystick held through that enable into a
// new target. A producer that maps operator input (a joystick, a button
// board) therefore opens its lanes with an OperatorArming, and evaluates it
// on every reading of those inputs:
//
//   talos::OperatorArming arming_{node};
//   talos::TargetLane<climber::ClimbTarget> climb_{node, "climber",
//                                                  "operator", arming_};
//   // in update(), or wherever the inputs are read:
//   const bool held = ds_.joystick(1).button(3);
//   if (arming_.evaluate(/*neutral=*/!held) && held) {
//     climb_.send(climber::ClimbTarget{...});
//   } else {
//     climb_.release();
//   }
//
// The lane stays released until an evaluation has seen every input neutral
// in the current Driver Station view; a TargetLane without an
// OperatorArming is never held.

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

#include "talOS/devices/access.h"
#include "talOS/node/node.h"

namespace talos {

// What a lane carries: a 16-byte header {uint64 mode_since_us; uint8 active;
// 7 bytes zero}, then M at offset 16; 16 + sizeof(M) bytes. The generation is
// held as bytes so the envelope keeps M's alignment: a 4-byte M makes a
// 20-byte envelope, as the lane topic is registered.
template <Message M>
struct TargetEnvelope {
  // The producer's Driver Station mode generation, native byte order: 0
  // when it had none.
  std::array<std::byte, 8> mode_since_us_bytes{};
  // 0: this producer has nothing to say.
  std::uint8_t active{0};
  std::array<std::uint8_t, 7> padding{};
  M message{};

  std::uint64_t mode_since_us() const {
    return std::bit_cast<std::uint64_t>(mode_since_us_bytes);
  }
  void set_mode_since_us(std::uint64_t us) {
    mode_since_us_bytes = std::bit_cast<std::array<std::byte, 8>>(us);
  }
};

// A lane's schema identity (talOS/schema/identity.h): talos.TargetEnvelope<M>
// with M's schema bytes, so a lane of another message type is refused at
// attach and log_dump decodes the header and then M.
template <Message M>
struct Descriptor<TargetEnvelope<M>> {
  static constexpr std::string_view kName = schema::EnvelopeName<M>::kValue;
  static constexpr std::uint64_t kHash =
      schema::EnvelopeHash(Descriptor<M>::kHash);
  static constexpr std::span<const std::uint8_t> kBfbs = Descriptor<M>::kBfbs;
};

namespace detail {
// The layout talOS/schema/identity.h documents for log_dump.
inline constexpr std::size_t kTargetEnvelopeHeaderBytes =
    schema::kEnvelopeHeaderBytes;
// What TargetAccess assumes of the envelope, checked by Target and
// TargetLane for each M.
template <Message M>
inline constexpr bool kTargetEnvelopeLayout =
    sizeof(TargetEnvelope<M>) == kTargetEnvelopeHeaderBytes + sizeof(M) &&
    alignof(TargetEnvelope<M>) == alignof(M) &&
    offsetof(TargetEnvelope<M>, active) == schema::kEnvelopeActiveOffset &&
    offsetof(TargetEnvelope<M>, message) == kTargetEnvelopeHeaderBytes;
}  // namespace detail

// The lanes of this node's target policy, resolved: see the top of this
// file. A cheap copyable value bound to its node, which is why there is no
// default constructor; take it in the node's constructor.
template <Message M>
class Target {
  static_assert(detail::kTargetEnvelopeLayout<M>);

 public:
  explicit Target(Node& node)
      : node_{&node},
        index_{detail::TargetAccess::open_target(node, sizeof(M), alignof(M),
                                                 schema::Of<M>())} {}

  // The selected lane's message, or null: robot disabled, Driver Station
  // stale or in a mode with no live lane of this generation.
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

template <Message M>
class TargetLane;

// Whether an operator-input producer may publish active targets: armed by
// an evaluation that found every input neutral, and only for the Driver
// Station view it was armed under. It is disarmed, and the next neutral
// evaluation is needed again,
//   - at construction (startup),
//   - when the Driver Station shows no mode (stale, detached, disabled or
//     e-stopped), a new mode generation or a new gateway boot -- so after a
//     disable or mode switch however brief, even one no evaluation sampled,
//   - after the link recovers from a stale spell (a packet arriving
//     command_timeout_us or more after the one before), sampled or not,
//   - when evaluations stop for command_timeout_us or more (a stalled or
//     paused process), and
//   - by disarm(), for producer-specific reasons (a joystick reconnecting).
// A cheap copyable value bound to its node, like Target and TargetLane.
class OperatorArming {
 public:
  explicit OperatorArming(Node& node)
      : node_{&node}, index_{detail::TargetAccess::open_arming(node)} {}

  // One reading of the producer's inputs; `neutral`: every input it maps is
  // at rest (axes inside their deadband, buttons and POVs released). True
  // when the producer may publish active targets on this reading: it was
  // already armed and nothing above has disarmed it. Otherwise false, and
  // a neutral reading while the Driver Station shows a mode arms it at once
  // (send() on this reading is active; the reading itself was neutral).
  // Call it on every reading, on the loop thread:
  // gaps of command_timeout_us disarm.
  bool evaluate(bool neutral) {
    return detail::TargetAccess::arming_evaluate(*node_, index_, neutral);
  }
  // Whether it is armed now, without evaluating: what a lane opened with it
  // checks on every send().
  bool armed() const {
    return detail::TargetAccess::arming_armed(*node_, index_);
  }
  void disarm() { detail::TargetAccess::arming_disarm(*node_, index_); }

 private:
  template <Message>
  friend class TargetLane;
  Node* node_{nullptr};
  std::size_t index_{0};
};

// One producer's lane, /<subsystem>/target/<source>. Takes the node's
// DriverStation, whose generation every send() stamps. Like Target, always
// bound to its node: there is no default constructor.
template <Message M>
class TargetLane {
  static_assert(detail::kTargetEnvelopeLayout<M>);

 public:
  TargetLane(Node& node, std::string_view subsystem, std::string_view source)
      : node_{&node},
        index_{detail::TargetAccess::open_lane(
            node, subsystem, source, sizeof(M), alignof(M), schema::Of<M>())} {}
  // A lane of an operator-input producer: send() publishes a release
  // instead while `arming` (of this node) is not armed().
  TargetLane(Node& node, std::string_view subsystem, std::string_view source,
             const OperatorArming& arming)
      : node_{&node},
        index_{detail::TargetAccess::open_lane(
            node, subsystem, source, sizeof(M), alignof(M), schema::Of<M>(),
            SameNode(node, arming))} {}

  // Asks for `message` (active = 1) under the mode generation this node's
  // Driver Station shows now (0, which no consumer accepts, while it shows
  // none). Republish every period: a lane older than [robot]
  // command_timeout_us is stale, and one from an earlier generation is
  // ignored. False when the transport dropped it, or when the lane's
  // OperatorArming is not armed and it went out as a release.
  bool send(const M& message) {
    return detail::TargetAccess::lane_send(*node_, index_, &message, sizeof(M));
  }
  // Nothing to say (active = 0): the consumer falls through to its next
  // lane at once.
  bool release() { return detail::TargetAccess::lane_release(*node_, index_); }

 private:
  static std::size_t SameNode(const Node& node, const OperatorArming& arming) {
    if (arming.node_ != &node) {
      throw std::invalid_argument(
          std::string{node.name()} +
          ": a target lane's OperatorArming belongs to another node");
    }
    return arming.index_;
  }

  Node* node_{nullptr};
  std::size_t index_{0};
};

}  // namespace talos
