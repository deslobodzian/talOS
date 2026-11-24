#include "talOS/devices/target.h"

#include <algorithm>
#include <cstring>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/devices/access.h"
#include "talOS/devices/devices.h"
#include "talOS/introspection/names.h"
#include "talOS/node/access.h"

namespace talos::detail {
namespace {

// One Target: a subscription per policy source, and each mode's lanes as
// indexes into them in priority order.
struct TargetState {
  std::size_t bytes = 0;
  std::size_t alignment = 0;
  std::vector<std::string> sources;
  std::vector<std::size_t> subscriptions;  // per source
  std::vector<std::int32_t> teleop;
  std::vector<std::int32_t> autonomous;
  std::vector<std::int32_t> test;
  DriverStation ds;
  std::size_t selection = 0;  // NodeAccess::add_selection: who drives
  // NodeAccess::next_update of the update() that last logged a READ (0:
  // none; inside update() it is at least 1).
  std::uint64_t read_update = 0;
};

struct LaneState {
  std::size_t publisher = 0;
  std::size_t bytes = 0;
  std::size_t alignment = 0;
  DriverStation ds;                   // whose generation send() stamps
  std::vector<std::byte> envelope;    // reused for every send
  std::optional<std::size_t> arming;  // holds send() to a release
};

// One OperatorArming: armed by a neutral evaluation, and only for the
// Driver Station view it was armed under.
struct ArmingState {
  DriverStation ds;
  bool armed = false;
  std::uint64_t boot_id = 0;
  std::uint64_t generation = 0;
  std::uint64_t recoveries = 0;
  std::optional<Time> last_evaluation;
  ArmingBlocker blocker{};
};

// This node's targets, lanes and armings, attached as a Node extension.
// Deques keep each entry where it is as more are opened.
struct Targets {
  explicit Targets(Node&) {}
  std::deque<TargetState> targets;
  std::deque<LaneState> lanes;
  std::deque<ArmingState> armings;
};

Targets* FindTargets(Node& node) {
  return static_cast<Targets*>(
      NodeAccess::find_extension(node, typeid(Targets)));
}

TargetState& TargetAt(Node& node, std::size_t index) {
  Targets* all = FindTargets(node);
  if (all == nullptr || index >= all->targets.size()) {
    throw std::out_of_range(std::string{node.name()} + ": no target at index " +
                            std::to_string(index));
  }
  return all->targets[index];
}

LaneState& LaneAt(Node& node, std::size_t index) {
  Targets* all = FindTargets(node);
  if (all == nullptr || index >= all->lanes.size()) {
    throw std::out_of_range(std::string{node.name()} +
                            ": no target lane at index " +
                            std::to_string(index));
  }
  return all->lanes[index];
}

ArmingState& ArmingAt(Node& node, std::size_t index) {
  Targets* all = FindTargets(node);
  if (all == nullptr || index >= all->armings.size()) {
    throw std::out_of_range(std::string{node.name()} +
                            ": no operator arming at index " +
                            std::to_string(index));
  }
  return all->armings[index];
}

// Armed, and nothing has happened since that the arming evaluation did not
// see: an evaluation within command_timeout_us, and the same Driver Station
// mode generation and gateway boot, with no stale spell of the link between.
bool Armed(Node& node, const ArmingState& state) {
  if (!state.armed || !state.last_evaluation) return false;
  if (node.now() - *state.last_evaluation >=
      NodeAccess::freshness_timeout(node)) {
    return false;
  }
  const std::uint64_t generation = DriverStationAccess::generation(state.ds);
  return generation != 0 && generation == state.generation &&
         state.ds.data().boot_id == state.boot_id &&
         DriverStationAccess::recoveries(state.ds) == state.recoveries;
}

// Hands whatever target_get decided -- on every path out of it -- to the
// node's registry row, so a viewer can see who drives the subsystem.
class ReportSelection {
 public:
  ReportSelection(Node& node, std::size_t selection, const std::int32_t& source)
      : node_{node}, selection_{selection}, source_{source} {}
  ~ReportSelection() { NodeAccess::set_selected(node_, selection_, source_); }
  ReportSelection(const ReportSelection&) = delete;
  ReportSelection& operator=(const ReportSelection&) = delete;

 private:
  Node& node_;
  std::size_t selection_;
  const std::int32_t& source_;
};

}  // namespace

std::size_t TargetEnvelopeBytes(std::size_t bytes, std::size_t alignment) {
  constexpr std::size_t kMaxPayloadBytes =
      std::numeric_limits<std::uint32_t>::max() - kTargetEnvelopeHeaderBytes;
  if (alignment == 0 || (alignment & (alignment - 1)) != 0 ||
      alignment > kMessageAlignment || bytes == 0 || bytes % alignment != 0 ||
      bytes > kMaxPayloadBytes) {
    throw std::invalid_argument(
        "target message of " + std::to_string(bytes) + " bytes aligned to " +
        std::to_string(alignment) +
        ": the alignment must be a power of two at most " +
        std::to_string(kMessageAlignment) +
        " that divides the size and fits in a target envelope");
  }
  return kTargetEnvelopeHeaderBytes + bytes;
}

void EncodeTargetEnvelope(std::uint64_t mode_since_us, bool active,
                          const void* message, std::size_t bytes,
                          std::size_t alignment, std::span<std::byte> out) {
  if (out.size() != TargetEnvelopeBytes(bytes, alignment)) {
    throw std::invalid_argument("target envelope buffer has the wrong size");
  }
  std::fill(out.begin(), out.end(), std::byte{0});
  std::memcpy(out.data(), &mode_since_us, sizeof mode_since_us);
  out[8] = std::byte{active ? std::uint8_t{1} : std::uint8_t{0}};
  if (message != nullptr) {
    std::memcpy(out.data() + kTargetEnvelopeHeaderBytes, message, bytes);
  }
}

TargetEnvelopeHeader DecodeTargetEnvelopeHeader(const std::byte* envelope) {
  TargetEnvelopeHeader header;
  std::memcpy(&header.mode_since_us, envelope, sizeof header.mode_since_us);
  header.active = envelope[8] == std::byte{1};
  return header;
}

std::size_t TargetAccess::open_target(Node& node, std::size_t bytes,
                                      std::size_t alignment,
                                      const schema::Identity& message) {
  const std::size_t envelope = TargetEnvelopeBytes(bytes, alignment);
  const schema::Identity lane = schema::Envelope(message);
  const config::TargetPolicy& policy =
      NodeAccess::robot_config(node).TargetsFor(node.name());
  std::vector<std::string> sources = policy.Sources();
  // Register the selection first: add_selection checks that the node is still
  // in construction before attaching extensions or subscribing to lanes.
  const std::size_t selection = NodeAccess::add_selection(node, sources);
  // Attaches the hardware link (robot_enabled) and the Driver Station in the
  // constructor, where registration belongs.
  (void)node.extension<HardwareLink>();
  TargetState state{.bytes = bytes,
                    .alignment = alignment,
                    .sources = std::move(sources),
                    .ds = DriverStationAccess::internal(node),
                    .selection = selection};
  for (const std::string& source : state.sources) {
    // Relative, so [subsystems.<name>.remap] applies. Optional: a lane with
    // no producer is normal (the graph lint warns), and no callback keeps
    // the node draining right before its timer.
    state.subscriptions.push_back(NodeAccess::add_subscription(
        node, "target/" + source, envelope, kMessageAlignment, lane,
        /*optional=*/true, {}));
  }
  const auto indexes = [&state](const std::vector<std::string>& mode) {
    std::vector<std::int32_t> out;
    out.reserve(mode.size());
    for (const std::string& source : mode) {
      out.push_back(static_cast<std::int32_t>(
          std::find(state.sources.begin(), state.sources.end(), source) -
          state.sources.begin()));
    }
    return out;
  };
  state.teleop = indexes(policy.teleop);
  state.autonomous = indexes(policy.autonomous);
  state.test = indexes(policy.test);
  auto& targets = node.extension<Targets>().targets;
  targets.push_back(std::move(state));
  return targets.size() - 1;
}

const void* TargetAccess::target_get(Node& node, std::size_t target,
                                     std::int32_t& source) {
  NodeAccess::require_loop_thread(node, "Target::get");
  source = -1;
  TargetState& state = TargetAt(node, target);
  const ReportSelection report{node, state.selection, source};
  if (!robot_enabled(node)) return nullptr;
  // Reject cross-boot actuation during a gateway reboot window where the new
  // boot's State arrives before its first Driver Station packet (REQ-SAFE-07,
  // REQ-SAFE-11, REQ-SAFE-13). ds_boot == 0 (send_joystick) stays allowed.
  const std::uint64_t hw_boot =
      HardwareAccess::boot_id(node.extension<HardwareLink>());
  const std::uint64_t ds_boot = state.ds.data().boot_id;
  if (hw_boot != 0 && ds_boot != 0 && hw_boot != ds_boot) return nullptr;
  const std::vector<std::int32_t>* lanes = nullptr;
  switch (DriverStationAccess::mode(state.ds)) {
    case DsMode::kTeleop:
      lanes = &state.teleop;
      break;
    case DsMode::kAutonomous:
      lanes = &state.autonomous;
      break;
    case DsMode::kTest:
      lanes = &state.test;
      break;
    case DsMode::kNone:
      return nullptr;
  }
  // Zero while the consumer's own Driver Station shows no mode; the switch
  // above has already returned then.
  const std::uint64_t generation = DriverStationAccess::generation(state.ds);
  const Duration timeout = NodeAccess::freshness_timeout(node);
  for (const std::int32_t lane : *lanes) {
    const std::size_t subscription =
        state.subscriptions[static_cast<std::size_t>(lane)];
    if (!NodeAccess::subscription_has_value(node, subscription)) continue;
    if (NodeAccess::subscription_age(node, subscription) >= timeout) continue;
    const auto* envelope = static_cast<const std::byte*>(
        NodeAccess::subscription_value(node, subscription));
    const TargetEnvelopeHeader header = DecodeTargetEnvelopeHeader(envelope);
    if (!header.active) continue;  // released
    // Issued under another enable or mode (or with no Driver Station view
    // at all): a word meant for a different generation.
    if (header.mode_since_us == 0 || header.mode_since_us != generation) {
      continue;
    }
    source = lane;
    // A selection inside update() is what that tick acts on, through its
    // outputs or a command topic: the dispatch log records which message it
    // was (an EventKind::READ record), which is how a session's tick-time
    // target age is measured. Only the first selecting get() or source() of
    // each update() logs one, so a tick is one READ however often it asks.
    // Every selecting call inside update() also tells the devices flush that
    // this tick has a live target (RULES.md 1); a selection from the
    // constructor, every() or a callback does not.
    if (NodeAccess::in_update(node)) {
      HardwareAccess::note_live_target(node.extension<HardwareLink>());
      if (state.read_update != NodeAccess::next_update(node)) {
        state.read_update = NodeAccess::next_update(node);
        NodeAccess::record_read(node, subscription);
      }
    }
    return envelope + kTargetEnvelopeHeaderBytes;
  }
  return nullptr;
}

std::span<const std::string> TargetAccess::target_sources(Node& node,
                                                          std::size_t target) {
  NodeAccess::require_loop_thread(node, "Target::source");
  return TargetAt(node, target).sources;
}

std::size_t TargetAccess::target_bytes(Node& node, std::size_t target) {
  NodeAccess::require_loop_thread(node, "Target::bytes");
  return TargetAt(node, target).bytes;
}

std::size_t TargetAccess::open_lane(Node& node, std::string_view subsystem,
                                    std::string_view source, std::size_t bytes,
                                    std::size_t alignment,
                                    const schema::Identity& message,
                                    std::optional<std::size_t> arming) {
  NodeAccess::require_loop_thread(node, "TargetLane");
  const std::size_t envelope = TargetEnvelopeBytes(bytes, alignment);
  if (arming) ArmingAt(node, *arming);  // throws unless this node has it
  if (auto problem = introspect::naming::CheckTargetLane(subsystem, source)) {
    throw std::invalid_argument(std::string{node.name()} + ": " + *problem);
  }
  const std::string topic =
      introspect::naming::TargetLaneTopic(subsystem, source);
  // Register the publisher and Driver Station before attaching Targets so a
  // post-start or arming refusal leaves node extensions untouched.
  const std::size_t publisher = NodeAccess::add_target_lane_publisher(
      node, topic, envelope, kMessageAlignment, schema::Envelope(message),
      /*operator_arming=*/arming.has_value());
  DriverStation ds = DriverStationAccess::internal(node);
  auto& lanes = node.extension<Targets>().lanes;
  lanes.push_back(LaneState{
      .publisher = publisher,
      .bytes = bytes,
      .alignment = alignment,
      // Subscribed here, so the producer drains the Driver Station right
      // before its timer and send() stamps the packet it saw last.
      .ds = ds,
      .envelope = std::vector<std::byte>(envelope),
      .arming = arming,
  });
  return lanes.size() - 1;
}

bool TargetAccess::lane_send(Node& node, std::size_t lane, const void* message,
                             std::size_t bytes) {
  NodeAccess::require_loop_thread(node, "TargetLane::send");
  LaneState& state = LaneAt(node, lane);
  if (message == nullptr || bytes != state.bytes) {
    throw std::invalid_argument(
        std::string{node.name()} + ": a target lane of " +
        std::to_string(state.bytes) + "-byte messages was sent " +
        (message == nullptr ? std::string{"no message"}
                            : std::to_string(bytes) + " bytes"));
  }
  // Not armed: the operator's inputs have not been neutral since whatever
  // last disarmed it, so the lane says nothing rather than what they say.
  const bool active =
      !state.arming || Armed(node, ArmingAt(node, *state.arming));
  EncodeTargetEnvelope(DriverStationAccess::generation(state.ds), active,
                       active ? message : nullptr, state.bytes, state.alignment,
                       state.envelope);
  return NodeAccess::publisher_send(node, state.publisher,
                                    state.envelope.data(),
                                    state.envelope.size()) &&
         active;
}

bool TargetAccess::lane_release(Node& node, std::size_t lane) {
  NodeAccess::require_loop_thread(node, "TargetLane::release");
  LaneState& state = LaneAt(node, lane);
  EncodeTargetEnvelope(DriverStationAccess::generation(state.ds), false,
                       nullptr, state.bytes, state.alignment, state.envelope);
  return NodeAccess::publisher_send(
      node, state.publisher, state.envelope.data(), state.envelope.size());
}

std::size_t TargetAccess::open_arming(Node& node) {
  NodeAccess::require_loop_thread(node, "OperatorArming");
  // Refuse post-start registration without mutating node state:
  // Node::after_update checks `started` (throwing std::logic_error) before
  // rejecting an empty callback (throwing std::invalid_argument).
  try {
    node.after_update({});
  } catch (const std::invalid_argument&) {
    // Still in construction.
  } catch (const std::logic_error&) {
    throw std::logic_error(
        std::string{node.name()} +
        ": OperatorArming must be opened in the constructor");
  }
  DriverStation ds = DriverStationAccess::internal(node);
  auto& armings = node.extension<Targets>().armings;
  armings.push_back(ArmingState{.ds = ds});
  return armings.size() - 1;
}

bool TargetAccess::arming_evaluate(Node& node, std::size_t arming,
                                   bool neutral) {
  return arming_evaluate(
      node, arming, neutral,
      neutral ? ArmingBlocker{} : ArmingBlocker{ArmingInputKind::kOther});
}

bool TargetAccess::arming_evaluate(Node& node, std::size_t arming, bool neutral,
                                   ArmingBlocker blocker) {
  NodeAccess::require_loop_thread(node, "OperatorArming::evaluate");
  ArmingState& state = ArmingAt(node, arming);
  const bool armed = Armed(node, state);
  state.last_evaluation = node.now();
  if (armed) {
    state.blocker = {};
    return true;
  }
  // Arms for the evaluations after this one, under the view seen now.
  const std::uint64_t generation = DriverStationAccess::generation(state.ds);
  state.armed = neutral && generation != 0;
  state.generation = generation;
  state.boot_id = state.ds.data().boot_id;
  state.recoveries = DriverStationAccess::recoveries(state.ds);
  state.blocker = neutral ? ArmingBlocker{} : blocker;
  return false;
}

bool TargetAccess::arming_armed(Node& node, std::size_t arming) {
  NodeAccess::require_loop_thread(node, "OperatorArming::armed");
  return Armed(node, ArmingAt(node, arming));
}

ArmingBlocker TargetAccess::arming_blocker(Node& node, std::size_t arming) {
  NodeAccess::require_loop_thread(node, "OperatorArming::blocker");
  const ArmingState& state = ArmingAt(node, arming);
  return Armed(node, state) ? ArmingBlocker{} : state.blocker;
}

void TargetAccess::arming_disarm(Node& node, std::size_t arming) {
  NodeAccess::require_loop_thread(node, "OperatorArming::disarm");
  ArmingState& state = ArmingAt(node, arming);
  state.armed = false;
  state.blocker = {};
}

}  // namespace talos::detail
