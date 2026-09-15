#pragma once

#include <cstdint>
#include <utility>

#include "talos_nodes/driver_station/driver_station_message_generated.h"
#include "talos_nodes/driver_station/packet.h"
#include "talos_nodes/drivetrain/drive_message_generated.h"
#include "talos_nodes/drivetrain/packet.h"
#include "talos_nodes/intake/intake_message_generated.h"
#include "talos_nodes/intake/packet.h"
#include "talos_nodes/shooter/packet.h"
#include "talos_nodes/shooter/shooter_message_generated.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/handles.h"

// Every target topic the drivetrain and shooter consume has exactly one writer:
// this node. Producers publish to their own topic and never coordinate, and the
// match mode -- which the FMS hands down, rather than anything the robot
// decides -- selects which producer reaches the actuators. Keeping it to one
// writer is what keeps the dispatch log replayable: two producers racing on one
// topic would resolve differently on replay than they did on the field.
namespace talos::arbiter {

// Aliases for the topics the subsystem packages declare, so the arbiter and the
// producers cannot drift onto differently spelled strings.
inline constexpr const char* kTeleopChassisTopic =
    talos::drive::kTeleopTargetTopic;
inline constexpr const char* kTeleopShooterTopic =
    talos::shooter::kTeleopShooterTargetTopic;
inline constexpr const char* kTeleopIntakeTopic =
    talos::intake::kTeleopIntakeTargetTopic;

// The two `auto` topics have no publisher, and will not until autonomous is
// written. The subscriptions stay so the wiring is already right on the day it
// is, and main.cc declares both sources with naming::kSourceFlagOptional -- a
// graph that reports an unwritten feature the same way it reports a genuinely
// broken link teaches people to skip the report.
inline constexpr const char* kAutoChassisTopic = talos::drive::kAutoTargetTopic;
inline constexpr const char* kAutoShooterTopic =
    talos::shooter::kAutoShooterTargetTopic;
inline constexpr const char* kAutoIntakeTopic =
    talos::intake::kAutoIntakeTargetTopic;

enum class Source : uint8_t { kNone, kTeleop, kAutonomous };

inline Source SourceFor(uint32_t fms_flags) {
  using namespace talos::driver_station;
  if ((fms_flags & kEnabled) == 0) return Source::kNone;
  if (fms_flags & kEStop) return Source::kNone;
  if (fms_flags & kAutonomous) return Source::kAutonomous;
  if (fms_flags & kTeleop) return Source::kTeleop;
  // Test mode and anything unrecognised drive nothing. A mode this node does
  // not understand is not a reason to pass a stale producer through.
  return Source::kNone;
}

// One arbitration lane: the gated forwarder from a producer pair (teleop,
// autonomous) to the single topic the subsystem consumes. The arbiter holds
// one lane per actuator family, so a new family is a member plus one Watch
// call rather than a new pair of near-identical handlers. Lanes are built in
// two phases — senders in the member initializer list, watchers in the
// constructor body — because registration order defines source ids: all
// senders must precede all watchers exactly as they do today.
template <typename Loop, typename Message>
class ArbitratedLane {
 public:
  ArbitratedLane(event::Sender<Loop, Message> out, const Source* source)
      : out_(std::move(out)), source_(source) {}

  void Watch(Loop& loop, const char* teleop_topic,
             const char* autonomous_topic) {
    event::watch<Message, &ArbitratedLane::OnTeleop>(loop, teleop_topic, this);
    event::watch<Message, &ArbitratedLane::OnAutonomous>(loop, autonomous_topic,
                                                         this);
  }

  void Publish(const Message& message) { out_.send(message); }

 private:
  void OnTeleop(const event::Context&, const Message& message) {
    if (*source_ == Source::kTeleop) out_.send(message);
  }
  void OnAutonomous(const event::Context&, const Message& message) {
    if (*source_ == Source::kAutonomous) out_.send(message);
  }

  event::Sender<Loop, Message> out_;
  const Source* source_;
};

template <typename Loop>
class ArbiterNode {
 public:
  explicit ArbiterNode(Loop& loop)
      : chassis_{event::make_sender<talos::drive::ChassisTarget>(
                     loop, talos::drive::kTargetTopic),
                 &source_},
        shooter_{event::make_sender<talos::shooter::ShooterTarget>(
                     loop, talos::shooter::kShooterTargetTopic),
                 &source_},
        intake_{event::make_sender<talos::intake::IntakeTarget>(
                    loop, talos::intake::kTargetTopic),
                &source_} {
    event::watch<driver_station::DriverStationState, &ArbiterNode::OnDsState>(
        loop, driver_station::kDsStateTopic, this);
    chassis_.Watch(loop, kTeleopChassisTopic, kAutoChassisTopic);
    shooter_.Watch(loop, kTeleopShooterTopic, kAutoShooterTopic);
    intake_.Watch(loop, kTeleopIntakeTopic, kAutoIntakeTopic);
  }

  void Start(event::MonotonicTime = {}) {}

  Source source() const { return source_; }

 private:
  void OnDsState(const event::Context& ctx,
                 const driver_station::DriverStationState& state) {
    const Source next = SourceFor(state.fms().flags());
    if (next == source_) return;

    // On every mode change, say "stop" once before the new producer speaks.
    // Without it the last target of the outgoing mode stays live on the topic
    // for a command timeout, so an autonomous request could still be driving
    // the robot after the switch to teleop.
    source_ = next;
    chassis_.Publish(
        talos::drive::ChassisTarget{0.0, 0.0, 0.0, ctx.now.nanos(), false});
    shooter_.Publish(
        talos::shooter::ShooterTarget{0.0, ctx.now.nanos(), false});
    intake_.Publish(talos::intake::IntakeTarget{
        static_cast<uint64_t>(ctx.now.nanos()), false, 0.0f});
  }

  Source source_{Source::kNone};
  ArbitratedLane<Loop, talos::drive::ChassisTarget> chassis_;
  ArbitratedLane<Loop, talos::shooter::ShooterTarget> shooter_;
  ArbitratedLane<Loop, talos::intake::IntakeTarget> intake_;
};

}  // namespace talos::arbiter
