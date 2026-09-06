#pragma once

#include <cstdint>

#include "2026-robot/main_processor/driver_station/driver_station_message_generated.h"
#include "2026-robot/main_processor/driver_station/node.h"
#include "2026-robot/main_processor/drivetrain/drive_message_generated.h"
#include "2026-robot/main_processor/drivetrain/packet.h"
#include "2026-robot/main_processor/shooter/packet.h"
#include "2026-robot/main_processor/shooter/shooter_message_generated.h"
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

// The two `auto` topics have no publisher, and will not until autonomous is
// written. The subscriptions stay so the wiring is already right on the day it
// is, and main.cc declares both sources with naming::kSourceFlagOptional -- a
// graph that reports an unwritten feature the same way it reports a genuinely
// broken link teaches people to skip the report.
inline constexpr const char* kAutoChassisTopic = talos::drive::kAutoTargetTopic;
inline constexpr const char* kAutoShooterTopic =
    talos::shooter::kAutoShooterTargetTopic;

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

template <typename Loop>
class ArbiterNode {
 public:
  explicit ArbiterNode(Loop& loop)
      : chassis_{event::make_sender<talos::drive::ChassisTarget>(
            loop, talos::drive::kTargetTopic)},
        shooter_{event::make_sender<talos::shooter::ShooterTarget>(
            loop, talos::shooter::kShooterTargetTopic)} {
    event::watch<driver_station::DriverStationState, &ArbiterNode::OnDsState>(
        loop, driver_station::kDsStateTopic, this);
    event::watch<talos::drive::ChassisTarget, &ArbiterNode::OnTeleopChassis>(
        loop, kTeleopChassisTopic, this);
    event::watch<talos::drive::ChassisTarget, &ArbiterNode::OnAutoChassis>(
        loop, kAutoChassisTopic, this);
    event::watch<talos::shooter::ShooterTarget, &ArbiterNode::OnTeleopShooter>(
        loop, kTeleopShooterTopic, this);
    event::watch<talos::shooter::ShooterTarget, &ArbiterNode::OnAutoShooter>(
        loop, kAutoShooterTopic, this);
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
    chassis_.send(
        talos::drive::ChassisTarget{0.0, 0.0, 0.0, ctx.now.nanos(), false});
    shooter_.send(talos::shooter::ShooterTarget{0.0, ctx.now.nanos(), false});
  }

  void OnTeleopChassis(const event::Context&,
                       const talos::drive::ChassisTarget& target) {
    if (source_ == Source::kTeleop) chassis_.send(target);
  }
  void OnAutoChassis(const event::Context&,
                     const talos::drive::ChassisTarget& target) {
    if (source_ == Source::kAutonomous) chassis_.send(target);
  }
  void OnTeleopShooter(const event::Context&,
                       const talos::shooter::ShooterTarget& target) {
    if (source_ == Source::kTeleop) shooter_.send(target);
  }
  void OnAutoShooter(const event::Context&,
                     const talos::shooter::ShooterTarget& target) {
    if (source_ == Source::kAutonomous) shooter_.send(target);
  }

  Source source_{Source::kNone};
  event::Sender<Loop, talos::drive::ChassisTarget> chassis_;
  event::Sender<Loop, talos::shooter::ShooterTarget> shooter_;
};

}  // namespace talos::arbiter
