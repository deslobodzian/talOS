#pragma once

#include <cstdint>

#include "driver_station/driver_station_message_generated.h"
#include "driver_station/topics.h"
#include "drivetrain/drive_message_generated.h"
#include "drivetrain/topics.h"
#include "intake/intake_message_generated.h"
#include "intake/topics.h"
#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/node/node.h"

// Every target topic the drivetrain, shooter and intake consume has exactly
// one writer: this node. Producers publish on their own lanes and never
// coordinate, and the match mode -- which the FMS hands down, rather than
// anything the robot decides -- selects which producer reaches the
// actuators. One writer is what keeps the log replayable: two producers
// racing on one topic would resolve differently on replay than on the field.
namespace arbiter {

enum class Source : std::uint8_t { kNone, kTeleop, kAutonomous };

inline Source SourceFor(std::uint32_t fms_flags) {
  using namespace talos::driver_station;
  if ((fms_flags & kEnabled) == 0) return Source::kNone;
  if (fms_flags & kEStop) return Source::kNone;
  if (fms_flags & kAutonomous) return Source::kAutonomous;
  if (fms_flags & kTeleop) return Source::kTeleop;
  // Test mode and anything unrecognised drive nothing. A mode this node does
  // not understand is not a reason to pass a stale producer through.
  return Source::kNone;
}

// One actuator family: forwards whichever producer the source selects, as
// each message arrives, to the one topic the subsystem consumes.
template <talos::Message M>
class Lane {
 public:
  Lane(talos::Node& node, const Source& source, const char* topic,
       const char* teleop, const char* autonomous)
      : out_{node.publish<M>(topic)} {
    node.subscribe<M>(teleop, [this, &source](const M& message) {
      if (source == Source::kTeleop) out_.send(message);
    });
    // Autonomous is not written yet, so this lane has no publisher. That is
    // the design, not a broken link, and the graph lint is told so.
    node.subscribe<M>(
        autonomous,
        [this, &source](const M& message) {
          if (source == Source::kAutonomous) out_.send(message);
        },
        {.optional = true});
  }

  void Send(const M& message) { out_.send(message); }

 private:
  talos::Publisher<M> out_;
};

class Arbiter {
 public:
  explicit Arbiter(talos::Node& node)
      : node_{node},
        // Subscribed before the lanes, so a mode change and a producer's
        // target that arrive together are taken mode first: the driver
        // station state that enables teleop also admits the teleop target
        // the operator interface derived from it.
        mode_{node.subscribe<driver_station::DriverStationState>(
            driver_station::kStateTopic,
            [this](const driver_station::DriverStationState& state) {
              OnMode(SourceFor(state.fms().flags()));
            })},
        chassis_{node, source_, drivetrain::kTargetTopic,
                 drivetrain::kTeleopTargetTopic, drivetrain::kAutoTargetTopic},
        shooter_{node, source_, shooter::kTargetTopic,
                 shooter::kTeleopTargetTopic, shooter::kAutoTargetTopic},
        intake_{node, source_, intake::kTargetTopic, intake::kTeleopTargetTopic,
                intake::kAutoTargetTopic} {}

 private:
  void OnMode(Source next) {
    if (next == source_) return;
    source_ = next;
    // On every mode change, say "stop" once before the new producer speaks.
    // Otherwise the outgoing mode's last target stays live on the topic for
    // a command timeout: an autonomous request could still be driving the
    // robot after the switch to teleop.
    const std::int64_t now = node_.now().nanos();
    chassis_.Send(drivetrain::ChassisTarget{0, 0, 0, now, false});
    shooter_.Send(shooter::ShooterTarget{0, now, false});
    intake_.Send(
        intake::IntakeTarget{static_cast<std::uint64_t>(now), false, 0});
  }

  talos::Node& node_;
  Source source_ = Source::kNone;
  talos::Subscription<driver_station::DriverStationState> mode_;
  Lane<drivetrain::ChassisTarget> chassis_;
  Lane<shooter::ShooterTarget> shooter_;
  Lane<intake::IntakeTarget> intake_;
};

}  // namespace arbiter
