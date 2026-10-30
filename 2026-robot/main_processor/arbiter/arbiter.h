#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "drivetrain/drive_message_generated.h"
#include "drivetrain/topics.h"
#include "intake/intake_message_generated.h"
#include "intake/topics.h"
#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

// Every target topic the drivetrain, shooter and intake consume has exactly
// one writer: this node. Producers publish on their own lanes and never
// coordinate, and the match mode -- which the FMS hands down, rather than
// anything the robot decides -- selects which producer reaches the
// actuators. One writer is what keeps the log replayable: two producers
// racing on one topic would resolve differently on replay than on the field.
namespace arbiter {

enum class Source : std::uint8_t { kNone, kTeleop, kAutonomous };

// Wire-compatible with the Target struct of any single-demand mechanism
// scaffolded by tools/new_subsystem.py (enabled:bool + 7 pad + demand:double).
struct alignas(8) GenericTarget {
  std::uint8_t enabled{0};
  std::uint8_t pad[7]{};
  double demand{0.0};
};
static_assert(sizeof(GenericTarget) == 16);
static_assert(talos::Message<GenericTarget>);

// The DriverStation handle reads disabled unless a packet younger than the
// command timeout says the DS is attached, so a silent or detached DS selects
// nothing.
inline Source SourceFor(const talos::DriverStation& ds) {
  if (!ds.enabled() || ds.estop()) return Source::kNone;
  if (ds.autonomous()) return Source::kAutonomous;
  if (ds.teleop()) return Source::kTeleop;
  // Test mode and anything unrecognised drive nothing. A mode this node does
  // not understand is not a reason to pass a stale producer through.
  return Source::kNone;
}

// One actuator family: forwards whichever producer the source selects, as
// each message arrives, to the one topic the subsystem consumes.
template <talos::Message M>
class Lane {
 public:
  Lane(talos::Node& node, const Source& source, std::string_view topic,
       std::string_view teleop, std::string_view autonomous,
       bool teleop_optional = false)
      : out_{node.publish<M>(topic)} {
    node.subscribe<M>(teleop,
                      [this, &source](const M& message) {
                        if (source == Source::kTeleop) out_.send(message);
                      },
                      {.optional = teleop_optional});
    // Autonomous is not written yet, so this lane has no publisher. That is
    // the design, not a broken link, and the graph lint is told so.
    node.subscribe<M>(autonomous,
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
        // station packet that enables teleop also admits the teleop target
        // the operator interface derived from it.
        ds_{node, [this] { OnMode(SourceFor(ds_)); }},
        chassis_{node, source_, drivetrain::topics::kTarget,
                 drivetrain::topics::kTeleopTarget,
                 drivetrain::topics::kAutoTarget},
        shooter_{node, source_, shooter::topics::kTarget,
                 shooter::topics::kTeleopTarget, shooter::topics::kAutoTarget},
        intake_{node, source_, intake::topics::kTarget,
                intake::topics::kTeleopTarget, intake::topics::kAutoTarget} {
    if (!node.config_path().empty()) {
      const auto cfg = talos::config::ParseRobotConfig(node.config_path());
      if (const auto* subs = cfg.toml_data["subsystems"].as_table()) {
        for (const auto& [key, val] : *subs) {
          const std::string_view sub_name = key.str();
          if (sub_name == "drivetrain" || sub_name == "shooter" ||
              sub_name == "intake") {
            continue;
          }
          const auto* tbl = val.as_table();
          if (tbl == nullptr) continue;
          const bool arbitrate = (*tbl)["arbitrate"].value_or(false);
          const std::int64_t button =
              (*tbl)["teleop_button"].value_or(std::int64_t{0});
          if (!arbitrate && button <= 0) continue;
          const std::string prefix = "/" + std::string{sub_name};
          extra_lanes_.push_back(std::make_unique<Lane<GenericTarget>>(
              node, source_, prefix + "/target", prefix + "/target/teleop",
              prefix + "/target/auto", /*teleop_optional=*/button <= 0));
        }
      }
    }
  }

  // A Driver Station that stops talking ages out here rather than leaving
  // its last mode selected.
  void update() { OnMode(SourceFor(ds_)); }

 private:
  void OnMode(Source next) {
    if (next == source_) return;
    source_ = next;
    // On every mode change, say "stop" once before the new producer speaks.
    // Otherwise the outgoing mode's last target stays live on the topic for
    // a command timeout: an autonomous request could still be driving the
    // robot after the switch to teleop.
    chassis_.Send(drivetrain::ChassisTarget{0, 0, 0, false});
    shooter_.Send(shooter::ShooterTarget{0, false});
    intake_.Send(intake::IntakeTarget{false, 0});
    for (const auto& lane : extra_lanes_) {
      lane->Send(GenericTarget{0, {}, 0.0});
    }
  }

  talos::Node& node_;
  Source source_ = Source::kNone;
  talos::DriverStation ds_;
  Lane<drivetrain::ChassisTarget> chassis_;
  Lane<shooter::ShooterTarget> shooter_;
  Lane<intake::IntakeTarget> intake_;
  std::vector<std::unique_ptr<Lane<GenericTarget>>> extra_lanes_;
};

}  // namespace arbiter
