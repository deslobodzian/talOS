#include "talOS/mechanisms/roller_with_sensor.h"

#include <chrono>
#include <string>

namespace talos::mechanisms {

RollerWithSensor::RollerWithSensor(Node& node, std::string_view name)
    : node_{&node},
      motor_{node, name},
      sensor_{node, Params{node, name}.get<std::string>("sensor")} {
  const Params params{node, name};
  sensor_inverted_ = params.get<bool>("sensor_inverted", false);
  piece_debounce_ = Debounce{params.micros("sensor_debounce_us", 0)};
  intake_v_ = params.number("intake_v");
  hold_v_ = params.optional_number("hold_v");
  eject_v_ = params.number("eject_v");
  eject_time_ = params.micros("eject_us");
  if (eject_time_ <= Duration::zero()) params.fail("eject_us", "must be > 0");
  idle_runs_ = params.get<bool>("idle_runs", false);
  feed_v_ = params.number("feed_v", intake_v_);
  feed_time_ = params.micros(
      "feed_us",
      std::chrono::duration_cast<std::chrono::microseconds>(eject_time_)
          .count());
  if (feed_time_ <= Duration::zero()) params.fail("feed_us", "must be > 0");
}

void RollerWithSensor::set_goal(RollerCommand command) {
  requested_ = true;
  request_ = command;
}

void RollerWithSensor::stop() { requested_ = false; }

void RollerWithSensor::update() {
  const Time now = node_->now();
  const bool live = requested_ && robot_enabled(*node_);
  const RollerCommand command = live ? request_ : RollerCommand::kIdle;
  requested_ = false;
  const bool newly_asked = command != last_command_;
  last_command_ = command;

  sensor_connected_ = sensor_.connected();
  has_piece_ = piece_debounce_.update(sensor_.get() != sensor_inverted_, now);

  const auto go = [&](RollerPhase next) {
    phase_ = next;
    phase_since_ = now;
  };
  // The roller output of the phase this tick starts in; transitions apply
  // from the next tick.
  std::optional<double> volts;
  switch (phase_) {
    case RollerPhase::kIdle:
      if (command == RollerCommand::kIntake) {
        go(RollerPhase::kIntaking);
      } else if (idle_runs_ && newly_asked &&
                 command == RollerCommand::kEject) {
        go(RollerPhase::kEjecting);
      } else if (idle_runs_ && newly_asked && command == RollerCommand::kFeed) {
        go(RollerPhase::kFeeding);
      }
      break;
    case RollerPhase::kIntaking:
      volts = intake_v_;
      if (has_piece_) {
        go(RollerPhase::kHolding);
      } else if (command == RollerCommand::kIdle) {
        go(RollerPhase::kIdle);
      } else if (command == RollerCommand::kEject) {
        go(RollerPhase::kEjecting);
      } else if (command == RollerCommand::kFeed) {
        go(RollerPhase::kFeeding);
      }
      break;
    case RollerPhase::kHolding:
      volts = hold_v_;
      if (command == RollerCommand::kEject) {
        go(RollerPhase::kEjecting);
      } else if (command == RollerCommand::kFeed) {
        go(RollerPhase::kFeeding);
      } else if (!has_piece_) {
        go(RollerPhase::kIdle);  // dropped it
      }
      break;
    case RollerPhase::kEjecting:
      volts = eject_v_;
      if (now - phase_since_ >= eject_time_) go(RollerPhase::kIdle);
      break;
    case RollerPhase::kFeeding:
      volts = feed_v_;
      if (now - phase_since_ >= feed_time_) go(RollerPhase::kIdle);
      break;
  }
  // Only a live request drives; anything not set this tick is neutral.
  driving_ = false;
  output_v_ = 0;
  if (live && volts.has_value()) {
    driving_ = true;
    output_v_ = *volts;
    motor_.set_voltage(output_v_);
  }
}

}  // namespace talos::mechanisms
