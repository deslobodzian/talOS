#include "talOS/mechanisms/roller_with_sensor.h"

#include <chrono>
#include <string>

namespace talos::mechanisms {

RollerWithSensor::RollerWithSensor(Node& node, std::string_view name)
    : node_{&node},
      label_{"RollerWithSensor '" + std::string{name} + "'"},
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
  RequireInUpdate(*node_, label_);
  requested_ = true;
  request_ = command;
  request_tick_ = node_->tick();
}

void RollerWithSensor::stop() {
  RequireInUpdate(*node_, label_);
  requested_ = false;
}

void RollerWithSensor::update() {
  RequireInUpdate(*node_, label_);
  const Time now = node_->now();
  const bool enabled = robot_enabled(*node_);
  // A request lasts the update() it was made in (see set_goal()).
  const bool live = requested_ && request_tick_ == node_->tick() && enabled;
  const RollerCommand command = live ? request_ : RollerCommand::kIdle;
  requested_ = false;
  // A tick with no live request counts as kIdle, so a release or a stale
  // lane ends an ask: press, release, press is two asks, a held button one.
  // Refusing a press held through a disable or a stale spell is the
  // producer's talos::OperatorArming.
  const bool newly_asked = live && command != last_command_;
  last_command_ = command;

  // A sensor sample that is not connected() (an input not answering, a
  // gateway configuring, a stale link) is no reading, whatever value it
  // carries: has_piece() keeps its last valid value, and a run of piece
  // readings still inside sensor_debounce_us starts over, so only
  // consecutive valid readings complete the debounce. A confirmed piece is
  // not a run in progress: it stays confirmed across the gap.
  sensor_connected_ = sensor_.connected();
  if (sensor_connected_) {
    has_piece_ = piece_debounce_.update(sensor_.get() != sensor_inverted_, now);
  } else if (!has_piece_) {
    piece_debounce_.reset();
  }

  const auto go = [&](RollerPhase next) {
    phase_ = next;
    phase_since_ = now;
  };
  // Losing enable (a Driver Station disable, a stale link, a fault) cancels
  // intaking and any timed run: re-enabled within eject_us, the roller does
  // not pick up where it stopped. A held piece stays kHolding (the sensor
  // still sees it).
  if (!enabled &&
      (phase_ == RollerPhase::kIntaking || phase_ == RollerPhase::kEjecting ||
       phase_ == RollerPhase::kFeeding)) {
    go(RollerPhase::kIdle);
  }
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
      // Intaking stops on the sensor, so it does not drive toward one it
      // cannot read: a piece may arrive during the gap. A held piece keeps
      // hold_v, and a timed eject or feed runs its clock, since neither
      // needs the sensor to stop.
      if (sensor_connected_) volts = intake_v_;
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
