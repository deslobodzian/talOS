#pragma once

#include <chrono>
#include <optional>

#include "demo/intake_generated.h"
#include "demo/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace demo {

// Roller + beam break. Pulls a piece in, holds it once the beam breaks, and
// spits it out for a fixed time when asked, on its target lane
// (/intake/target/<source>, see [targets] in intake.toml).
//
// With no target (robot disabled, lane stale, released or never sent) it
// behaves as asked for IDLE and drives nothing: the roller is neutral, while
// the state machine still tracks the beam break and finishes a timed eject.
class Intake {
 public:
  using State = IntakeState;  // IDLE, INTAKING, HOLDING, EJECTING

  explicit Intake(talos::Node& node)
      : roller_{node, "roller"},
        beam_break_{node, "beam_break"},
        goal_{node},
        status_{node.publish<IntakeStatus>(topics::kIntakeStatus)},
        sm_{node, State::IDLE},
        intake_volts_{node.param<double>("intake_v")},
        hold_volts_{node.param<double>("hold_v")},
        eject_volts_{node.param<double>("eject_v")},
        eject_time_{std::chrono::microseconds{node.param<int>("eject_us")}} {}

  void update() {
    const IntakeGoal* goal = goal_.get();
    const IntakeCommand command =
        goal != nullptr ? goal->command() : IntakeCommand::IDLE;
    const bool has_piece = beam_break_.get();
    std::optional<double> volts;  // IDLE: none, neutral

    switch (sm_.state()) {
      case State::IDLE:
        if (command == IntakeCommand::INTAKE) sm_.go(State::INTAKING);
        break;
      case State::INTAKING:
        volts = intake_volts_;
        if (has_piece)
          sm_.go(State::HOLDING);
        else if (command == IntakeCommand::IDLE)
          sm_.go(State::IDLE);
        else if (command == IntakeCommand::EJECT)
          sm_.go(State::EJECTING);
        break;
      case State::HOLDING:
        volts = hold_volts_;
        if (command == IntakeCommand::EJECT)
          sm_.go(State::EJECTING);
        else if (!has_piece)
          sm_.go(State::IDLE);  // dropped it
        break;
      case State::EJECTING:
        volts = eject_volts_;
        if (sm_.time_in_state() >= eject_time_) sm_.go(State::IDLE);
        break;
    }
    // Only a live target drives the roller; anything not set this tick is
    // neutral.
    if (goal != nullptr && volts) roller_.set_voltage(*volts);
    status_.send(IntakeStatus{sm_.state(), has_piece});
  }

 private:
  talos::Motor roller_;
  talos::DigitalInput beam_break_;
  talos::Target<IntakeGoal> goal_;
  talos::Publisher<IntakeStatus> status_;
  talos::StateMachine<State> sm_;
  double intake_volts_, hold_volts_, eject_volts_;
  talos::Duration eject_time_;
};

}  // namespace demo
