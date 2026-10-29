#pragma once

#include <chrono>

#include "demo/intake_generated.h"
#include "demo/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace demo {

// Roller + beam break. Pulls a piece in, holds it once the beam breaks, and
// spits it out for a fixed time when asked.
class Intake {
 public:
  using State = IntakeState;  // IDLE, INTAKING, HOLDING, EJECTING

  explicit Intake(talos::Node& node)
      : roller_{node, "roller"},
        beam_break_{node, "beam_break"},
        goal_{node.subscribe<IntakeGoal>(topics::kIntakeGoal)},
        status_{node.publish<IntakeStatus>(topics::kIntakeStatus)},
        sm_{node, State::IDLE},
        intake_volts_{node.param<double>("intake_volts")},
        hold_volts_{node.param<double>("hold_volts")},
        eject_volts_{node.param<double>("eject_volts")},
        eject_time_{std::chrono::milliseconds{node.param<int>("eject_ms")}} {}

  void update() {
    // A stale goal means nobody is driving us: treat it as IDLE.
    const IntakeCommand command =
        goal_.fresh() ? goal_->command() : IntakeCommand::IDLE;
    const bool has_piece = beam_break_.get();

    switch (sm_.state()) {
      case State::IDLE:
        if (command == IntakeCommand::INTAKE) sm_.go(State::INTAKING);
        break;
      case State::INTAKING:
        roller_.set_voltage(intake_volts_);
        if (has_piece)
          sm_.go(State::HOLDING);
        else if (command == IntakeCommand::IDLE)
          sm_.go(State::IDLE);
        else if (command == IntakeCommand::EJECT)
          sm_.go(State::EJECTING);
        break;
      case State::HOLDING:
        roller_.set_voltage(hold_volts_);
        if (command == IntakeCommand::EJECT)
          sm_.go(State::EJECTING);
        else if (!has_piece)
          sm_.go(State::IDLE);  // dropped it
        break;
      case State::EJECTING:
        roller_.set_voltage(eject_volts_);
        if (sm_.time_in_state() >= eject_time_) sm_.go(State::IDLE);
        break;
    }
    // Anything not set this tick (IDLE) is neutral, and every motor is
    // neutral while the robot is disabled.
    status_.send(IntakeStatus{sm_.state(), has_piece});
  }

 private:
  talos::Motor roller_;
  talos::DigitalInput beam_break_;
  talos::Subscription<IntakeGoal> goal_;
  talos::Publisher<IntakeStatus> status_;
  talos::StateMachine<State> sm_;
  double intake_volts_, hold_volts_, eject_volts_;
  talos::Duration eject_time_;
};

}  // namespace demo
