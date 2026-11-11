#pragma once

#include "demo/intake_generated.h"
#include "demo/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/mechanisms/roller_with_sensor.h"
#include "talOS/node/node.h"

namespace demo {

// Roller + beam break, as a talos::mechanisms::RollerWithSensor named
// "roller": [motors.roller], and its [roller] parameters (voltages, eject
// time, the beam break's device name) in intake.toml. Pulls a piece in,
// holds it once the beam breaks, and spits it out for a fixed time when
// asked, on its target lane (/intake/target/<source>, see [targets] in
// intake.toml).
//
// With no target (lane stale, released or never sent) it behaves as asked
// for IDLE and drives nothing: the roller is neutral, while the state
// machine still tracks the beam break and finishes a timed eject. Losing
// enable (robot disabled, stale link) also cancels an eject or intake in
// progress; a held piece stays HOLDING.
class Intake {
 public:
  explicit Intake(talos::Node& node)
      : roller_{node, "roller"},
        goal_{node},
        status_{node.publish<IntakeStatus>(topics::kIntakeStatus)} {}

  void update() {
    if (const IntakeGoal* goal = goal_.get()) {
      roller_.set_goal(ToRoller(goal->command()));
    } else {
      roller_.stop();
    }
    roller_.update();
    status_.send(IntakeStatus{ToState(roller_.phase()), roller_.has_piece()});
  }

 private:
  using RollerCommand = talos::mechanisms::RollerCommand;
  using RollerPhase = talos::mechanisms::RollerPhase;

  static RollerCommand ToRoller(IntakeCommand command) {
    switch (command) {
      case IntakeCommand::INTAKE:
        return RollerCommand::kIntake;
      case IntakeCommand::EJECT:
        return RollerCommand::kEject;
      case IntakeCommand::IDLE:
        break;
    }
    return RollerCommand::kIdle;
  }
  // The intake never asks to feed, so kFeeding does not occur; it would
  // read as EJECTING (a timed run).
  static IntakeState ToState(RollerPhase phase) {
    switch (phase) {
      case RollerPhase::kIntaking:
        return IntakeState::INTAKING;
      case RollerPhase::kHolding:
        return IntakeState::HOLDING;
      case RollerPhase::kEjecting:
      case RollerPhase::kFeeding:
        return IntakeState::EJECTING;
      case RollerPhase::kIdle:
        break;
    }
    return IntakeState::IDLE;
  }

  talos::mechanisms::RollerWithSensor roller_;
  talos::Target<IntakeGoal> goal_;
  talos::Publisher<IntakeStatus> status_;
};

}  // namespace demo
