#pragma once

#include "demo/elevator_generated.h"
#include "demo/intake_generated.h"
#include "demo/superstructure_generated.h"
#include "demo/topics.h"
#include "talOS/node/node.h"

namespace demo {

// Sequences the intake and the elevator, which are separate nodes: it only
// reads their states and writes their goals.
//
//   STOWED -> INTAKING -> HOLDING -> RAISING -> SCORING -> STOWED
//          (INTAKE)    (has piece) (SCORE_HIGH) (at height) (piece gone)
class Superstructure {
 public:
  using State = SuperstructureState;

  explicit Superstructure(talos::Node& node)
      // Nothing in this example publishes the goal (a driver or autonomous
      // node would), so it is optional to the launcher's graph check.
      : goal_{node.subscribe<SuperstructureGoal>(topics::kSuperstructureGoal,
                                                 {.optional = true})},
        intake_{node.subscribe<IntakeStatus>(topics::kIntakeStatus)},
        elevator_{node.subscribe<ElevatorStatus>(topics::kElevatorStatus)},
        intake_goal_{node.publish<IntakeGoal>(topics::kIntakeGoal)},
        elevator_goal_{node.publish<ElevatorGoal>(topics::kElevatorGoal)},
        status_{
            node.publish<SuperstructureStatus>(topics::kSuperstructureStatus)},
        sm_{node, State::STOWED},
        high_rot_{node.param<double>("high_rot")} {}

  void update() {
    const SuperstructureCommand command =
        goal_.fresh() ? goal_->command() : SuperstructureCommand::STOW;
    // Only advance on what the mechanisms report now. If either one has gone
    // quiet, hold the current step and keep sending its goals.
    if (intake_.fresh() && elevator_.fresh()) advance(command);

    switch (sm_.state()) {
      case State::STOWED:
      case State::HOLDING:
        send(IntakeCommand::IDLE, 0.0);
        break;
      case State::INTAKING:
        send(IntakeCommand::INTAKE, 0.0);
        break;
      case State::RAISING:
        send(IntakeCommand::IDLE, high_rot_);
        break;
      case State::SCORING:
        send(IntakeCommand::EJECT, high_rot_);
        break;
    }
    status_.send(SuperstructureStatus{sm_.state()});
  }

 private:
  void advance(SuperstructureCommand command) {
    const bool has_piece = intake_->has_piece();
    switch (sm_.state()) {
      case State::STOWED:
        if (has_piece)
          sm_.go(State::HOLDING);
        else if (command == SuperstructureCommand::INTAKE)
          sm_.go(State::INTAKING);
        break;
      case State::INTAKING:
        if (has_piece)
          sm_.go(State::HOLDING);
        else if (command == SuperstructureCommand::STOW)
          sm_.go(State::STOWED);
        break;
      case State::HOLDING:
        if (!has_piece)
          sm_.go(State::STOWED);
        else if (command == SuperstructureCommand::SCORE_HIGH)
          sm_.go(State::RAISING);
        break;
      case State::RAISING:
        if (elevator_at(high_rot_))
          sm_.go(State::SCORING);
        else if (command == SuperstructureCommand::STOW)
          sm_.go(State::HOLDING);
        break;
      case State::SCORING:
        if (!has_piece) sm_.go(State::STOWED);
        break;
    }
  }

  // at_goal for *this* height, not a goal the elevator reached earlier.
  bool elevator_at(double height) const {
    return elevator_->at_goal() && elevator_->goal_rot() == height;
  }

  void send(IntakeCommand intake, double elevator_height_rot) {
    intake_goal_.send(IntakeGoal{intake});
    elevator_goal_.send(ElevatorGoal{elevator_height_rot});
  }

  talos::Subscription<SuperstructureGoal> goal_;
  talos::Subscription<IntakeStatus> intake_;
  talos::Subscription<ElevatorStatus> elevator_;
  talos::Publisher<IntakeGoal> intake_goal_;
  talos::Publisher<ElevatorGoal> elevator_goal_;
  talos::Publisher<SuperstructureStatus> status_;
  talos::StateMachine<State> sm_;
  double high_rot_;
};

}  // namespace demo
