#pragma once

#include "demo/elevator_generated.h"
#include "demo/intake_generated.h"
#include "demo/superstructure_generated.h"
#include "demo/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace demo {

// Sequences the intake and the elevator, which are separate nodes: it only
// reads their states and writes their "superstructure" target lanes
// (/intake/target/superstructure, /elevator/target/superstructure), releasing
// both whenever either status is stale. Its own goal is a Target:
// /superstructure/target/teleop in teleop,
// /superstructure/target/auto in autonomous (the default policy). No live
// goal (disabled, stale, released, never sent) counts as STOW.
//
//   STOWED -> INTAKING -> HOLDING -> RAISING -> SCORING -> STOWED
//          (INTAKE)    (has piece) (SCORE_HIGH) (at height) (piece gone)
class Superstructure {
 public:
  using State = SuperstructureState;

  explicit Superstructure(talos::Node& node)
      : goal_{node},
        intake_{node.subscribe<IntakeStatus>(topics::kIntakeStatus)},
        elevator_{node.subscribe<ElevatorStatus>(topics::kElevatorStatus)},
        intake_goal_{node, "intake", "superstructure"},
        elevator_goal_{node, "elevator", "superstructure"},
        status_{
            node.publish<SuperstructureStatus>(topics::kSuperstructureStatus)},
        sm_{node, State::STOWED},
        high_rot_{node.param<double>("high_rot")} {}

  void update() {
    const SuperstructureGoal* goal = goal_.get();
    const SuperstructureCommand command =
        goal != nullptr ? goal->command() : SuperstructureCommand::STOW;
    const bool intake_fresh = intake_.fresh();
    const bool fresh = intake_fresh && elevator_.fresh();
    // Cancellation never waits for a dependency. Forget an interrupted step:
    // recovery must start from fresh observations, not resume an old eject.
    if (command == SuperstructureCommand::STOW || !fresh)
      sm_.go(intake_fresh && intake_->has_piece() ? State::HOLDING
                                                 : State::STOWED);
    else
      advance(command);

    if (!fresh) {
      intake_goal_.release();
      elevator_goal_.release();
      status_.send(SuperstructureStatus{sm_.state()});
      return;
    }

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
        break;
      case State::HOLDING:
        if (!has_piece)
          sm_.go(State::STOWED);
        else if (command == SuperstructureCommand::SCORE_HIGH)
          sm_.go(State::RAISING);
        break;
      case State::RAISING:
        if (!has_piece)
          sm_.go(State::STOWED);
        else if (elevator_at(high_rot_))
          sm_.go(State::SCORING);
        break;
      case State::SCORING:
        if (!has_piece)
          sm_.go(State::STOWED);
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

  talos::Target<SuperstructureGoal> goal_;
  talos::Subscription<IntakeStatus> intake_;
  talos::Subscription<ElevatorStatus> elevator_;
  talos::TargetLane<IntakeGoal> intake_goal_;
  talos::TargetLane<ElevatorGoal> elevator_goal_;
  talos::Publisher<SuperstructureStatus> status_;
  talos::StateMachine<State> sm_;
  double high_rot_;
};

}  // namespace demo
