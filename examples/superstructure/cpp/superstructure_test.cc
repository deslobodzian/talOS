// The whole example robot in one simulated process: the intake, elevator and
// superstructure nodes talking over their real topics, with SimHardware in
// place of the bridge, all configured by config/robot.toml and
// config/subsystems/.

#include "examples/superstructure/cpp/superstructure.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "examples/superstructure/cpp/elevator.h"
#include "examples/superstructure/cpp/intake.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"

namespace demo {
namespace {

using namespace std::chrono_literals;
using talos::hardware::Mode;
using talos::testing::Sim;
using talos::testing::SimHardware;

std::string RobotConfig() {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv("TALOS_TEST_CONFIG");
  if (!runfiles || location == nullptr) {
    throw std::runtime_error("robot config unavailable: " + error);
  }
  return runfiles->Rlocation(location);
}

// The values in config/subsystems/*.toml.
constexpr double kIntakeVolts = 8.0;
constexpr double kHoldVolts = 1.0;
constexpr double kEjectVolts = -10.0;
constexpr double kHighRot = 10.0;

class SuperstructureTest : public ::testing::Test {
 protected:
  // Holds a driver command the way a driver node would: republished every
  // 20 ms.
  void Hold(SuperstructureCommand command, std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 20ms) {
      sim.publish(topics::kSuperstructureGoal, SuperstructureGoal{command});
      sim.step(20ms);
    }
  }
  void SetBeamBreak(bool broken) {
    hw.set_digital_input("intake", "beam_break", broken);
  }

  SuperstructureState SuperstructureNow() const {
    return Latest<SuperstructureStatus>(topics::kSuperstructureStatus).state();
  }
  IntakeStatus IntakeNow() const {
    return Latest<IntakeStatus>(topics::kIntakeStatus);
  }
  ElevatorStatus ElevatorNow() const {
    return Latest<ElevatorStatus>(topics::kElevatorStatus);
  }
  talos::hardware::MotorRequest RollerRequest() const {
    return hw.motor_request("intake", "roller");
  }
  talos::hardware::MotorRequest LiftRequest() const {
    return hw.motor_request("elevator", "lift");
  }

  template <class M>
  M Latest(const char* topic) const {
    const auto message = sim.latest<M>(topic);
    EXPECT_TRUE(message.has_value()) << topic;
    return message.value_or(M{});
  }

  // Up to HOLDING: intake until the beam breaks.
  void Collect() {
    Hold(SuperstructureCommand::INTAKE, 100ms);
    SetBeamBreak(true);
    Hold(SuperstructureCommand::INTAKE, 100ms);
    ASSERT_EQ(SuperstructureNow(), SuperstructureState::HOLDING);
  }

  Sim sim{RobotConfig()};
  demo::Intake& intake = sim.add<demo::Intake>("intake");
  demo::Elevator& elevator = sim.add<demo::Elevator>("elevator");
  demo::Superstructure& superstructure =
      sim.add<demo::Superstructure>("superstructure");
  SimHardware hw{sim};
};

TEST_F(SuperstructureTest, IntakesRaisesScoresAndStows) {
  hw.enable();
  hw.ideal_motors();

  // STOWED: elevator at 0, roller off.
  Hold(SuperstructureCommand::STOW, 100ms);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::STOWED);
  EXPECT_EQ(RollerRequest().mode, Mode::kNeutral);
  EXPECT_EQ(LiftRequest().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(LiftRequest().demand, 0.0);
  EXPECT_EQ(ElevatorNow().state(), ElevatorState::STOWED);
  EXPECT_TRUE(ElevatorNow().at_goal());

  // INTAKING: roller pulls in.
  Hold(SuperstructureCommand::INTAKE, 100ms);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::INTAKING);
  EXPECT_EQ(IntakeNow().state(), IntakeState::INTAKING);
  EXPECT_EQ(RollerRequest().mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(RollerRequest().demand, kIntakeVolts);

  // The beam breaks: HOLDING, with a small hold voltage.
  SetBeamBreak(true);
  Hold(SuperstructureCommand::INTAKE, 100ms);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::HOLDING);
  EXPECT_EQ(IntakeNow().state(), IntakeState::HOLDING);
  EXPECT_TRUE(IntakeNow().has_piece());
  EXPECT_DOUBLE_EQ(RollerRequest().demand, kHoldVolts);
  EXPECT_DOUBLE_EQ(LiftRequest().demand, 0.0);

  // SCORE_HIGH with a lift that has not got there yet: RAISING.
  hw.ideal_motors(false);
  Hold(SuperstructureCommand::SCORE_HIGH, 100ms);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::RAISING);
  EXPECT_EQ(LiftRequest().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(LiftRequest().demand, kHighRot);
  EXPECT_EQ(ElevatorNow().state(), ElevatorState::MOVING);
  EXPECT_FALSE(ElevatorNow().at_goal());
  EXPECT_DOUBLE_EQ(RollerRequest().demand,
                   kHoldVolts);  // still holding the piece

  // The lift arrives: at_goal, then SCORING ejects.
  hw.ideal_motors();
  Hold(SuperstructureCommand::SCORE_HIGH, 100ms);
  EXPECT_EQ(ElevatorNow().state(), ElevatorState::AT_GOAL);
  EXPECT_TRUE(ElevatorNow().at_goal());
  EXPECT_DOUBLE_EQ(ElevatorNow().height_rot(), kHighRot);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::SCORING);
  EXPECT_EQ(IntakeNow().state(), IntakeState::EJECTING);
  EXPECT_DOUBLE_EQ(RollerRequest().demand, kEjectVolts);

  // The piece leaves: back to STOWED, elevator down; the intake finishes
  // its timed eject and idles.
  SetBeamBreak(false);
  Hold(SuperstructureCommand::SCORE_HIGH, 100ms);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::STOWED);
  EXPECT_DOUBLE_EQ(LiftRequest().demand, 0.0);
  EXPECT_EQ(ElevatorNow().state(), ElevatorState::STOWED);
  EXPECT_EQ(IntakeNow().state(), IntakeState::EJECTING);
  EXPECT_DOUBLE_EQ(RollerRequest().demand, kEjectVolts);

  Hold(SuperstructureCommand::SCORE_HIGH, 500ms);
  EXPECT_EQ(IntakeNow().state(), IntakeState::IDLE);
  EXPECT_EQ(RollerRequest().mode, Mode::kNeutral);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::STOWED);
}

TEST_F(SuperstructureTest, DisablingNeutralsEveryMotorAndKeepsTheStates) {
  hw.enable();
  hw.ideal_motors();
  Collect();
  hw.ideal_motors(false);
  Hold(SuperstructureCommand::SCORE_HIGH, 60ms);
  ASSERT_EQ(SuperstructureNow(), SuperstructureState::RAISING);

  hw.enable(false);
  Hold(SuperstructureCommand::SCORE_HIGH, 200ms);
  EXPECT_EQ(RollerRequest().mode, Mode::kNeutral);
  EXPECT_EQ(LiftRequest().mode, Mode::kNeutral);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::RAISING);
  EXPECT_EQ(IntakeNow().state(), IntakeState::HOLDING);
  EXPECT_EQ(ElevatorNow().state(), ElevatorState::MOVING);

  // Re-enabled, everything carries on from where it was.
  hw.enable();
  hw.ideal_motors();
  Hold(SuperstructureCommand::SCORE_HIGH, 100ms);
  EXPECT_EQ(LiftRequest().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(LiftRequest().demand, kHighRot);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::SCORING);
  EXPECT_DOUBLE_EQ(RollerRequest().demand, kEjectVolts);
}

// The superstructure on its own, with the mechanism states published by hand
// so either one can go quiet.
class SuperstructureAlone : public ::testing::Test {
 protected:
  // One 20 ms period: the driver's command, plus whichever mechanism states
  // are still being reported.
  void Tick(SuperstructureCommand command, std::optional<IntakeStatus> intake,
            std::optional<ElevatorStatus> elevator) {
    sim.publish(topics::kSuperstructureGoal, SuperstructureGoal{command});
    if (intake) sim.publish(topics::kIntakeStatus, *intake);
    if (elevator) sim.publish(topics::kElevatorStatus, *elevator);
    sim.step(20ms);
  }
  SuperstructureState State() const {
    return sim.latest<SuperstructureStatus>(topics::kSuperstructureStatus)
        .value_or(SuperstructureStatus{})
        .state();
  }
  double ElevatorGoalRot() const {
    return sim.latest<ElevatorGoal>(topics::kElevatorGoal)
        .value_or(ElevatorGoal{})
        .height_rot();
  }

  const IntakeStatus kHolding{IntakeState::HOLDING, true};
  const ElevatorStatus kDown{ElevatorState::STOWED, 0.0, 0.0, true};
  const ElevatorStatus kUp{ElevatorState::AT_GOAL, kHighRot, kHighRot, true};

  Sim sim{RobotConfig()};
  demo::Superstructure& superstructure =
      sim.add<demo::Superstructure>("superstructure");
};

TEST_F(SuperstructureAlone, StaleStatusHoldsTheStateAndDoesNotAdvance) {
  for (int i = 0; i < 5; ++i) {
    Tick(SuperstructureCommand::INTAKE, kHolding, kDown);
  }
  ASSERT_EQ(State(), SuperstructureState::HOLDING);

  // The intake goes quiet for longer than command_timeout_us (100 ms).
  for (int i = 0; i < 6; ++i) {
    Tick(SuperstructureCommand::INTAKE, std::nullopt, kDown);
  }
  // Its last word was "holding a piece", but it is stale: SCORE_HIGH must not
  // raise, and HOLDING's goals (elevator down, intake idle) keep going out.
  for (int i = 0; i < 10; ++i) {
    Tick(SuperstructureCommand::SCORE_HIGH, std::nullopt, kDown);
  }
  EXPECT_EQ(State(), SuperstructureState::HOLDING);
  EXPECT_DOUBLE_EQ(ElevatorGoalRot(), 0.0);
  const auto intake_goal = sim.latest<IntakeGoal>(topics::kIntakeGoal);
  if (!intake_goal.has_value()) {
    FAIL() << "no intake goal";
  }
  EXPECT_EQ(intake_goal->command(), IntakeCommand::IDLE);

  // It reports again: RAISING.
  Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
  Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
  ASSERT_EQ(State(), SuperstructureState::RAISING);
  EXPECT_DOUBLE_EQ(ElevatorGoalRot(), kHighRot);

  // Now the elevator goes quiet past the timeout, and the driver lets go
  // (STOW). Without a fresh elevator state it stays RAISING and keeps asking
  // for the height rather than acting on old data.
  for (int i = 0; i < 6; ++i) {
    Tick(SuperstructureCommand::SCORE_HIGH, kHolding, std::nullopt);
  }
  for (int i = 0; i < 10; ++i) {
    Tick(SuperstructureCommand::STOW, kHolding, std::nullopt);
  }
  EXPECT_EQ(State(), SuperstructureState::RAISING);
  EXPECT_DOUBLE_EQ(ElevatorGoalRot(), kHighRot);

  // A fresh at_goal for the high goal: SCORING.
  Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kUp);
  Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kUp);
  EXPECT_EQ(State(), SuperstructureState::SCORING);
}

TEST_F(SuperstructureAlone, AtGoalForAnEarlierHeightDoesNotCount) {
  for (int i = 0; i < 5; ++i) {
    Tick(SuperstructureCommand::INTAKE, kHolding, kDown);
  }
  // The elevator is at_goal, but for 0: RAISING waits for its own height.
  for (int i = 0; i < 10; ++i) {
    Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
  }
  EXPECT_EQ(State(), SuperstructureState::RAISING);
}

TEST(ElevatorAlone, HoldsItsLastGoalWhenTheTargetGoesStale) {
  Sim sim{RobotConfig()};
  sim.add<demo::Elevator>("elevator");
  SimHardware hw{sim};
  hw.enable();
  hw.ideal_motors();
  sim.publish(topics::kElevatorGoal, ElevatorGoal{kHighRot});
  sim.step(300ms);  // the target is long stale (command_timeout_us = 100 ms)
  EXPECT_EQ(hw.motor_request("elevator", "lift").mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(hw.motor_request("elevator", "lift").demand, kHighRot);
  const auto status = sim.latest<ElevatorStatus>(topics::kElevatorStatus);
  if (!status.has_value()) {
    FAIL() << "no elevator status";
  }
  EXPECT_EQ(status->state(), ElevatorState::AT_GOAL);
}

}  // namespace
}  // namespace demo
