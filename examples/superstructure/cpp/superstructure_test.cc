// The whole example robot in one simulated process: the intake, elevator and
// superstructure nodes talking over their real topics and target lanes, with
// SimHardware in place of the bridge and the Driver Station, all configured by
// config/robot.toml and config/subsystems/.

#include "examples/superstructure/cpp/superstructure.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "examples/superstructure/cpp/elevator.h"
#include "examples/superstructure/cpp/intake.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"

namespace demo {
namespace {

using namespace std::chrono_literals;
using talos::hardware::Mode;
using talos::testing::DsPacket;
using talos::testing::LatestTarget;
using talos::testing::PublishTarget;
using talos::testing::ReleaseTarget;
using talos::testing::Sim;
using talos::testing::SimHardware;
namespace ds = talos::driver_station;

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
constexpr double kMaxHeightRot = 12.0;
constexpr double kHomingVolts = -3.0;
constexpr auto kHomingTimeout = 6000ms;
constexpr auto kHomingStall = 500ms;
// The switch zeroes the lift once it has read closed for home_samples (3)
// elevator periods.
constexpr auto kHomeTime = 30ms;

// Enables the robot and puts the Driver Station in enabled teleop: a Target
// drives only with both. The 10 ms step lets the mode begin before the first
// lane is published.
void EnableTeleop(Sim& sim, SimHardware& hw) {
  hw.enable();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(10ms);
}

// The elevator's bottom switch closed: the lift starts at the bottom, so it
// zeroes there (Motor::rezero) without moving. The switch is normally open to
// ground behind a pull-up: closed reads false.
void LiftAtBottom(SimHardware& hw, bool down = true) {
  hw.set_digital_input("elevator", "bottom_limit", !down);
}

class SuperstructureTest : public ::testing::Test {
 protected:
  // The lift zeroes on its switch, then leaves it as it rises.
  SuperstructureTest() {
    LiftAtBottom(hw);
    sim.step(kHomeTime);
    LiftAtBottom(hw, false);
  }

  // Holds a driver command the way a driver node would: its teleop lane
  // republished every 20 ms.
  void Hold(SuperstructureCommand command, std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 20ms) {
      PublishTarget(sim, "superstructure", "teleop",
                    SuperstructureGoal{command});
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
  EnableTeleop(sim, hw);
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
  const auto intake_lane =
      LatestTarget<IntakeGoal>(sim, "intake", "superstructure");
  if (!intake_lane.has_value()) {
    FAIL() << "Expected an intake target";
  }
  const auto& intake_target = *intake_lane;
  EXPECT_EQ(intake_target.active, 1);
  EXPECT_EQ(intake_target.message.command(), IntakeCommand::INTAKE);

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

TEST_F(SuperstructureTest, DisablingNeutralsEveryMotorAndStepsBackToHolding) {
  EnableTeleop(sim, hw);
  hw.ideal_motors();
  Collect();
  hw.ideal_motors(false);
  Hold(SuperstructureCommand::SCORE_HIGH, 60ms);
  ASSERT_EQ(SuperstructureNow(), SuperstructureState::RAISING);

  // Disabled, every motor is neutral and every state is still published.
  // The superstructure's own goal is a Target too, so it has none while
  // disabled: that counts as STOW, and RAISING steps back to HOLDING (the
  // piece stays in). The elevator, with no target, is neutral and reports
  // STOWED at its height of 0 (the lift never moved).
  hw.enable(false);
  Hold(SuperstructureCommand::SCORE_HIGH, 200ms);
  EXPECT_EQ(RollerRequest().mode, Mode::kNeutral);
  EXPECT_EQ(LiftRequest().mode, Mode::kNeutral);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::HOLDING);
  EXPECT_EQ(IntakeNow().state(), IntakeState::HOLDING);
  EXPECT_EQ(ElevatorNow().state(), ElevatorState::STOWED);
  EXPECT_FALSE(ElevatorNow().at_goal());

  // Re-enabled with SCORE_HIGH still held, it raises and scores again.
  hw.enable();
  hw.ideal_motors();
  Hold(SuperstructureCommand::SCORE_HIGH, 160ms);
  EXPECT_EQ(LiftRequest().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(LiftRequest().demand, kHighRot);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::SCORING);
  EXPECT_DOUBLE_EQ(RollerRequest().demand, kEjectVolts);
}

TEST_F(SuperstructureTest, NoGoalCountsAsStow) {
  EnableTeleop(sim, hw);
  hw.ideal_motors();
  // Never sent: STOWED, and the superstructure still drives both mechanisms'
  // lanes (elevator to 0, intake idle).
  sim.step(100ms);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::STOWED);
  EXPECT_EQ(LiftRequest().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(LiftRequest().demand, 0.0);
  EXPECT_EQ(RollerRequest().mode, Mode::kNeutral);

  // INTAKING, then the driver's lane is released: STOW, roller off.
  Hold(SuperstructureCommand::INTAKE, 100ms);
  ASSERT_EQ(SuperstructureNow(), SuperstructureState::INTAKING);
  ReleaseTarget<SuperstructureGoal>(sim, "superstructure", "teleop");
  sim.step(60ms);
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::STOWED);
  EXPECT_EQ(IntakeNow().state(), IntakeState::IDLE);
  EXPECT_EQ(RollerRequest().mode, Mode::kNeutral);

  // INTAKING, then the driver's lane goes stale: STOW as well.
  Hold(SuperstructureCommand::INTAKE, 100ms);
  ASSERT_EQ(SuperstructureNow(), SuperstructureState::INTAKING);
  sim.step(160ms);  // past command_timeout_us (100 ms)
  EXPECT_EQ(SuperstructureNow(), SuperstructureState::STOWED);
  EXPECT_EQ(RollerRequest().mode, Mode::kNeutral);
}

// The superstructure on its own, with the mechanism states published by hand
// so either one can go quiet.
class SuperstructureAlone : public ::testing::Test {
 protected:
  void SetUp() override { EnableTeleop(sim, hw); }

  // One 20 ms period: the driver's command, plus whichever mechanism states
  // are still being reported.
  void Tick(SuperstructureCommand command, std::optional<IntakeStatus> intake,
            std::optional<ElevatorStatus> elevator) {
    PublishTarget(sim, "superstructure", "teleop", SuperstructureGoal{command});
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
    const auto lane =
        LatestTarget<ElevatorGoal>(sim, "elevator", "superstructure");
    EXPECT_TRUE(lane.has_value() && lane->active == 1);
    return lane.has_value() ? lane->message.height_rot() : -1.0;
  }
  void ExpectLanesReleased() const {
    const auto intake =
        LatestTarget<IntakeGoal>(sim, "intake", "superstructure");
    const auto elevator =
        LatestTarget<ElevatorGoal>(sim, "elevator", "superstructure");
    ASSERT_TRUE(intake.has_value());
    ASSERT_TRUE(elevator.has_value());
    EXPECT_EQ(intake->active, 0);
    EXPECT_EQ(elevator->active, 0);
  }

  const IntakeStatus kHolding{IntakeState::HOLDING, true};
  const ElevatorStatus kDown{ElevatorState::STOWED, 0.0, 0.0, true};
  const ElevatorStatus kUp{ElevatorState::AT_GOAL, kHighRot, kHighRot, true};

  Sim sim{RobotConfig()};
  demo::Superstructure& superstructure =
      sim.add<demo::Superstructure>("superstructure");
  SimHardware hw{sim};
};

TEST_F(SuperstructureAlone, StowWinsWhenRaisingArrivesAtScoringHeight) {
  Tick(SuperstructureCommand::INTAKE, kHolding, kDown);
  Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
  ASSERT_EQ(State(), SuperstructureState::RAISING);
  Tick(SuperstructureCommand::STOW, kHolding, kUp);
  EXPECT_EQ(State(), SuperstructureState::HOLDING);
  EXPECT_DOUBLE_EQ(ElevatorGoalRot(), 0.0);
  const auto intake = LatestTarget<IntakeGoal>(sim, "intake", "superstructure");
  ASSERT_TRUE(intake.has_value());
  EXPECT_EQ(intake->message.command(), IntakeCommand::IDLE);
}

TEST_F(SuperstructureAlone, ScoringStowsOnExplicitCommandOrTargetRelease) {
  for (bool release : {false, true}) {
    Tick(SuperstructureCommand::INTAKE, kHolding, kDown);
    Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
    Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kUp);
    ASSERT_EQ(State(), SuperstructureState::SCORING);
    if (release) {
      ReleaseTarget<SuperstructureGoal>(sim, "superstructure", "teleop");
      sim.publish(topics::kIntakeStatus, kHolding);
      sim.publish(topics::kElevatorStatus, kUp);
      sim.step(20ms);
    } else {
      Tick(SuperstructureCommand::STOW, kHolding, kUp);
    }
    EXPECT_EQ(State(), SuperstructureState::HOLDING);
    EXPECT_DOUBLE_EQ(ElevatorGoalRot(), 0.0);
    const auto intake =
        LatestTarget<IntakeGoal>(sim, "intake", "superstructure");
    ASSERT_TRUE(intake.has_value());
    EXPECT_EQ(intake->message.command(), IntakeCommand::IDLE);
  }
}

TEST_F(SuperstructureAlone, StaleIntakeReleasesLanesUntilObservationsReturn) {
  Tick(SuperstructureCommand::INTAKE, kHolding, kDown);
  ASSERT_EQ(State(), SuperstructureState::HOLDING);
  for (int i = 0; i < 6; ++i) {
    Tick(SuperstructureCommand::SCORE_HIGH, std::nullopt, kDown);
  }
  EXPECT_EQ(State(), SuperstructureState::STOWED);
  ExpectLanesReleased();

  Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
  EXPECT_EQ(State(), SuperstructureState::HOLDING);
  EXPECT_DOUBLE_EQ(ElevatorGoalRot(), 0.0);
  Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
  EXPECT_EQ(State(), SuperstructureState::RAISING);
  EXPECT_DOUBLE_EQ(ElevatorGoalRot(), kHighRot);
}

TEST_F(SuperstructureAlone, StaleStatusWhileRaisingStowsImmediately) {
  for (bool stale_intake : {false, true}) {
    Tick(SuperstructureCommand::INTAKE, kHolding, kDown);
    Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
    ASSERT_EQ(State(), SuperstructureState::RAISING);
    for (int i = 0; i < 4; ++i) {
      Tick(SuperstructureCommand::SCORE_HIGH,
           stale_intake ? std::nullopt : std::optional{kHolding},
           stale_intake ? std::optional{kDown} : std::nullopt);
    }
    ASSERT_EQ(State(), SuperstructureState::RAISING);

    // STOW coincides with the first tick that sees the status as stale.
    Tick(SuperstructureCommand::STOW,
         stale_intake ? std::nullopt : std::optional{kHolding},
         stale_intake ? std::optional{kDown} : std::nullopt);
    EXPECT_EQ(State(), stale_intake ? SuperstructureState::STOWED
                                    : SuperstructureState::HOLDING);
    ExpectLanesReleased();

    // Cancellation persists when the elevator reports its old high goal.
    Tick(SuperstructureCommand::STOW, kHolding, kUp);
    EXPECT_EQ(State(), SuperstructureState::HOLDING);
    EXPECT_DOUBLE_EQ(ElevatorGoalRot(), 0.0);
  }
}

TEST_F(SuperstructureAlone, PieceLostWhileRaisingDoesNotStartEject) {
  Tick(SuperstructureCommand::INTAKE, kHolding, kDown);
  Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
  ASSERT_EQ(State(), SuperstructureState::RAISING);
  Tick(SuperstructureCommand::SCORE_HIGH,
       IntakeStatus{IntakeState::IDLE, false}, kUp);
  EXPECT_EQ(State(), SuperstructureState::STOWED);
  EXPECT_DOUBLE_EQ(ElevatorGoalRot(), 0.0);
  const auto intake = LatestTarget<IntakeGoal>(sim, "intake", "superstructure");
  ASSERT_TRUE(intake.has_value());
  EXPECT_EQ(intake->active, 1);
  EXPECT_EQ(intake->message.command(), IntakeCommand::IDLE);
}

TEST_F(SuperstructureAlone, StaleScoringReleasesEjectAndDoesNotReplayIt) {
  const IntakeStatus empty{IntakeState::IDLE, false};
  for (bool stale_intake : {false, true}) {
    Tick(SuperstructureCommand::INTAKE, kHolding, kDown);
    Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
    Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kUp);
    ASSERT_EQ(State(), SuperstructureState::SCORING);
    for (int i = 0; i < 6; ++i) {
      Tick(SuperstructureCommand::SCORE_HIGH,
           stale_intake ? std::nullopt : std::optional{kHolding},
           stale_intake ? std::optional{kUp} : std::nullopt);
    }
    EXPECT_NE(State(), SuperstructureState::SCORING);
    ExpectLanesReleased();

    // The piece left while a dependency was silent. A held SCORE_HIGH must
    // not replay EJECT or the high goal when current observations return.
    for (int i = 0; i < 3; ++i) {
      Tick(SuperstructureCommand::SCORE_HIGH, empty, kUp);
      EXPECT_EQ(State(), SuperstructureState::STOWED);
      EXPECT_DOUBLE_EQ(ElevatorGoalRot(), 0.0);
      const auto intake =
          LatestTarget<IntakeGoal>(sim, "intake", "superstructure");
      ASSERT_TRUE(intake.has_value());
      EXPECT_EQ(intake->active, 1);
      EXPECT_EQ(intake->message.command(), IntakeCommand::IDLE);
    }
  }
}

TEST_F(SuperstructureAlone, GoalLossAndDisableCancelWithStaleStatuses) {
  for (int cancellation = 0; cancellation < 3; ++cancellation) {
    Tick(SuperstructureCommand::INTAKE, kHolding, kDown);
    Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kDown);
    Tick(SuperstructureCommand::SCORE_HIGH, kHolding, kUp);
    ASSERT_EQ(State(), SuperstructureState::SCORING);
    for (int i = 0; i < 4; ++i) {
      Tick(SuperstructureCommand::SCORE_HIGH, std::nullopt, std::nullopt);
    }
    ASSERT_EQ(State(), SuperstructureState::SCORING);
    if (cancellation == 0)
      ReleaseTarget<SuperstructureGoal>(sim, "superstructure", "teleop");
    else if (cancellation == 1)
      hw.enable(false);
    sim.step(cancellation == 2 ? 120ms : 20ms);  // Released, disabled, expired.
    EXPECT_EQ(State(), SuperstructureState::STOWED);
    ExpectLanesReleased();

    // Fresh mechanism states cannot revive a cancelled score.
    sim.publish(topics::kIntakeStatus, kHolding);
    sim.publish(topics::kElevatorStatus, kUp);
    sim.step(20ms);
    EXPECT_EQ(State(), SuperstructureState::HOLDING);
    EXPECT_DOUBLE_EQ(ElevatorGoalRot(), 0.0);
    if (cancellation == 1) {
      hw.enable();
      sim.step(
          20ms);  // Refresh hardware without shifting the 20 ms test phase.
    }
  }
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

// The elevator on its own, driven through its superstructure lane by hand,
// homed at the bottom.
class ElevatorAlone : public ::testing::Test {
 protected:
  void SetUp() override {
    EnableTeleop(sim, hw);
    hw.ideal_motors();
    LiftAtBottom(hw);
    sim.step(kHomeTime);
    LiftAtBottom(hw, false);
  }

  // The superstructure's lane, republished every 10 ms.
  void Drive(double height_rot, std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 10ms) {
      PublishTarget(sim, "elevator", "superstructure",
                    ElevatorGoal{height_rot});
      sim.step(10ms);
    }
  }
  talos::hardware::MotorRequest Lift() const {
    return hw.motor_request("elevator", "lift");
  }
  // Drive(), checking the lift output the hardware applies after every
  // gateway tick: true if any was a motion-magic goal. (A request the node
  // built from a State before a reset is in the old frame; the gateway
  // applies it neutral.)
  bool DriveSawMotionMagic(double height_rot,
                           std::chrono::milliseconds duration) {
    bool seen = false;
    for (auto t = 0ms; t < duration; t += 10ms) {
      PublishTarget(sim, "elevator", "superstructure",
                    ElevatorGoal{height_rot});
      for (int i = 0; i < 2; ++i) {
        sim.step(5ms);
        seen |= hw.applied_motor("elevator", "lift").mode == Mode::kMotionMagic;
      }
    }
    return seen;
  }
  // The carriage reaches its bottom stop, at `raw_rot` in the motor's frame.
  // Long enough on the closed switch to zero, then the lift rises off it.
  void ReachBottom(double raw_rot) {
    hw.set_motor("elevator", "lift", raw_rot, 0.0);
    LiftAtBottom(hw);
    sim.step(kHomeTime);
    LiftAtBottom(hw, false);
  }
  void ExpectHoming() const {
    EXPECT_EQ(Lift().mode, Mode::kVoltage);
    EXPECT_DOUBLE_EQ(Lift().demand, kHomingVolts);
    EXPECT_EQ(Status().state(), ElevatorState::HOMING);
    EXPECT_DOUBLE_EQ(Status().goal_rot(), 0.0);
    EXPECT_FALSE(Status().at_goal());
  }
  ElevatorStatus Status() const {
    const auto status = sim.latest<ElevatorStatus>(topics::kElevatorStatus);
    EXPECT_TRUE(status.has_value());
    return status.value_or(ElevatorStatus{});
  }
  // Neutral, and the status says it is driving nothing.
  void ExpectNeutral() const {
    EXPECT_EQ(Lift().mode, Mode::kNeutral);
    EXPECT_DOUBLE_EQ(Status().goal_rot(), 0.0);
    EXPECT_FALSE(Status().at_goal());
  }

  Sim sim{RobotConfig()};
  demo::Elevator& elevator = sim.add<demo::Elevator>("elevator");
  SimHardware hw{sim};
};

TEST_F(ElevatorAlone, NeutralWhenTheLaneGoesStale) {
  Drive(kHighRot, 50ms);
  EXPECT_DOUBLE_EQ(Lift().demand, kHighRot);
  EXPECT_EQ(Status().state(), ElevatorState::AT_GOAL);
  EXPECT_TRUE(Status().at_goal());
  EXPECT_DOUBLE_EQ(Status().goal_rot(), kHighRot);
  ASSERT_EQ(Lift().mode, Mode::kMotionMagic);
  // Nothing more: it does not hold the old goal past command_timeout_us.
  sim.step(110ms);
  ExpectNeutral();
  // Up at kHighRot with nothing driving it: MOVING, not AT_GOAL.
  EXPECT_EQ(Status().state(), ElevatorState::MOVING);
}

TEST_F(ElevatorAlone, NeutralWhenTheLaneIsReleased) {
  Drive(kHighRot, 50ms);
  ASSERT_EQ(Lift().mode, Mode::kMotionMagic);
  ReleaseTarget<ElevatorGoal>(sim, "elevator", "superstructure");
  sim.step(20ms);
  ExpectNeutral();
}

TEST_F(ElevatorAlone, NeutralBeforeAnyTargetAndWhenDisabled) {
  sim.step(100ms);
  ExpectNeutral();
  EXPECT_EQ(Status().state(), ElevatorState::STOWED);
  hw.enable(false);
  Drive(kHighRot, 100ms);
  ExpectNeutral();
}

TEST_F(ElevatorAlone, NeutralOnDriverStationLoss) {
  Drive(kHighRot, 50ms);
  ASSERT_EQ(Lift().mode, Mode::kMotionMagic);
  hw.stop_driver_station();
  Drive(kHighRot, 150ms);  // the lane stays fresh; the DS does not
  ExpectNeutral();
}

TEST_F(ElevatorAlone, NeutralWithTheLiftDisconnected) {
  hw.set_connected("elevator", "lift", false);
  Drive(kHighRot, 50ms);
  ExpectNeutral();
  EXPECT_EQ(Status().state(), ElevatorState::MOVING);
}

TEST_F(ElevatorAlone, NeutralOnANonFiniteGoal) {
  Drive(std::numeric_limits<double>::quiet_NaN(), 50ms);
  ExpectNeutral();
}

// REQ-SAFE-36
TEST_F(ElevatorAlone, ClampsGoalsToTheTravel) {
  Drive(kMaxHeightRot + 8.0, 30ms);
  EXPECT_EQ(Lift().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(Lift().demand, kMaxHeightRot);
  EXPECT_DOUBLE_EQ(Status().goal_rot(), kMaxHeightRot);
  Drive(-3.0, 30ms);
  EXPECT_DOUBLE_EQ(Lift().demand, 0.0);
  EXPECT_DOUBLE_EQ(Status().goal_rot(), 0.0);
}

// A switch stuck closed zeroed the lift at the bottom; the first move up
// shows it still closed well above 0, and the elevator stops.
// REQ-SAFE-36
TEST_F(ElevatorAlone, AStuckSwitchStopsTheFirstMoveUp) {
  LiftAtBottom(hw);
  Drive(kHighRot, 30ms);  // ideal motors: up at once
  EXPECT_EQ(Lift().mode, Mode::kNeutral);
  EXPECT_EQ(Status().state(), ElevatorState::HOMING_FAILED);
  EXPECT_FALSE(Status().at_goal());
  Drive(kHighRot, 100ms);
  EXPECT_EQ(Lift().mode, Mode::kNeutral);
}

// Powered on away from the bottom, switch open: no zero yet. The lift moves
// when pushed (velocity -1 rps), so homing does not count as stalled.
class ElevatorHoming : public ElevatorAlone {
 protected:
  void SetUp() override {
    LiftAtBottom(hw, false);
    hw.set_motor("elevator", "lift", 0.0, -1.0);  // the power-on count
    EnableTeleop(sim, hw);
    hw.ideal_motors();
  }
  // The operator disables and re-enables at the Driver Station.
  void DisableAndEnable() {
    hw.set_driver_station(DsPacket(ds::kTeleop));
    sim.step(20ms);
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(10ms);
  }
};

// REQ-SAFE-36
TEST_F(ElevatorHoming, HomesOnALiveTargetThenReachesTheGoal) {
  // Unhomed: a goal drives the lift down, never to a motion-magic goal in
  // the power-on frame.
  EXPECT_FALSE(DriveSawMotionMagic(kHighRot, 100ms));
  ExpectHoming();

  // The switch closes 4 rotations below where the TalonFX powered on: that
  // is height 0, and the goal goes out in the homed frame.
  ReachBottom(-4.0);
  Drive(kHighRot, 30ms);
  EXPECT_EQ(Lift().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(Lift().demand, kHighRot - 4.0);
  EXPECT_EQ(Status().state(), ElevatorState::AT_GOAL);
  EXPECT_DOUBLE_EQ(Status().height_rot(), kHighRot);
  EXPECT_TRUE(Status().at_goal());
}

// A one-tick closed reading mid-travel (EMI on the switch wire) is not the
// bottom: homing keeps driving down and the lift stays unzeroed, so the next
// goal does not go out in a frame zeroed partway up.
// REQ-SAFE-36
TEST_F(ElevatorHoming, OneClosedSampleMidTravelDoesNotZero) {
  Drive(kHighRot, 50ms);
  hw.set_motor("elevator", "lift", -2.0, -1.0);  // partway down
  LiftAtBottom(hw);
  Drive(kHighRot, 10ms);  // one elevator period
  LiftAtBottom(hw, false);
  EXPECT_FALSE(DriveSawMotionMagic(kHighRot, 100ms));
  ExpectHoming();

  ReachBottom(-4.0);  // home_samples consecutive closed readings
  Drive(kHighRot, 30ms);
  EXPECT_EQ(Lift().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(Lift().demand, kHighRot - 4.0);
}

// REQ-SAFE-36
TEST_F(ElevatorAlone, AResetAtHeightRehomesBeforeAnyGoal) {
  Drive(kHighRot, 50ms);
  ASSERT_EQ(Status().state(), ElevatorState::AT_GOAL);
  ASSERT_DOUBLE_EQ(Lift().demand, kHighRot);

  // The TalonFX resets at height: its count restarts at 0 up there. A goal
  // of kHighRot in that frame would drive the carriage 10 rotations past
  // where it is; homing goes down to the switch first.
  hw.reset_device("elevator", "lift");
  hw.set_motor("elevator", "lift", 0.0, -1.0);
  EXPECT_FALSE(DriveSawMotionMagic(kHighRot, 100ms));
  ExpectHoming();

  ReachBottom(-kHighRot);
  Drive(kHighRot, 30ms);
  EXPECT_EQ(Lift().mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(Lift().demand, 0.0);  // kHighRot above the switch
  EXPECT_EQ(Status().state(), ElevatorState::AT_GOAL);
  EXPECT_DOUBLE_EQ(Status().height_rot(), kHighRot);
}

// REQ-SAFE-36
TEST_F(ElevatorHoming, NoHomingMotionWithoutALiveFiniteTarget) {
  sim.step(100ms);  // never sent
  EXPECT_EQ(Lift().mode, Mode::kNeutral);
  EXPECT_EQ(Status().state(), ElevatorState::HOMING);

  Drive(std::numeric_limits<double>::quiet_NaN(), 50ms);
  EXPECT_EQ(Lift().mode, Mode::kNeutral);

  hw.enable(false);
  Drive(kHighRot, 100ms);
  EXPECT_EQ(Lift().mode, Mode::kNeutral);
  EXPECT_EQ(Status().state(), ElevatorState::HOMING);

  hw.enable();
  Drive(kHighRot, 50ms);
  ExpectHoming();
  sim.step(110ms);  // the lane goes stale
  EXPECT_EQ(Lift().mode, Mode::kNeutral);

  Drive(kHighRot, 50ms);
  ExpectHoming();
  ReleaseTarget<ElevatorGoal>(sim, "elevator", "superstructure");
  sim.step(20ms);
  EXPECT_EQ(Lift().mode, Mode::kNeutral);
  EXPECT_EQ(Status().state(), ElevatorState::HOMING);
}

// REQ-SAFE-36
TEST_F(ElevatorHoming, TimeoutStopsHomingUntilTheDriverStationDisables) {
  Drive(kHighRot, kHomingTimeout - 100ms);
  ExpectHoming();
  Drive(kHighRot, 200ms);  // the switch never closes
  EXPECT_EQ(Lift().mode, Mode::kNeutral);
  EXPECT_EQ(Status().state(), ElevatorState::HOMING_FAILED);
  EXPECT_FALSE(Status().at_goal());

  // Neither a live target nor a stale-link glitch (robot_enabled false for
  // 150 ms) restarts it; a Driver Station disable does.
  Drive(kHighRot, 500ms);
  EXPECT_EQ(Lift().mode, Mode::kNeutral);
  hw.lose_states(30);
  Drive(kHighRot, 150ms);
  Drive(kHighRot, 100ms);
  EXPECT_EQ(Lift().mode, Mode::kNeutral);
  EXPECT_EQ(Status().state(), ElevatorState::HOMING_FAILED);
  DisableAndEnable();
  Drive(kHighRot, 30ms);
  ExpectHoming();
}

// REQ-SAFE-36
TEST_F(ElevatorHoming, AStalledLiftStopsHoming) {
  hw.set_motor("elevator", "lift", 0.0, 0.0);  // pushed, but not moving
  Drive(kHighRot, kHomingStall - 100ms);
  ExpectHoming();
  Drive(kHighRot, 200ms);
  EXPECT_EQ(Lift().mode, Mode::kNeutral);
  EXPECT_EQ(Status().state(), ElevatorState::HOMING_FAILED);
}

// config/subsystems/elevator.toml's elevator, as a single-file config with
// one [lift] key replaced.
std::string ElevatorConfig(const std::string& key, const std::string& value) {
  std::vector<std::pair<std::string, std::string>> params{
      {"tolerance_rot", "0.1"},
      {"min_rot", "0.0"},
      {"max_rot", "12.0"},
      {"homing", "\"limit_switch\""},
      {"limit_switch", "\"bottom_limit\""},
      {"homing_v", "-3.0"},
      {"homing_timeout_us", "6000000"},
      {"homing_stall_rps", "0.2"},
      {"homing_stall_us", "500000"},
      {"home_margin_rot", "0.5"},
      {"home_samples", "3"}};
  std::string toml =
      "[robot]\nperiod_us = 5000\ncommand_timeout_us = 100000\n\n"
      "[subsystems.elevator]\nperiod_us = 10000\n\n"
      "[subsystems.elevator.lift]\n";
  for (const auto& [k, v] : params)
    toml += k + " = " + (k == key ? value : v) + "\n";
  return toml + R"(
[subsystems.elevator.motors.lift]
type = "TalonFX"
bus = "rio"
can_id = 2
position_frame = "rezero"

[subsystems.elevator.digital_inputs.bottom_limit]
dio = 1
)";
}

// REQ-SAFE-36
TEST(ElevatorConfig, RefusesUnsafeHomingParameters) {
  {
    auto sim = Sim::FromToml(ElevatorConfig("", ""));
    EXPECT_NO_THROW(sim.add<demo::Elevator>("elevator"));
  }
  const std::vector<std::pair<std::string, std::string>> bad{
      {"homing_v", "0.0"},        {"homing_v", "2.0"},
      {"max_rot", "0.0"},         {"max_rot", "nan"},
      {"homing_timeout_us", "0"}, {"homing_stall_rps", "0.0"},
      {"homing_stall_us", "0"},   {"home_margin_rot", "-0.5"},
      {"home_samples", "0"}};
  for (const auto& [key, value] : bad) {
    SCOPED_TRACE(key + " = " + value);
    auto sim = Sim::FromToml(ElevatorConfig(key, value));
    EXPECT_THROW(sim.add<demo::Elevator>("elevator"), std::invalid_argument);
  }
}

TEST(IntakeAlone, EjectDurationUsesMicroseconds) {
  Sim sim{RobotConfig()};
  sim.add<demo::Intake>("intake");
  SimHardware hw{sim};
  EnableTeleop(sim, hw);
  const auto drive = [&](IntakeCommand command, int ticks) {
    for (int i = 0; i < ticks; ++i) {
      PublishTarget(sim, "intake", "superstructure", IntakeGoal{command});
      sim.step(20ms);
    }
  };
  drive(IntakeCommand::INTAKE, 2);
  drive(IntakeCommand::EJECT, 1);
  const auto initial_status = sim.latest<IntakeStatus>(topics::kIntakeStatus);
  if (!initial_status.has_value()) {
    FAIL() << "Expected initial intake status";
  }
  const auto& initial = *initial_status;
  ASSERT_EQ(initial.state(), IntakeState::EJECTING);

  // eject_us = 500000 is 500 ms, not 500 seconds or half a millisecond.
  drive(IntakeCommand::EJECT, 23);
  const auto ejecting_status = sim.latest<IntakeStatus>(topics::kIntakeStatus);
  if (!ejecting_status.has_value()) {
    FAIL() << "Expected intake status during ejection";
  }
  const auto& ejecting = *ejecting_status;
  EXPECT_EQ(ejecting.state(), IntakeState::EJECTING);
  EXPECT_DOUBLE_EQ(hw.motor_request("intake", "roller").demand, kEjectVolts);
  drive(IntakeCommand::EJECT, 3);
  const auto final_status = sim.latest<IntakeStatus>(topics::kIntakeStatus);
  if (!final_status.has_value()) {
    FAIL() << "Expected final intake status";
  }
  const auto& final = *final_status;
  EXPECT_EQ(final.state(), IntakeState::IDLE);
  EXPECT_EQ(hw.motor_request("intake", "roller").mode, Mode::kNeutral);
}

// The intake with no target: the state machine carries on, the roller is
// neutral.
TEST(IntakeAlone, NeutralWhenTheLaneGoesStale) {
  Sim sim{RobotConfig()};
  sim.add<demo::Intake>("intake");
  SimHardware hw{sim};
  EnableTeleop(sim, hw);
  for (int i = 0; i < 5; ++i) {
    PublishTarget(sim, "intake", "superstructure",
                  IntakeGoal{IntakeCommand::INTAKE});
    sim.step(20ms);
  }
  hw.set_digital_input("intake", "beam_break", true);
  for (int i = 0; i < 5; ++i) {
    PublishTarget(sim, "intake", "superstructure",
                  IntakeGoal{IntakeCommand::INTAKE});
    sim.step(20ms);
  }
  ASSERT_EQ(hw.motor_request("intake", "roller").mode, Mode::kVoltage);
  ASSERT_DOUBLE_EQ(hw.motor_request("intake", "roller").demand, kHoldVolts);

  sim.step(140ms);  // past command_timeout_us (100 ms)
  EXPECT_EQ(hw.motor_request("intake", "roller").mode, Mode::kNeutral);
  const auto status = sim.latest<IntakeStatus>(topics::kIntakeStatus);
  if (!status.has_value()) {
    FAIL() << "no intake status";
  }
  EXPECT_EQ(status->state(), IntakeState::HOLDING);
  EXPECT_TRUE(status->has_piece());
}

}  // namespace
}  // namespace demo
