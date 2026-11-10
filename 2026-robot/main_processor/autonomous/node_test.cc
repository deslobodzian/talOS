#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include "autonomous/autonomous.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"

namespace autonomous {
namespace {
using namespace std::chrono_literals;
namespace ds = talos::driver_station;

TEST(TrajectoryTest, InterpolatesMultipleSamplesAndCompletes) {
  const std::array<TrajectorySample, 3> samples{
      {{0, 0, 0, 0, 1, 0, 0}, {1, 1, 0, 0, 1, 0, 0}, {2, 2, 0, 0, 1, 0, 0}}};
  Trajectory trajectory{samples};
  EXPECT_DOUBLE_EQ(trajectory.SampleAt(0.5).x_m, 0.5);
  EXPECT_DOUBLE_EQ(trajectory.SampleAt(1.5).x_m, 1.5);
  EXPECT_DOUBLE_EQ(trajectory.total_time_s(), 2);
  EXPECT_DOUBLE_EQ(trajectory.SampleAt(2).x_m, 2);
  EXPECT_DOUBLE_EQ(trajectory.SampleAt(3).x_m, 2);
  trajectory.FlipForRed(16.541, 8.069);
  EXPECT_DOUBLE_EQ(trajectory.SampleAt(0).x_m, 16.541);
  EXPECT_DOUBLE_EQ(trajectory.SampleAt(0).y_m, 8.069);
  EXPECT_DOUBLE_EQ(trajectory.SampleAt(0).vx_mps, -1);
}

TEST(TrajectoryTest, RampStartsAndEndsAtRest) {
  const auto trajectory = Trajectory::FromPoses(
      geometry::Pose2d{0, 0, 0}, geometry::Pose2d{3, 0, 1}, 2, 3);
  EXPECT_DOUBLE_EQ(trajectory.SampleAt(0).vx_mps, 0);
  EXPECT_NEAR(trajectory.SampleAt(trajectory.total_time_s() / 2).vx_mps, 2,
              1e-9);
  const auto end = trajectory.SampleAt(trajectory.total_time_s());
  EXPECT_DOUBLE_EQ(end.x_m, 3);
  EXPECT_DOUBLE_EQ(end.yaw_rad, 1);
  EXPECT_DOUBLE_EQ(end.vx_mps, 0);
}

TEST(TrajectoryTest, CombinesFeedforwardFeedbackAndRobotFrameLimits) {
  const HolonomicController controller{2, 3, 10, 4};
  const auto target =
      controller.Calculate({0, 2, 1, std::numbers::pi / 2 + 0.2, 1, 2, 0.5},
                           geometry::Pose2d{1, 1, std::numbers::pi / 2});
  EXPECT_NEAR(target.vx_mps(), 2, 1e-9);
  EXPECT_NEAR(target.vy_mps(), -3, 1e-9);
  EXPECT_NEAR(target.omega_radps(), 1.1, 1e-9);
  const auto limited = HolonomicController{1, 1, 2, 3}.Calculate(
      {0, 100, 100, 0, 0, 0, 100}, geometry::Pose2d{});
  EXPECT_NEAR(std::hypot(limited.vx_mps(), limited.vy_mps()), 2, 1e-9);
  EXPECT_DOUBLE_EQ(limited.omega_radps(), 3);
}

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> files(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!files || location == nullptr) throw std::runtime_error(error);
  return files->Rlocation(location);
}

class AutonomousTest : public ::testing::Test {
 protected:
  talos::testing::Sim sim{Runfile("TALOS_TEST_CONFIG")};
  Autonomous& coordinator = sim.add<Autonomous>("autonomous");
  talos::testing::SimHardware hw{sim};

  void SetUp() override { Mode(0); }

  void Mode(unsigned flags, bool red = false) {
    auto packet = talos::testing::DsPacket(flags);
    packet.fms.alliance = red ? ds::Alliance::kRed : ds::Alliance::kBlue;
    hw.set_driver_station(packet);
    sim.step(20ms);
  }
  void Pov(int angle, unsigned flags = 0) {
    auto packet = talos::testing::DsPacket(flags);
    packet.joysticks[0].connected = true;
    packet.joysticks[0].pov_count = 1;
    packet.joysticks[0].povs[0] = angle;
    hw.set_driver_station(packet);
    sim.step(20ms);
  }
  std::uint8_t Routine() {
    return sim.latest<AutonomousState>(topics::kState)->routine_index();
  }
  void Select(unsigned index) {
    Pov(-1);
    for (unsigned i = 0; i < index; ++i) {
      Pov(90);
      Pov(-1);
    }
    ASSERT_EQ(Routine(), index);
  }
  void Shooter(bool ready) {
    sim.publish(shooter::topics::kState,
                shooter::ShooterState{0, 60, 60, ready, true, true});
  }
  void Pose(double x = 0, double y = 0, double yaw = 0, bool valid = true,
            bool localized = true) {
    sim.publish(
        "/pose_estimator/state",
        geometry::PoseEstimate{0, geometry::Pose2d{x, y, yaw},
                               geometry::Twist2d{}, valid, localized, 0, 0});
    sim.publish(intake::topics::kState,
                intake::IntakeState{true, 0, false, 0, intake::Inhibit::DRIVE});
  }
  template <class M>
  talos::TargetEnvelope<M> Lane(const char* name) {
    auto result = sim.latest<talos::TargetEnvelope<M>>(std::string{"/"} + name +
                                                       "/target/auto");
    EXPECT_TRUE(result.has_value());
    return result.value_or(talos::TargetEnvelope<M>{});
  }
  void ExpectReleased() {
    EXPECT_FALSE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
    EXPECT_FALSE(Lane<shooter::ShooterTarget>("shooter").active);
    EXPECT_FALSE(Lane<intake::IntakeTarget>("intake").active);
  }
  drivetrain::ChassisTarget Drive() {
    return Lane<drivetrain::ChassisTarget>("drivetrain").message;
  }
  float Intake() {
    return Lane<intake::IntakeTarget>("intake").message.roller_velocity_rps();
  }
  autonomous::Phase Phase() {
    auto result = sim.latest<autonomous::AutonomousState>("/autonomous/state");
    EXPECT_TRUE(result.has_value());
    return result.value_or(autonomous::AutonomousState{}).phase();
  }
  void ReadyFor(talos::Duration duration) {
    for (auto elapsed = 0ms; elapsed < duration; elapsed += 20ms) {
      Shooter(true);
      sim.step(20ms);
    }
  }
  void StartDrive(bool red = false) {
    Mode(ds::kEnabled | ds::kAutonomous, red);
    Shooter(true);
    sim.step(20ms);
    ReadyFor(480ms);
    Pose();
    sim.step(20ms);
    Pose();
    sim.step(20ms);
  }
  void ReachPhase(autonomous::Phase phase) {
    if (phase == autonomous::Phase::DRIVE || phase == autonomous::Phase::HOLD) {
      StartDrive();
      if (phase == autonomous::Phase::HOLD) {
        for (int i = 0; i < 300; ++i) {
          Pose(6, 4);
          sim.step(20ms);
        }
      }
    } else {
      Mode(ds::kEnabled | ds::kAutonomous);
      if (phase == autonomous::Phase::FEED) {
        Shooter(true);
        sim.step(20ms);
      } else if (phase == autonomous::Phase::ABORTED) {
        sim.step(2020ms);
      }
    }
    ASSERT_EQ(Phase(), phase);
  }
};

class AutonomousPhaseTest
    : public AutonomousTest,
      public ::testing::WithParamInterface<autonomous::Phase> {};

TEST_F(AutonomousTest, RestartIntoEnabledAutoIsInhibitedUntilRearmed) {
  // A fresh process cannot know whether a prior routine already fired.
  talos::testing::Sim restarted{Runfile("TALOS_TEST_CONFIG")};
  restarted.add<Autonomous>("autonomous");
  talos::testing::SimHardware restarted_hw{restarted};
  restarted_hw.set_driver_station(
      talos::testing::DsPacket(ds::kEnabled | ds::kAutonomous));
  for (int i = 0; i < 60; ++i) {
    restarted.publish(shooter::topics::kState,
                      shooter::ShooterState{0, 60, 60, true, true, true});
    restarted.step(20ms);
    const auto state = restarted.latest<AutonomousState>(topics::kState);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->phase(), autonomous::Phase::INHIBITED);
    const auto drive_lane =
        restarted.latest<talos::TargetEnvelope<drivetrain::ChassisTarget>>(
            "/drivetrain/target/auto");
    const auto shooter_lane =
        restarted.latest<talos::TargetEnvelope<shooter::ShooterTarget>>(
            "/shooter/target/auto");
    const auto intake_lane =
        restarted.latest<talos::TargetEnvelope<intake::IntakeTarget>>(
            "/intake/target/auto");
    ASSERT_TRUE(drive_lane.has_value());
    ASSERT_TRUE(shooter_lane.has_value());
    ASSERT_TRUE(intake_lane.has_value());
    EXPECT_FALSE(drive_lane->active);
    EXPECT_FALSE(shooter_lane->active);
    EXPECT_FALSE(intake_lane->active);
  }
  restarted_hw.stop_driver_station();
  restarted.step(200ms);
  EXPECT_EQ(restarted.latest<AutonomousState>(topics::kState)->phase(),
            autonomous::Phase::INHIBITED);
  restarted_hw.set_driver_station(talos::testing::DsPacket(0));
  restarted.step(20ms);
  EXPECT_EQ(restarted.latest<AutonomousState>(topics::kState)->phase(),
            autonomous::Phase::IDLE);
  restarted_hw.set_driver_station(
      talos::testing::DsPacket(ds::kEnabled | ds::kAutonomous));
  restarted.step(20ms);
  EXPECT_EQ(restarted.latest<AutonomousState>(topics::kState)->phase(),
            autonomous::Phase::SPINUP);
}

TEST_P(AutonomousPhaseTest, StaleDriverStationResumesSameGenerationAndPhase) {
  ReachPhase(GetParam());
  const auto generation = talos::testing::DriverStationGeneration(sim);
  hw.stop_driver_station();
  for (int i = 0; i < 8; ++i) {
    Shooter(GetParam() == autonomous::Phase::FEED);
    sim.step(20ms);
  }
  EXPECT_EQ(Phase(), GetParam());
  ExpectReleased();
  Pose();
  Shooter(GetParam() == autonomous::Phase::FEED);
  Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(talos::testing::DriverStationGeneration(sim), generation);
  EXPECT_EQ(Phase(), GetParam());
  if (GetParam() == autonomous::Phase::HOLD ||
      GetParam() == autonomous::Phase::ABORTED) {
    ExpectReleased();
  } else if (GetParam() == autonomous::Phase::DRIVE) {
    EXPECT_EQ(Intake(), 30);
    EXPECT_DOUBLE_EQ(
        Lane<shooter::ShooterTarget>("shooter").message.target_velocity_rps(),
        0);
  }
}

TEST_F(AutonomousTest, DriverStationOutageDoesNotCountReleasedFeedTime) {
  Mode(ds::kEnabled | ds::kAutonomous);
  Shooter(true);
  sim.step(20ms);
  ASSERT_EQ(Phase(), autonomous::Phase::FEED);
  auto feed_time = 0ns;
  hw.stop_driver_station();
  for (int i = 0; i < 15; ++i) {
    const auto intake_lane = Lane<intake::IntakeTarget>("intake");
    if (intake_lane.active &&
        intake_lane.message.roller_velocity_rps() == -20) {
      feed_time += 20ms;
    }
    Shooter(true);
    sim.step(20ms);
  }
  EXPECT_EQ(Phase(), autonomous::Phase::FEED);
  ExpectReleased();
  Shooter(true);
  Mode(ds::kEnabled | ds::kAutonomous);
  ASSERT_EQ(Phase(), autonomous::Phase::FEED);
  for (; feed_time + 20ms < 500ms; feed_time += 20ms) {
    ReadyFor(20ms);
  }
  EXPECT_EQ(Phase(), autonomous::Phase::FEED);
  ReadyFor(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
}

TEST_P(AutonomousPhaseTest, NewModeGenerationStartsFreshRoutine) {
  ReachPhase(GetParam());
  const auto generation = talos::testing::DriverStationGeneration(sim);
  Mode(0);
  EXPECT_EQ(Phase(), autonomous::Phase::IDLE);
  sim.step(120ms);  // Expire any old shooter readiness.
  Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_NE(talos::testing::DriverStationGeneration(sim), generation);
  EXPECT_EQ(Phase(), autonomous::Phase::SPINUP);
  Shooter(true);
  sim.step(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::FEED);
  ReadyFor(480ms);
  EXPECT_EQ(Phase(), autonomous::Phase::FEED);
  ReadyFor(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
}

TEST_P(AutonomousPhaseTest, DriverStationOutageDoesNotExtendWallDeadline) {
  ReachPhase(GetParam());
  hw.stop_driver_station();
  sim.step(11s);
  EXPECT_EQ(Phase(), GetParam() == autonomous::Phase::HOLD
                         ? autonomous::Phase::HOLD
                         : autonomous::Phase::ABORTED);
  ExpectReleased();
  Shooter(true);
  Pose();
  Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(Phase(), GetParam() == autonomous::Phase::HOLD
                         ? autonomous::Phase::HOLD
                         : autonomous::Phase::ABORTED);
  ExpectReleased();
}

INSTANTIATE_TEST_SUITE_P(RoutinePhases, AutonomousPhaseTest,
                         ::testing::Values(autonomous::Phase::SPINUP,
                                           autonomous::Phase::FEED,
                                           autonomous::Phase::DRIVE,
                                           autonomous::Phase::HOLD,
                                           autonomous::Phase::ABORTED));

TEST_F(AutonomousTest, ShooterDropoutStopsFeedAndPreservesAccumulatedTime) {
  Mode(ds::kEnabled | ds::kAutonomous);
  Shooter(true);
  sim.step(20ms);
  ReadyFor(200ms);
  Shooter(false);
  sim.step(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::SPINUP);
  EXPECT_EQ(Intake(), 0);
  sim.step(200ms);
  EXPECT_EQ(Phase(), autonomous::Phase::SPINUP);
  EXPECT_EQ(Intake(), 0);
  Shooter(true);
  sim.step(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::FEED);
  EXPECT_EQ(Intake(), -20);
  ReadyFor(260ms);
  EXPECT_EQ(Phase(), autonomous::Phase::FEED);
  ReadyFor(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
}

TEST_F(AutonomousTest, StaleAndWrongVelocityShooterBothInterruptFeed) {
  Mode(ds::kEnabled | ds::kAutonomous);
  Shooter(true);
  sim.step(20ms);
  sim.step(120ms);
  EXPECT_EQ(Phase(), autonomous::Phase::SPINUP);
  EXPECT_EQ(Intake(), 0);
  Shooter(true);
  sim.step(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::FEED);
  sim.publish(shooter::topics::kState,
              shooter::ShooterState{0, 30, 30, true, true, true});
  sim.step(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::SPINUP);
  EXPECT_EQ(Intake(), 0);
}

TEST_F(AutonomousTest, ShooterFlappingCannotResetFirstSpinupDeadline) {
  Mode(ds::kEnabled | ds::kAutonomous);
  sim.step(1500ms);
  Shooter(true);
  sim.step(20ms);
  ReadyFor(100ms);
  Shooter(false);
  sim.step(20ms);
  Shooter(true);
  sim.step(20ms);
  ReadyFor(100ms);
  Shooter(false);
  sim.step(20ms);
  sim.step(300ms);
  EXPECT_EQ(Phase(), autonomous::Phase::ABORTED);
  ExpectReleased();
}

class AutonomousPoseTest : public AutonomousTest,
                           public ::testing::WithParamInterface<bool> {};

TEST_P(AutonomousPoseTest, UnusablePosePausesTrajectoryUntilRecovery) {
  StartDrive();
  const auto before = Drive();
  Pose(0, 0, 0, GetParam(), !GetParam());
  sim.step(20ms);
  EXPECT_FALSE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
  EXPECT_DOUBLE_EQ(Drive().vx_mps(), 0);
  EXPECT_DOUBLE_EQ(Drive().vy_mps(), 0);
  EXPECT_DOUBLE_EQ(Drive().omega_radps(), 0);
  EXPECT_EQ(Intake(), 0);
  sim.step(5s);
  Pose();
  sim.step(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
  EXPECT_TRUE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
  EXPECT_DOUBLE_EQ(Drive().vx_mps(), before.vx_mps());
  EXPECT_DOUBLE_EQ(Drive().vy_mps(), before.vy_mps());
  Pose();
  sim.step(20ms);
  EXPECT_GT(Drive().vx_mps(), before.vx_mps());
}

INSTANTIATE_TEST_SUITE_P(PoseValidity, AutonomousPoseTest, ::testing::Bool());

TEST_F(AutonomousTest, DisabledAndTeleopReleaseAllLanes) {
  Mode(0);
  EXPECT_FALSE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
  EXPECT_FALSE(Lane<shooter::ShooterTarget>("shooter").active);
  EXPECT_FALSE(Lane<intake::IntakeTarget>("intake").active);
  Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_TRUE(Lane<shooter::ShooterTarget>("shooter").active);
  Mode(ds::kEnabled | ds::kTeleop);
  EXPECT_FALSE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
  EXPECT_FALSE(Lane<shooter::ShooterTarget>("shooter").active);
  EXPECT_FALSE(Lane<intake::IntakeTarget>("intake").active);
}

TEST_F(AutonomousTest, WaitsForSpeedFeedsThenRotatesDriveAndHolds) {
  Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_DOUBLE_EQ(
      Lane<shooter::ShooterTarget>("shooter").message.target_velocity_rps(),
      60);
  sim.step(200ms);
  EXPECT_EQ(Intake(), 0);
  Shooter(false);
  sim.step(20ms);
  EXPECT_EQ(Intake(), 0);
  sim.publish(shooter::topics::kState,
              shooter::ShooterState{0, 30, 30, true, true, true});
  sim.step(20ms);
  EXPECT_EQ(Intake(), 0);
  Shooter(true);
  sim.step(20ms);
  EXPECT_EQ(Intake(), -20);
  EXPECT_EQ(Phase(), autonomous::Phase::FEED);
  EXPECT_DOUBLE_EQ(Drive().vx_mps(), 0);
  ReadyFor(480ms);
  EXPECT_EQ(Intake(), -20);
  Pose(0, 0, std::numbers::pi / 2);
  sim.step(20ms);
  EXPECT_NEAR(Drive().vx_mps(), 0, 1e-9);
  EXPECT_DOUBLE_EQ(Drive().vy_mps(), 0);
  Pose(0, 0, std::numbers::pi / 2);
  sim.step(20ms);
  EXPECT_LT(Drive().vy_mps(), 0);
  EXPECT_LT(Drive().omega_radps(), 0);
  EXPECT_EQ(Intake(), 30);
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
  Pose(6, 4, 0);
  for (int i = 0; i < 250; ++i) {
    Pose(6, 4, 0);
    sim.step(20ms);
  }
  EXPECT_DOUBLE_EQ(Drive().vx_mps(), 0);
  EXPECT_DOUBLE_EQ(Drive().vy_mps(), 0);
  EXPECT_DOUBLE_EQ(Drive().omega_radps(), 0);
  EXPECT_EQ(Intake(), 0);
  EXPECT_EQ(Phase(), autonomous::Phase::HOLD);
  ExpectReleased();
}

TEST_F(AutonomousTest, SpinupDeadlineAbortsAndRequiresLeavingEnabledAuto) {
  Mode(ds::kEnabled | ds::kAutonomous);
  Shooter(false);
  sim.step(1900ms);
  EXPECT_EQ(Phase(), autonomous::Phase::SPINUP);
  sim.step(120ms);
  EXPECT_EQ(Phase(), autonomous::Phase::ABORTED);
  ExpectReleased();

  Shooter(true);
  Pose();
  sim.step(100ms);
  EXPECT_EQ(Phase(), autonomous::Phase::ABORTED);
  ExpectReleased();

  Mode(0);
  EXPECT_EQ(Phase(), autonomous::Phase::IDLE);
  ExpectReleased();
  sim.step(120ms);  // Let the previous ready report expire.
  Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(Phase(), autonomous::Phase::SPINUP);
  EXPECT_TRUE(Lane<shooter::ShooterTarget>("shooter").active);
  Shooter(true);
  sim.step(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::FEED);
}

TEST_F(AutonomousTest, UnreachableWaypointAbortsAtDriveDeadline) {
  StartDrive();
  for (int i = 0; i < 490; ++i) {
    Pose();
    sim.step(20ms);
  }
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
  Pose();
  sim.step(200ms);
  EXPECT_EQ(Phase(), autonomous::Phase::ABORTED);
  ExpectReleased();
  Pose(6, 4, 0);
  sim.step(100ms);
  EXPECT_EQ(Phase(), autonomous::Phase::ABORTED);
  ExpectReleased();
}

TEST_F(AutonomousTest, PoseOutageDoesNotExtendDriveDeadline) {
  StartDrive();
  sim.step(10s);
  EXPECT_EQ(Phase(), autonomous::Phase::ABORTED);
  ExpectReleased();
}

TEST_F(AutonomousTest, StalePosePausesTrajectoryUntilRecovery) {
  StartDrive();
  auto last_active = Drive();
  for (int i = 0; i < 6; ++i) {
    sim.step(20ms);
    if (Lane<drivetrain::ChassisTarget>("drivetrain").active) {
      last_active = Drive();
    }
  }
  EXPECT_FALSE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
  EXPECT_DOUBLE_EQ(Drive().vx_mps(), 0);
  EXPECT_DOUBLE_EQ(Drive().vy_mps(), 0);
  EXPECT_DOUBLE_EQ(Drive().omega_radps(), 0);
  EXPECT_EQ(Intake(), 0);
  sim.step(5s);
  Pose();
  sim.step(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
  EXPECT_TRUE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
  EXPECT_DOUBLE_EQ(Drive().vx_mps(), last_active.vx_mps());
  EXPECT_DOUBLE_EQ(Drive().vy_mps(), last_active.vy_mps());
}

TEST_F(AutonomousTest, RedRotatesWaypointAndHeading) {
  StartDrive(true);
  EXPECT_GT(Drive().vx_mps(), 0);
  EXPECT_GT(Drive().vy_mps(), 0);
  EXPECT_GT(std::abs(Drive().omega_radps()), 0);
}

TEST_F(AutonomousTest, BeamStopsCollectionAndDriverStationLossReleases) {
  StartDrive();
  sim.publish(intake::topics::kState,
              intake::IntakeState{true, 0, true, 0, intake::Inhibit::DRIVE});
  sim.step(20ms);
  EXPECT_EQ(Intake(), 0);
  hw.stop_driver_station();
  sim.step(160ms);
  EXPECT_FALSE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
  EXPECT_FALSE(Lane<shooter::ShooterTarget>("shooter").active);
  EXPECT_FALSE(Lane<intake::IntakeTarget>("intake").active);
}

TEST_F(AutonomousTest, SelectionEdgesAreSeededDisabledOnlyAndLatchedAtArm) {
  Pov(90);  // Held on the first connected snapshot is not a press.
  EXPECT_EQ(Routine(), 0);
  Pov(90);
  EXPECT_EQ(Routine(), 0);
  Pov(-1);
  Pov(90);
  EXPECT_EQ(Routine(), 1);
  Pov(-1);
  Pov(270);
  EXPECT_EQ(Routine(), 0);
  Pov(-1);
  Pov(270);
  EXPECT_EQ(Routine(), 3);
  Pov(-1, ds::kEnabled | ds::kTeleop);
  Pov(90, ds::kEnabled | ds::kTeleop);
  EXPECT_EQ(Routine(), 3);
  Pov(-1, ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(Routine(), 3);
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
  Pov(90, ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(Routine(), 3);
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
  Pov(90);  // A press held across disable is not a new edge.
  EXPECT_EQ(Routine(), 3);
  Pov(-1);
  Pov(90);
  EXPECT_EQ(Routine(), 0);
}

// FMS holds the robot disabled in autonomous mode before the match starts,
// which is exactly when the drive team picks the routine.
TEST_F(AutonomousTest, SelectionWorksInFmsPreMatchAndIgnoresReconnectHeldPov) {
  Pov(-1, ds::kAutonomous);
  Pov(90, ds::kAutonomous);
  EXPECT_EQ(Routine(), 1);
  hw.stop_driver_station();
  sim.step(160ms);
  Pov(90, ds::kAutonomous);  // Held across a DS reconnect is not a press.
  EXPECT_EQ(Routine(), 1);
  Pov(-1, ds::kAutonomous);
  Pov(90, ds::kAutonomous);
  EXPECT_EQ(Routine(), 2);
}

class SelectedRoutineTest : public AutonomousTest,
                            public ::testing::WithParamInterface<unsigned> {};

TEST_P(SelectedRoutineTest, DisabledAndStaleDriverStationReleaseEveryRoutine) {
  Select(GetParam());
  ExpectReleased();
  Mode(ds::kEnabled | ds::kAutonomous);
  Shooter(true);
  Pose();
  sim.step(20ms);
  EXPECT_EQ(Routine(), GetParam());
  hw.stop_driver_station();
  sim.step(160ms);
  ExpectReleased();
  Mode(0);
  ExpectReleased();
}

TEST_P(SelectedRoutineTest, FeedbackInterlocksAndRoutineOutputs) {
  Select(GetParam());
  Mode(ds::kEnabled | ds::kAutonomous);
  if (GetParam() == 1) {
    EXPECT_EQ(Phase(), autonomous::Phase::HOLD);
    for (int i = 0; i < 800; ++i) {
      Shooter(true);
      Pose();
      sim.step(20ms);
      ExpectReleased();
    }
  } else if (GetParam() == 3) {
    EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
    EXPECT_FALSE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
    EXPECT_FALSE(Lane<shooter::ShooterTarget>("shooter").active);
    EXPECT_FALSE(Lane<intake::IntakeTarget>("intake").active);
    Pose();
    sim.step(20ms);
    Pose();
    sim.step(20ms);
    EXPECT_GT(Drive().vx_mps(), 0);
    Pose(0, 0, 0, true, false);
    sim.step(20ms);
    EXPECT_FALSE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
    Pose();
    sim.step(20ms);
    sim.step(120ms);
    EXPECT_FALSE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
    EXPECT_FALSE(Lane<shooter::ShooterTarget>("shooter").active);
    EXPECT_FALSE(Lane<intake::IntakeTarget>("intake").active);
  } else {
    EXPECT_EQ(Phase(), autonomous::Phase::SPINUP);
    EXPECT_EQ(Intake(), 0);
    Shooter(true);
    sim.step(20ms);
    EXPECT_EQ(Intake(), -20);
    sim.step(120ms);
    EXPECT_EQ(Phase(), autonomous::Phase::SPINUP);
    EXPECT_EQ(Intake(), 0);
    Shooter(true);
    sim.step(20ms);
    ReadyFor(500ms);
    EXPECT_EQ(Phase(), GetParam() == 2 ? autonomous::Phase::HOLD
                                       : autonomous::Phase::DRIVE);
    if (GetParam() == 2) {
      ExpectReleased();
      Shooter(true);
      Pose();
      sim.step(1s);
      EXPECT_EQ(Phase(), autonomous::Phase::HOLD);
      ExpectReleased();
    }
  }
}
INSTANTIATE_TEST_SUITE_P(Routines, SelectedRoutineTest,
                         ::testing::Values(0U, 1U, 2U, 3U));

TEST(AutonomousSelection, PersistsAcrossRestartAndCorruptionUsesDefault) {
  const auto dir = std::filesystem::path{std::getenv("TEST_TMPDIR")} /
                   "autonomous-selection-test";
  std::filesystem::create_directories(dir);
  std::filesystem::remove(dir / "autonomous-selection");
  const auto run = [&](bool select, bool enabled) {
    talos::testing::Sim sim{Runfile("TALOS_TEST_CONFIG")};
    sim.set_session_dir(dir);
    sim.add<Autonomous>("autonomous");
    talos::testing::SimHardware hw{sim};
    const auto packet = [&](int angle) {
      auto data = talos::testing::DsPacket(
          enabled ? ds::kEnabled | ds::kAutonomous : 0);
      data.joysticks[0].connected = true;
      data.joysticks[0].pov_count = 1;
      data.joysticks[0].povs[0] = angle;
      hw.set_driver_station(data);
      sim.step(20ms);
    };
    packet(-1);
    if (select) packet(270);  // preload -> taxi, persisted as index 3.
    const auto state = sim.latest<AutonomousState>(topics::kState);
    EXPECT_EQ(state->routine_index(), select || enabled ? 3 : 0);
    if (enabled) {
      EXPECT_EQ(state->phase(), autonomous::Phase::INHIBITED);
      EXPECT_FALSE(sim.latest<talos::TargetEnvelope<shooter::ShooterTarget>>(
                          "/shooter/target/auto")
                       ->active);
    }
  };
  run(true, false);
  run(false, true);
  {
    std::ofstream corrupt{dir / "autonomous-selection", std::ios::trunc};
    corrupt << "corrupt";
  }
  run(false, false);
}

TEST(AutonomousParameters, RejectsInvalidRoutineAndSelectionConfiguration) {
  for (const char* setting :
       {"routines = []", "routines = ['missing']",
        "routines = ['none', 'none']", "default_routine = 'missing'",
        "selection_port_index = 6", "selection_pov_index = -1",
        "selection_next_direction_index = 8",
        "selection_previous_direction_index = 2",
        "routine.taxi.target_x_m = nan",
        "routine.shoot_only.spinup_timeout_s = 0.0"}) {
    SCOPED_TRACE(setting);
    auto sim = talos::testing::Sim::FromToml(std::string{R"(
[robot]
period_us = 20000
command_timeout_us = 100000
[subsystems.mechanism.motors.unused]
can_id = 1
[subsystems.autonomous]
)"} + setting);
    EXPECT_THROW(sim.add<Autonomous>("autonomous"), std::invalid_argument);
  }
}

TEST(AutonomousParameters, RejectsNonpositiveAndNonfinitePhaseDurations) {
  for (const char* setting :
       {"spinup_timeout_s = 0.0", "spinup_timeout_s = -1.0",
        "spinup_timeout_s = nan", "spinup_timeout_s = inf",
        "drive_timeout_s = 0.0", "drive_timeout_s = -1.0",
        "drive_timeout_s = nan", "drive_timeout_s = inf",
        "feed_duration_us = 0", "feed_duration_us = -1"}) {
    SCOPED_TRACE(setting);
    auto sim = talos::testing::Sim::FromToml(std::string{R"(
[robot]
period_us = 20000
command_timeout_us = 100000
[subsystems.mechanism.motors.unused]
can_id = 1
[subsystems.autonomous]
)"} + setting);
    EXPECT_THROW(sim.add<Autonomous>("autonomous"), std::invalid_argument);
  }
}

struct Result {
  int status;
  std::string output;
};
Result RunNode(const std::string& args) {
  const std::string command = "'" + Runfile("TALOS_TEST_NODE") + "' " + args +
                              " --config '" + Runfile("TALOS_TEST_CONFIG") +
                              "' 2>&1";
  FILE* pipe = ::popen(command.c_str(), "r");
  if (!pipe) throw std::runtime_error("popen failed");
  Result result{0, {}};
  char buffer[4096];
  while (const auto n = std::fread(buffer, 1, sizeof buffer, pipe)) {
    result.output.append(buffer, n);
  }
  const int status = ::pclose(pipe);
  result.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return result;
}
TEST(AutonomousBinary, ReplaysDeterministically) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string log =
      std::string{tmp != nullptr ? tmp : "/tmp"} + "/autonomous.tlog";
  auto run = RunNode("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(run.status, 0) << run.output;
  auto replay = RunNode("--replay '" + log + "'");
  EXPECT_EQ(replay.status, 0) << replay.output;
  EXPECT_NE(replay.output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay.output;
}
}  // namespace
}  // namespace autonomous
