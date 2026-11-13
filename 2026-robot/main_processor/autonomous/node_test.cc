#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include "autonomous/autonomous.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/testing.h"
#include "talOS/memory/shared_memory_ptr.h"
#include "talOS/node/allocation_probe.h"
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

  std::uint16_t match_number_ = 0;  // FMS match number on every Mode packet

  void Mode(unsigned flags, bool red = false) {
    auto packet = talos::testing::DsPacket(flags);
    packet.fms.alliance = red ? ds::Alliance::kRed : ds::Alliance::kBlue;
    packet.fms.match_number = match_number_;
    hw.set_driver_station(packet);
    sim.step(20ms);
  }
  void Pov(int angle, unsigned flags = 0) {
    auto packet = talos::testing::DsPacket(flags);
    packet.joysticks[0].connected = true;
    packet.joysticks[0].pov_count = 1;
    packet.joysticks[0].povs[0] = static_cast<std::int16_t>(angle);
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
  // Commanded feed time (intake lane active at the feed velocity) over
  // `ticks` periods with a ready shooter and a localized pose at the origin.
  talos::Duration FeedWhileReady(int ticks) {
    talos::Duration fed{};
    for (int i = 0; i < ticks; ++i) {
      Shooter(true);
      Pose();
      sim.step(20ms);
      const auto lane = Lane<intake::IntakeTarget>("intake");
      if (lane.active && lane.message.roller_velocity_rps() == -20) fed += 20ms;
    }
    return fed;
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

// REQ-SAFE-31
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

// REQ-SAFE-33
TEST_P(AutonomousPhaseTest, ReenableInSameMatchResumesStepWithoutRefiring) {
  ReachPhase(GetParam());
  const auto generation = talos::testing::DriverStationGeneration(sim);
  Mode(0);
  EXPECT_EQ(Phase(), GetParam());
  ExpectReleased();
  sim.step(120ms);  // Expire any old shooter readiness.
  ExpectReleased();
  Pose();
  Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_NE(talos::testing::DriverStationGeneration(sim), generation);
  // FEED waits in SPINUP for fresh readiness; every other step resumes as is.
  EXPECT_EQ(Phase(), GetParam() == autonomous::Phase::FEED
                         ? autonomous::Phase::SPINUP
                         : GetParam());
  // The preload feeds 500 ms once per match: FEED had fed 20 ms before the
  // disable, and DRIVE and HOLD had finished it.
  const auto fed = FeedWhileReady(50);
  switch (GetParam()) {
    case autonomous::Phase::SPINUP:
      EXPECT_EQ(fed, 500ms);
      break;
    case autonomous::Phase::FEED:
      EXPECT_EQ(fed, 480ms);
      break;
    default:
      EXPECT_EQ(fed, 0ms);
  }
  EXPECT_EQ(Phase(), GetParam() == autonomous::Phase::SPINUP ||
                             GetParam() == autonomous::Phase::FEED
                         ? autonomous::Phase::DRIVE
                         : GetParam());
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

TEST_F(AutonomousTest, MatchDataArrivingAfterArmIsTheSameMatch) {
  StartDrive();  // Armed and fired with no FMS match data.
  match_number_ = 12;
  Mode(0);
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
  sim.step(120ms);  // Expire the old shooter readiness.
  Pose();
  Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
  EXPECT_EQ(FeedWhileReady(50), 0ms);
  match_number_ = 13;  // Then a different match is one.
  Mode(0);
  EXPECT_EQ(Phase(), autonomous::Phase::IDLE);
}

TEST_F(AutonomousTest, ResumedDriveReplansFromCurrentPose) {
  StartDrive();
  Mode(0);
  ExpectReleased();
  // Pushed while disabled: the old trajectory would pull back to its start.
  Pose(3, 2);
  Mode(ds::kEnabled | ds::kAutonomous);
  ASSERT_EQ(Phase(), autonomous::Phase::DRIVE);
  EXPECT_TRUE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
  EXPECT_NEAR(Drive().vx_mps(), 0, 1e-9);
  EXPECT_NEAR(Drive().vy_mps(), 0, 1e-9);
  Pose(3, 2);
  sim.step(20ms);
  EXPECT_GT(Drive().vx_mps(), 0);
  EXPECT_GT(Drive().vy_mps(), 0);
}

// RULES.md 3: a driving tick (trajectory sample, controller, progress
// record, three lane writes, state publish) allocates nothing. The pose is
// published before the window and stays fresh through it.
// REQ-RT-06
TEST_F(AutonomousTest, UpdateDoesNotAllocate) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  StartDrive();
  Pose();
  sim.step(20ms);
  Pose();
  talos::testing::AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(Phase(), autonomous::Phase::DRIVE);
  EXPECT_TRUE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
  EXPECT_GT(Drive().vx_mps(), 0);
}

// What happens between leaving enabled autonomous and re-entering it.
enum class Between {
  kDisabledTeleop,     // FMS teleop follows auto: another match
  kTest,               // test mode: another match
  kAutonomousPeriod,   // disabled longer than the autonomous period
  kNextMatchNumber,    // FMS reports a different match
  kDriverStationLost,  // WPILib reads teleop with no DS attached
  kMatchInfoLost,      // FMS match data missing at re-enable
};

class AutonomousMatchTest : public AutonomousTest,
                            public ::testing::WithParamInterface<Between> {};

TEST_P(AutonomousMatchTest, OnlyANewMatchStartsFromFirstStep) {
  match_number_ = 12;
  StartDrive();  // The preload has fired.
  ASSERT_EQ(Phase(), autonomous::Phase::DRIVE);
  switch (GetParam()) {
    case Between::kDisabledTeleop:
      Mode(ds::kTeleop);
      break;
    case Between::kTest:
      Mode(ds::kTest);
      break;
    case Between::kAutonomousPeriod:
      Mode(0);
      sim.step(20s);
      break;
    case Between::kNextMatchNumber:
      match_number_ = 13;
      Mode(0);
      break;
    case Between::kDriverStationLost: {
      ds::DriverStationData packet{};
      packet.fms.flags = ds::kTeleop;
      packet.fms.match_number = match_number_;
      hw.set_driver_station(packet);
      sim.step(20ms);
      break;
    }
    case Between::kMatchInfoLost:
      match_number_ = 0;
      Mode(0);
      break;
  }
  const bool new_match = GetParam() == Between::kDisabledTeleop ||
                         GetParam() == Between::kTest ||
                         GetParam() == Between::kAutonomousPeriod ||
                         GetParam() == Between::kNextMatchNumber;
  ExpectReleased();
  EXPECT_EQ(Phase(),
            new_match ? autonomous::Phase::IDLE : autonomous::Phase::DRIVE);
  sim.step(120ms);  // Expire the old shooter readiness.
  Pose();
  Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(Phase(),
            new_match ? autonomous::Phase::SPINUP : autonomous::Phase::DRIVE);
  EXPECT_EQ(FeedWhileReady(50), new_match ? 500ms : 0ms);
}

INSTANTIATE_TEST_SUITE_P(
    Gaps, AutonomousMatchTest,
    ::testing::Values(Between::kDisabledTeleop, Between::kTest,
                      Between::kAutonomousPeriod, Between::kNextMatchNumber,
                      Between::kDriverStationLost, Between::kMatchInfoLost));

class AutonomousPoseTest : public AutonomousTest,
                           public ::testing::WithParamInterface<bool> {};

// REQ-SAFE-34
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

// Node test matrix (docs/TESTING.md §7), producer: cases 1, 2, 6 and 12 under
// the names //tools:check_node_matrix looks for.
// matrix: n/a 11 (autonomous maps no operator input to disarm)
// A new generation in the same match resumes the paused step without
// refiring (ReenableInSameMatchResumesStepWithoutRefiring), and a restarted
// process starts INHIBITED (RestartIntoEnabledAutoIsInhibitedUntilRearmed).

// Every lane is released while the robot is not enabled in autonomous:
// before the first enable, disabled in autonomous (FMS between the
// autonomous and teleop periods) and enabled in teleop.
TEST_F(AutonomousTest, DisabledRobotReleasesItsLanes) {
  ExpectReleased();
  Mode(ds::kEnabled | ds::kAutonomous);
  ASSERT_TRUE(Lane<shooter::ShooterTarget>("shooter").active);
  Mode(ds::kAutonomous);
  ExpectReleased();
  Mode(ds::kEnabled | ds::kAutonomous);
  ASSERT_TRUE(Lane<shooter::ShooterTarget>("shooter").active);
  Mode(ds::kEnabled | ds::kTeleop);
  ExpectReleased();
}

// The preload routine on live inputs: it waits for the shooter at speed,
// feeds, then drives to its waypoint, collecting, and holds.
TEST_F(AutonomousTest, SendsAtALiveInput) {
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

TEST_F(AutonomousTest, SpinupDeadlineAbortsUntilANewMatch) {
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

  Mode(ds::kTeleop);
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

// REQ-SAFE-34
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

TEST_F(AutonomousTest, BeamStopsCollection) {
  StartDrive();
  ASSERT_EQ(Intake(), 30);
  sim.publish(intake::topics::kState,
              intake::IntakeState{true, 0, true, 0, intake::Inhibit::DRIVE});
  sim.step(20ms);
  EXPECT_EQ(Intake(), 0);
}

TEST_F(AutonomousTest, DriverStationLossReleasesItsLanes) {
  StartDrive();
  ASSERT_TRUE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
  hw.stop_driver_station();
  sim.step(160ms);  // past command_timeout_us: the Driver Station view is stale
  ExpectReleased();
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
  Pov(90, ds::kTeleop);  // A press held across disable is not a new edge.
  EXPECT_EQ(Routine(), 3);
  Pov(-1, ds::kTeleop);
  Pov(90, ds::kTeleop);
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
  ::shm_unlink(ShmObjectName(kProgressObject).c_str());  // No routine record.
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
      data.joysticks[0].povs[0] = static_cast<std::int16_t>(angle);
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

// One process lifetime of the autonomous node. Lives share a launcher output
// directory, where the progress record lives, but each runs in a session
// directory of its own, as after a launcher restart. Every Sim clock starts
// at 0, standing in for the boot's monotonic clock: a later life must step
// past the earlier one's last tick before it sees the Driver Station.
class Life {
 public:
  // `robot_toml`, when given, replaces the robot config (inline, single-file).
  Life(const std::filesystem::path& output, const char* session,
       const std::string& robot_toml = {})
      : sim_{Make(output / session, robot_toml)}, hw_{sim_} {}

  void Mode(unsigned flags, std::uint16_t match = 0) {
    auto packet = talos::testing::DsPacket(flags);
    packet.fms.match_number = match;
    hw_.set_driver_station(packet);
    sim_.step(20ms);
  }
  void Step(talos::Duration duration) { sim_.step(duration); }
  // Commanded feed time over `ticks` periods with a ready shooter and pose.
  talos::Duration FeedWhileReady(int ticks) {
    talos::Duration fed{};
    for (int i = 0; i < ticks; ++i) {
      sim_.publish(shooter::topics::kState,
                   shooter::ShooterState{0, 60, 60, true, true, true});
      sim_.publish(
          "/pose_estimator/state",
          geometry::PoseEstimate{0, geometry::Pose2d{}, geometry::Twist2d{},
                                 true, true, 0, 0});
      sim_.publish(
          intake::topics::kState,
          intake::IntakeState{true, 0, false, 0, intake::Inhibit::DRIVE});
      sim_.step(20ms);
      const auto lane = Lane<intake::IntakeTarget>("intake");
      if (lane.active && lane.message.roller_velocity_rps() == -20) fed += 20ms;
    }
    return fed;
  }
  autonomous::Phase Phase() const {
    return sim_.latest<AutonomousState>(topics::kState)->phase();
  }
  template <class M>
  talos::TargetEnvelope<M> Lane(const char* name) const {
    return sim_
        .latest<talos::TargetEnvelope<M>>(std::string{"/"} + name +
                                          "/target/auto")
        .value_or(talos::TargetEnvelope<M>{});
  }
  bool AnyLaneActive() const {
    return Lane<drivetrain::ChassisTarget>("drivetrain").active ||
           Lane<shooter::ShooterTarget>("shooter").active ||
           Lane<intake::IntakeTarget>("intake").active;
  }

 private:
  static talos::testing::Sim Make(const std::filesystem::path& session,
                                  const std::string& robot_toml) {
    std::filesystem::create_directories(session);
    auto sim = robot_toml.empty()
                   ? talos::testing::Sim{Runfile("TALOS_TEST_CONFIG")}
                   : talos::testing::Sim::FromToml(robot_toml);
    sim.set_session_dir(session);
    sim.add<Autonomous>("autonomous");
    return sim;
  }
  talos::testing::Sim sim_;
  talos::testing::SimHardware hw_;
};

// Leaves no progress record behind: it would last until reboot.
class RemoveProgressRecord : public ::testing::Environment {
 public:
  void TearDown() override {
    ::shm_unlink(ShmObjectName(kProgressObject).c_str());
  }
};
[[maybe_unused]] ::testing::Environment* const kRemoveProgressRecord =
    ::testing::AddGlobalTestEnvironment(new RemoveProgressRecord);

// A fresh output directory, and no progress record from an earlier test (the
// record is per TEST_TMPDIR namespace, not per directory).
std::filesystem::path OutputDir(const std::string& name) {
  const auto dir = std::filesystem::path{std::getenv("TEST_TMPDIR")} / name;
  std::filesystem::remove_all(dir);
  ::shm_unlink(ShmObjectName(kProgressObject).c_str());
  return dir;
}

// Lets `damage` edit the progress record in place, as a killed writer or a
// stray write would leave it.
void DamageProgress(const std::function<void(ProgressFile&)>& damage) {
  const int fd = ::shm_open(ShmObjectName(kProgressObject).c_str(),
                            O_RDWR | O_CREAT, 0600);
  ASSERT_GE(fd, 0);
  struct stat status {};
  ASSERT_EQ(::fstat(fd, &status), 0);
  if (status.st_size == 0) {
    ASSERT_EQ(::ftruncate(fd, sizeof(ProgressFile)), 0);
  }
  void* memory = ::mmap(nullptr, sizeof(ProgressFile), PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd, 0);
  ::close(fd);
  ASSERT_NE(memory, MAP_FAILED);
  damage(*static_cast<ProgressFile*>(memory));
  ::munmap(memory, sizeof(ProgressFile));
}

class AutonomousRestartTest
    : public ::testing::WithParamInterface<autonomous::Phase>,
      public ::testing::Test {};

// REQ-SAFE-31, REQ-SAFE-33
TEST_P(AutonomousRestartTest, RestartInSameMatchResumesWithoutRefiring) {
  const bool drive = GetParam() == autonomous::Phase::DRIVE;
  const auto output = OutputDir(drive ? "restart-drive" : "restart-feed");
  talos::Duration fed{};
  {
    Life first{output, "1"};
    first.Mode(0);
    first.Mode(ds::kEnabled | ds::kAutonomous, 12);
    fed = first.FeedWhileReady(drive ? 30 : 10);
    ASSERT_EQ(first.Phase(), GetParam());
    ASSERT_EQ(fed, drive ? 500ms : 200ms);
  }  // Killed mid-step.
  Life second{output, "2"};
  second.Step(1s);  // Silent DS: nothing commanded.
  EXPECT_FALSE(second.AnyLaneActive());
  second.Mode(0, 12);  // Disabled: the restored step waits.
  EXPECT_EQ(second.Phase(), GetParam());
  EXPECT_FALSE(second.AnyLaneActive());
  second.Mode(ds::kEnabled | ds::kAutonomous, 12);
  // FEED waits in SPINUP for readiness; DRIVE drives on.
  EXPECT_EQ(second.Phase(),
            drive ? autonomous::Phase::DRIVE : autonomous::Phase::SPINUP);
  // FEED: 500 ms less what the record counts as fed, the 180 ms before the
  // last tick plus that tick's feed, live until stale (100 ms command
  // timeout). DRIVE: the preload is done.
  EXPECT_EQ(second.FeedWhileReady(50), drive ? 0ms : 220ms);
  EXPECT_EQ(second.Phase(), autonomous::Phase::DRIVE);
  EXPECT_TRUE(second.Lane<drivetrain::ChassisTarget>("drivetrain").active);
}

INSTANTIATE_TEST_SUITE_P(Steps, AutonomousRestartTest,
                         ::testing::Values(autonomous::Phase::FEED,
                                           autonomous::Phase::DRIVE));

// REQ-SAFE-33
TEST(AutonomousRestart, CorruptRecordHoldsUntilANewMatch) {
  const auto output = OutputDir("restart-corrupt");
  DamageProgress([](ProgressFile& file) {
    std::memset(static_cast<void*>(&file), 0xab, sizeof file);
  });
  Life life{output, "1"};
  life.Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(life.Phase(), autonomous::Phase::HOLD);
  EXPECT_EQ(life.FeedWhileReady(50), 0ms);
  EXPECT_FALSE(life.AnyLaneActive());
  life.Mode(0);
  life.Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(life.Phase(), autonomous::Phase::HOLD);
  EXPECT_FALSE(life.AnyLaneActive());
  life.Mode(ds::kTeleop);  // Another match.
  life.Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(life.FeedWhileReady(50), 500ms);
}

TEST(AutonomousRestart, TornNewestSlotResumesFromTheWholeOne) {
  const auto output = OutputDir("restart-torn");
  {
    Life first{output, "1"};
    first.Mode(0);
    first.Mode(ds::kEnabled | ds::kAutonomous, 12);
    ASSERT_EQ(first.FeedWhileReady(10), 200ms);
  }
  // Killed while writing the last tick's record (180 ms fed plus a 100 ms
  // live feed): the previous one (160 + 100 ms) is whole.
  DamageProgress([](ProgressFile& file) {
    auto& newest = file.slots[0].sequence > file.slots[1].sequence
                       ? file.slots[0]
                       : file.slots[1];
    newest.progress.feed_elapsed_ns ^= 1;
  });
  Life second{output, "2"};
  second.Step(1s);
  second.Mode(0, 12);
  EXPECT_EQ(second.Phase(), autonomous::Phase::FEED);  // Neither IDLE nor HOLD.
  second.Mode(ds::kEnabled | ds::kAutonomous, 12);
  EXPECT_EQ(second.Phase(), autonomous::Phase::SPINUP);
  EXPECT_EQ(second.FeedWhileReady(50), 240ms);
  EXPECT_EQ(second.Phase(), autonomous::Phase::DRIVE);
}

// The routine list changed between lives (a redeploy mid-match): the record's
// indices name routines this list may not have, so nothing can resume.
TEST(AutonomousRestart, ChangedRoutineListHolds) {
  const auto output = OutputDir("restart-routines");
  {
    Life first{output, "1"};
    first.Mode(0);
    first.Mode(ds::kEnabled | ds::kAutonomous);
    ASSERT_EQ(first.FeedWhileReady(10), 200ms);
  }
  Life second{output, "2", R"(
[robot]
period_us = 20000
command_timeout_us = 100000
[subsystems.mechanism.motors.unused]
can_id = 1
[subsystems.autonomous]
routines = ["shoot_only", "preload"]
)"};
  second.Step(1s);
  second.Mode(0);
  second.Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(second.Phase(), autonomous::Phase::HOLD);
  EXPECT_EQ(second.FeedWhileReady(50), 0ms);
  EXPECT_FALSE(second.AnyLaneActive());
}

// Restarted in another match: the record is ignored, so the process arms
// fresh once it has seen a disabled mode, as without one.
class AutonomousRestartMatchTest : public ::testing::TestWithParam<bool> {};

TEST_P(AutonomousRestartMatchTest, RecordFromAnotherMatchIsIgnored) {
  const bool period_elapsed = GetParam();
  const auto output =
      OutputDir(period_elapsed ? "restart-period" : "restart-match");
  {
    Life first{output, "1"};
    first.Mode(0, 12);
    first.Mode(ds::kEnabled | ds::kAutonomous, 12);
    ASSERT_EQ(first.FeedWhileReady(30), 500ms);
  }
  Life second{output, "2"};
  const std::uint16_t match = period_elapsed ? 12 : 13;
  second.Step(period_elapsed ? 21s : 1s);
  second.Mode(ds::kEnabled | ds::kAutonomous, match);
  EXPECT_EQ(second.Phase(), autonomous::Phase::INHIBITED);
  EXPECT_FALSE(second.AnyLaneActive());
  second.Mode(0, match);
  EXPECT_EQ(second.Phase(), autonomous::Phase::IDLE);
  second.Mode(ds::kEnabled | ds::kAutonomous, match);
  EXPECT_EQ(second.FeedWhileReady(50), 500ms);
}

INSTANTIATE_TEST_SUITE_P(Gaps, AutonomousRestartMatchTest, ::testing::Bool());

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
        "autonomous_period_us = 0", "feed_duration_us = 0",
        "feed_duration_us = -1"}) {
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
