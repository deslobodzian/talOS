#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
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
  trajectory.FlipForRed(16.54, 8.21);
  EXPECT_DOUBLE_EQ(trajectory.SampleAt(0).x_m, 16.54);
  EXPECT_DOUBLE_EQ(trajectory.SampleAt(0).y_m, 8.21);
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

  void Mode(unsigned flags, bool red = false) {
    auto packet = talos::testing::DsPacket(flags);
    packet.fms.alliance = red ? ds::Alliance::kRed : ds::Alliance::kBlue;
    hw.set_driver_station(packet);
    sim.step(20ms);
  }
  void Shooter(bool ready) {
    sim.publish(shooter::topics::kState,
                shooter::ShooterState{0, 60, 60, ready, true, true});
  }
  void Pose(double x = 0, double y = 0, double yaw = 0, bool valid = true) {
    sim.publish("/pose_estimator/state",
                geometry::PoseEstimate{0, geometry::Pose2d{x, y, yaw},
                                       geometry::Twist2d{}, valid, 0, 0});
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
  void StartDrive(bool red = false) {
    Mode(ds::kEnabled | ds::kAutonomous, red);
    Shooter(true);
    sim.step(20ms);
    sim.step(480ms);
    Pose();
    sim.step(20ms);
    Pose();
    sim.step(20ms);
  }
};

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

TEST_F(AutonomousTest, ReportsPhaseThroughDisabledAutoAndDisable) {
  // Disabled: the state topic still publishes, reporting the idle phase.
  Mode(0);
  EXPECT_EQ(Phase(), autonomous::Phase::IDLE);

  // Enabled+auto starts the routine: spinup, then feed once the shooter is
  // at speed, then drive, then hold on arrival.
  Mode(ds::kEnabled | ds::kAutonomous);
  EXPECT_EQ(Phase(), autonomous::Phase::SPINUP);
  Shooter(true);
  sim.step(20ms);
  EXPECT_EQ(Phase(), autonomous::Phase::FEED);
  StartDrive();
  for (int i = 0; i < 300; ++i) {
    Pose(6, 4, 0);
    sim.step(20ms);
  }
  EXPECT_EQ(Phase(), autonomous::Phase::HOLD);

  // Losing the Driver Station (stale target) returns the routine to idle.
  hw.stop_driver_station();
  sim.step(160ms);
  EXPECT_EQ(Phase(), autonomous::Phase::IDLE);
  EXPECT_FALSE(Lane<drivetrain::ChassisTarget>("drivetrain").active);
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
  Shooter(true);
  sim.step(20ms);
  EXPECT_EQ(Intake(), -20);
  EXPECT_DOUBLE_EQ(Drive().vx_mps(), 0);
  sim.step(480ms);
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
  Pose(6, 4, 0);
  for (int i = 0; i < 250; ++i) {
    Pose(6, 4, 0);
    sim.step(20ms);
  }
  EXPECT_DOUBLE_EQ(Drive().vx_mps(), 0);
  EXPECT_DOUBLE_EQ(Drive().vy_mps(), 0);
  EXPECT_DOUBLE_EQ(Drive().omega_radps(), 0);
  EXPECT_EQ(Intake(), 0);
}

TEST_F(AutonomousTest, InvalidAndStalePoseStopImmediately) {
  StartDrive();
  EXPECT_GT(Drive().vx_mps(), 0);
  Pose(0, 0, 0, false);
  sim.step(20ms);
  EXPECT_DOUBLE_EQ(Drive().vx_mps(), 0);
  EXPECT_EQ(Intake(), 0);
  Pose();
  sim.step(20ms);
  EXPECT_GT(Drive().vx_mps(), 0);
  sim.step(120ms);
  EXPECT_DOUBLE_EQ(Drive().vx_mps(), 0);
  EXPECT_DOUBLE_EQ(Drive().vy_mps(), 0);
  EXPECT_DOUBLE_EQ(Drive().omega_radps(), 0);
  EXPECT_EQ(Intake(), 0);
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
TEST(AutonomousBinary, DescribesAndReplaysDeterministically) {
  auto describe = RunNode("--describe");
  ASSERT_EQ(describe.status, 0) << describe.output;
  for (const char* topic : {"/drivetrain/target/auto", "/shooter/target/auto",
                            "/intake/target/auto"}) {
    EXPECT_NE(describe.output.find(topic), std::string::npos);
  }
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
