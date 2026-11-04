#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include "odometry/odometry.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/node/testing.h"

namespace odometry {
namespace {

using namespace std::chrono_literals;
using drivetrain::DrivetrainState;
using talos::testing::Sim;

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

// A drivetrain state with valid modules and heading.
DrivetrainState Valid(std::uint64_t sample_time_ns, double vx_mps,
                      double vy_mps, double omega_radps, double yaw_rad) {
  return {sample_time_ns, true,    true, true, vx_mps, vy_mps,
          omega_radps,    yaw_rad, {},   {},   {},     {}};
}

class OdometryTest : public ::testing::Test {
 protected:
  OdometryState Drive(const DrivetrainState& state) {
    sim.publish(drivetrain::topics::kState, state);
    sim.step(10ms);
    const auto pose = sim.latest<OdometryState>(topics::kState);
    EXPECT_TRUE(pose.has_value());
    return pose.value_or(OdometryState{});
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  Odometry& odometry = sim.add<Odometry>("odometry");
};

TEST_F(OdometryTest, IntegratesFieldRotatedVelocityOverTheSampleClock) {
  OdometryState pose = Drive(Valid(1'000'000'000, 0.0, 0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(pose.x_m(), 0.0);
  EXPECT_DOUBLE_EQ(pose.y_m(), 0.0);

  // 2 m/s forward for 100 ms, facing downfield.
  pose = Drive(Valid(1'100'000'000, 2.0, 0.0, 0.0, 0.0));
  EXPECT_NEAR(pose.x_m(), 0.2, 1e-9);
  EXPECT_NEAR(pose.y_m(), 0.0, 1e-9);

  // 2 m/s forward for 100 ms, facing a quarter turn left: field +y.
  pose = Drive(Valid(1'200'000'000, 2.0, 0.0, 0.0, std::numbers::pi / 2));
  EXPECT_NEAR(pose.x_m(), 0.2, 1e-9);
  EXPECT_NEAR(pose.y_m(), 0.2, 1e-9);
  EXPECT_DOUBLE_EQ(pose.yaw_rot(), 0.25);

  // 1 m/s left for 200 ms, facing downfield.
  pose = Drive(Valid(1'400'000'000, 0.0, 1.0, 0.0, 0.0));
  EXPECT_NEAR(pose.x_m(), 0.2, 1e-9);
  EXPECT_NEAR(pose.y_m(), 0.4, 1e-9);
  EXPECT_EQ(pose.timestamp_ns(), 1'400'000'000u);
}

TEST_F(OdometryTest, PassesTheChassisVelocitiesAndHeadingThrough) {
  const OdometryState pose =
      Drive(Valid(1'020'000'000, 1.5, -0.5, 0.8, std::numbers::pi / 4));
  EXPECT_EQ(pose.timestamp_ns(), 1'020'000'000u);
  EXPECT_DOUBLE_EQ(pose.vx_mps(), 1.5);
  EXPECT_DOUBLE_EQ(pose.vy_mps(), -0.5);
  EXPECT_DOUBLE_EQ(pose.omega_radps(), 0.8);
  EXPECT_DOUBLE_EQ(pose.yaw_rot(), 0.125);
}

TEST_F(OdometryTest, InvalidModulesMoveNothing) {
  Drive(Valid(1'000'000'000, 0.0, 0.0, 0.0, 0.0));
  DrivetrainState invalid = Valid(1'100'000'000, 3.0, 0.0, 0.0, 0.0);
  invalid.mutate_modules_valid(false);
  sim.publish(drivetrain::topics::kState, invalid);
  sim.step(10ms);
  const OdometryState pose = Drive(Valid(1'200'000'000, 0.0, 0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(pose.x_m(), 0.0);
}

TEST_F(OdometryTest, AClockStepBackwardsMovesNothing) {
  Drive(Valid(2'000'000'000, 0.0, 0.0, 0.0, 0.0));
  const OdometryState pose = Drive(Valid(1'000'000'000, 3.0, 0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(pose.x_m(), 0.0);
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
  if (pipe == nullptr) throw std::runtime_error("popen failed");
  Result result{0, {}};
  char buffer[4096];
  while (const std::size_t n = std::fread(buffer, 1, sizeof buffer, pipe)) {
    result.output.append(buffer, n);
  }
  const int status = ::pclose(pipe);
  result.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return result;
}

TEST(OdometryBinary, LoggedSimRunReplaysWithoutDivergence) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string log =
      std::string{tmp != nullptr ? tmp : "/tmp"} + "/odometry_test.tlog";
  const Result run = RunNode("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(run.status, 0) << run.output;
  const Result replay = RunNode("--replay '" + log + "'");
  EXPECT_EQ(replay.status, 0) << replay.output;
  EXPECT_NE(replay.output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay.output;
}

}  // namespace
}  // namespace odometry
