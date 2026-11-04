#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "shooter/shooter.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"

namespace shooter {
namespace {

using namespace std::chrono_literals;
using talos::hardware::Mode;
using talos::testing::Sim;
using talos::testing::SimHardware;
namespace ds = talos::driver_station;

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

class ShooterTest : public ::testing::Test {
 protected:
  // An enabled robot in teleop: the policy selects /shooter/target/teleop.
  void Teleop() {
    hw.enable();
    hw.set_driver_station(talos::testing::DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(10ms);
  }
  // Holds a lane the way a producer does: republished every 20 ms.
  void Hold(double rps, std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 20ms) {
      talos::testing::PublishTarget(sim, "shooter", "teleop",
                                    ShooterTarget{rps});
      sim.step(20ms);
    }
  }
  ShooterState State() const {
    const auto state = sim.latest<ShooterState>(topics::kState);
    EXPECT_TRUE(state.has_value());
    return state.value_or(ShooterState{});
  }
  void ExpectNeutral() const {
    EXPECT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kNeutral);
    EXPECT_DOUBLE_EQ(State().target_velocity_rps(), 0.0);
    EXPECT_FALSE(State().at_speed());
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  Shooter& shooter = sim.add<Shooter>("shooter");
  SimHardware hw{sim};
};

TEST_F(ShooterTest, SpinsToAFreshTargetAndReportsAtSpeed) {
  Teleop();
  hw.ideal_motors();
  Hold(80.0, 60ms);

  const auto request = hw.motor_request("shooter", "flywheel");
  EXPECT_EQ(request.mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(request.demand, 80.0);
  const ShooterState state = State();
  EXPECT_DOUBLE_EQ(state.target_velocity_rps(), 80.0);
  EXPECT_DOUBLE_EQ(state.flywheel_velocity_rps(), 80.0);
  EXPECT_TRUE(state.at_speed());
  EXPECT_TRUE(state.enabled());
}

TEST_F(ShooterTest, NotAtSpeedWhileTheFlywheelIsSlow) {
  Teleop();
  hw.set_motor("shooter", "flywheel", 0.0, 50.0);
  Hold(80.0, 60ms);
  EXPECT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kVelocity);
  EXPECT_FALSE(State().at_speed());
  EXPECT_DOUBLE_EQ(State().flywheel_velocity_rps(), 50.0);
}

TEST_F(ShooterTest, NeverSentStaysNeutral) {
  Teleop();
  sim.step(60ms);
  ExpectNeutral();
  EXPECT_TRUE(State().enabled());
}

TEST_F(ShooterTest, StaleLaneGoesNeutral) {
  Teleop();
  Hold(80.0, 40ms);
  ASSERT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kVelocity);
  sim.step(150ms);  // the producer went quiet: the lane is stale
  ExpectNeutral();
}

TEST_F(ShooterTest, ReleasedLaneGoesNeutralAtOnce) {
  Teleop();
  Hold(80.0, 40ms);
  ASSERT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kVelocity);
  talos::testing::ReleaseTarget<ShooterTarget>(sim, "shooter", "teleop");
  sim.step(20ms);
  ExpectNeutral();
}

TEST_F(ShooterTest, DisabledRobotGoesNeutral) {
  Teleop();
  hw.enable(false);
  Hold(80.0, 40ms);
  ExpectNeutral();
  EXPECT_FALSE(State().enabled());
}

TEST_F(ShooterTest, DriverStationLossGoesNeutral) {
  Teleop();
  Hold(80.0, 40ms);
  ASSERT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kVelocity);
  // The Driver Station goes silent; its last packet ages out.
  hw.stop_driver_station();
  Hold(80.0, 160ms);
  ExpectNeutral();
}

TEST_F(ShooterTest, NonFiniteTargetGoesNeutral) {
  Teleop();
  Hold(std::numeric_limits<double>::quiet_NaN(), 40ms);
  ExpectNeutral();
}

TEST_F(ShooterTest, ReportsTheBeamBreak) {
  sim.step(20ms);
  EXPECT_FALSE(State().beam_broken());
  hw.set_digital_input("shooter", "beam_break", true);
  sim.step(20ms);
  EXPECT_TRUE(State().beam_broken());
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

TEST(ShooterBinary, LoggedSimRunReplaysWithoutDivergence) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string log =
      std::string{tmp != nullptr ? tmp : "/tmp"} + "/shooter_test.tlog";
  const Result run = RunNode("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(run.status, 0) << run.output;
  const Result replay = RunNode("--replay '" + log + "'");
  EXPECT_EQ(replay.status, 0) << replay.output;
  EXPECT_NE(replay.output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay.output;
}

}  // namespace
}  // namespace shooter
