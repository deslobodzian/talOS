#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/devices.h"
#include "talOS/devices/testing.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/node/testing.h"
#include "wrist/wrist.h"

namespace wrist {
namespace {

using namespace std::chrono_literals;
using talos::hardware::Mode;
using talos::testing::AllocationProbe;
using talos::testing::DriverStationGeneration;
using talos::testing::DsPacket;
using talos::testing::PublishTarget;
using talos::testing::ReleaseTarget;
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

// The node test matrix, docs/TESTING.md §7: the case names are the matrix's,
// which //tools:check_node_matrix looks for.
class WristTest : public ::testing::Test {
 protected:
  using Target = Wrist::Target;
  using State = Wrist::State;

  // An enabled teleop robot: the default policy reads the teleop lane.
  void Teleop() {
    hw.enable();
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(10ms);  // the mode is current before any lane arrives
  }
  // Holds a lane the way a producer does: republished every 20 ms.
  void Hold(double demand, std::chrono::milliseconds duration,
            const char* source = "teleop") {
    for (auto t = 0ms; t < duration; t += 20ms) {
      PublishTarget(sim, "wrist", source, Target{demand});
      sim.step(20ms);
    }
  }
  // What the node asked for (the flush sends neutral for an unset motor).
  Mode Request() const { return hw.motor_request("wrist", "wrist_motor").mode; }
  // What the gateway applies, after its enable and expiry gates.
  Mode Applied() const { return hw.applied_motor("wrist", "wrist_motor").mode; }
  State Latest() const {
    const auto state = sim.latest<State>(topics::kState);
    EXPECT_TRUE(state.has_value());
    return state.value_or(State{});
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  Wrist& node = sim.add<Wrist>("wrist");
  SimHardware hw{sim};
};

// REQ-SAFE-07
TEST_F(WristTest, DrivesAtALiveLane) {
  hw.ideal_motors();
  Teleop();
  Hold(30.0, 60ms);

  const auto applied = hw.applied_motor("wrist", "wrist_motor");
  EXPECT_EQ(applied.mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(applied.demand, 30.0);
  const State state = Latest();
  EXPECT_TRUE(state.enabled());
  EXPECT_DOUBLE_EQ(state.velocity_rps(), 30.0);
  EXPECT_TRUE(state.connected());
}

// REQ-SAFE-02
TEST_F(WristTest, DisabledRobotGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Applied(), Mode::kVelocity);
  hw.enable(false);
  Hold(30.0, 60ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Applied(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());  // state keeps publishing
}

// REQ-SAFE-07
TEST_F(WristTest, StaleLaneGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);

  sim.step(150ms);  // past command_timeout_us: the lane is stale
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Applied(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());  // state keeps publishing
}

// REQ-SAFE-07
TEST_F(WristTest, ReleasedLaneGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);

  ReleaseTarget<Target>(sim, "wrist", "teleop");
  sim.step(40ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
}

// REQ-SAFE-07
TEST_F(WristTest, NeverSentGoesNeutral) {
  Teleop();
  sim.step(60ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());
}

// REQ-SAFE-07
TEST_F(WristTest, DriverStationLossGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);

  hw.stop_driver_station();  // the bridge stops forwarding DS packets
  Hold(30.0, 160ms);         // past command_timeout_us: the DS is stale
  EXPECT_EQ(Request(), Mode::kNeutral);
}

// REQ-CONTRACT-25: testdata/robot.toml's wrist plus a neighbor motor that
// stays connected, so losing the wrist's motor is one device lost, as on a
// robot, which only the node's own feedback gate stops (losing every motor
// is the gateway's fault, which disables the robot).
TEST(WristFeedback, DisconnectedFeedbackGoesNeutral) {
  Sim sim = Sim::FromToml(R"(
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.wrist.motors.wrist_motor]
type = "TalonFX"
can_id = 1

[subsystems.neighbor.motors.wheel]
type = "TalonFX"
can_id = 2
)");
  sim.add<Wrist>("wrist");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(10ms);
  const auto hold = [&](std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 20ms) {
      PublishTarget(sim, "wrist", "teleop", Wrist::Target{30.0});
      sim.step(20ms);
    }
  };
  hold(40ms);
  ASSERT_EQ(hw.motor_request("wrist", "wrist_motor").mode, Mode::kVelocity);

  hw.set_connected("wrist", "wrist_motor", false);
  hold(40ms);
  ASSERT_TRUE(talos::robot_enabled(sim.node("wrist")));
  EXPECT_EQ(hw.motor_request("wrist", "wrist_motor").mode, Mode::kNeutral);
  const auto state = sim.latest<Wrist::State>(topics::kState);
  ASSERT_TRUE(state.has_value());
  EXPECT_FALSE(state->connected());
  EXPECT_FALSE(state->enabled());
}

// REQ-SAFE-04
TEST_F(WristTest, NonFiniteDemandGoesNeutral) {
  Teleop();
  Hold(std::numeric_limits<double>::quiet_NaN(), 40ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());
}

// matrix: n/a 9 (the wrist reads no parameters)

// REQ-SAFE-07
TEST_F(WristTest, OtherModesLaneIsIgnored) {
  Teleop();
  Hold(30.0, 40ms, "auto");  // autonomous's lane, live, in teleop
  EXPECT_EQ(Request(), Mode::kNeutral);
  ReleaseTarget<Target>(sim, "wrist", "auto");
  Hold(30.0, 40ms);  // teleop's lane drives in teleop
  ASSERT_EQ(Request(), Mode::kVelocity);

  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kAutonomous));
  sim.step(10ms);
  Hold(30.0, 40ms);  // teleop's lane, live, in autonomous
  EXPECT_EQ(Request(), Mode::kNeutral);
  Hold(20.0, 40ms, "auto");
  const auto request = hw.motor_request("wrist", "wrist_motor");
  EXPECT_EQ(request.mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(request.demand, 20.0);
}

// REQ-SAFE-09
TEST_F(WristTest, EarlierGenerationTargetIsIgnored) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);
  const std::uint64_t before = DriverStationGeneration(sim);
  hw.set_driver_station(DsPacket(ds::kTeleop));  // disabled
  sim.step(40ms);
  // Re-enabled. SimHardware publishes the packet at its next 20 ms hook,
  // which the Sim runs after the node's tick at that instant, so the packet
  // waits for the node's next drain; a target its producer queued under the
  // earlier enable, published before that drain, arrives in the same one.
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(20ms);
  ASSERT_NE(DriverStationGeneration(sim), before);
  PublishTarget(sim, "wrist", "teleop", Target{30.0}, before);
  sim.step(20ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  // The same target stamped with the current generation drives at once.
  PublishTarget(sim, "wrist", "teleop", Target{30.0});
  sim.step(20ms);
  EXPECT_EQ(Request(), Mode::kVelocity);
}

// REQ-RT-06: the steady-state tick (target read, request, flush, state
// publish) allocates nothing. The target is published before the window and
// stays fresh through it (60 ms < command_timeout_us).
TEST_F(WristTest, UpdateDoesNotAllocate) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  Teleop();
  Hold(30.0, 40ms);
  PublishTarget(sim, "wrist", "teleop", Target{30.0});
  AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(Request(), Mode::kVelocity);
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

TEST(WristBinary, LoggedSimRunReplaysWithoutDivergence) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string log =
      std::string{tmp != nullptr ? tmp : "/tmp"} + "/node_test.tlog";
  const Result run = RunNode("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(run.status, 0) << run.output;
  const Result replay = RunNode("--replay '" + log + "'");
  EXPECT_EQ(replay.status, 0) << replay.output;
  EXPECT_NE(replay.output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay.output;
}

}  // namespace
}  // namespace wrist
