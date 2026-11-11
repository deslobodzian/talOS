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

#include "position_cpp/position_cpp.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/testing.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/node/testing.h"

namespace position_cpp {
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
class PositionCppTest : public ::testing::Test {
 protected:
  using Target = PositionCpp::Target;
  using State = PositionCpp::State;

  // An enabled teleop robot: the default policy reads the teleop lane.
  void Teleop() {
    hw.enable();
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    // The mode is current before any lane arrives, and a motor on its home
    // switch has zeroed (home_samples ticks).
    sim.step(60ms);
  }
  // Holds a lane the way a producer does: republished every 20 ms.
  void Hold(double demand, std::chrono::milliseconds duration,
            const char* source = "teleop") {
    for (auto t = 0ms; t < duration; t += 20ms) {
      PublishTarget(sim, "position_cpp", source, Target{demand});
      sim.step(20ms);
    }
  }
  // What the node asked for (the flush sends neutral for an unset motor).
  Mode Request() const {
    return hw.motor_request("position_cpp", "position_cpp_motor").mode;
  }
  // What the gateway applies, after its enable and expiry gates.
  Mode Applied() const {
    return hw.applied_motor("position_cpp", "position_cpp_motor").mode;
  }
  State Latest() const {
    const auto state = sim.latest<State>(topics::kState);
    EXPECT_TRUE(state.has_value());
    return state.value_or(State{});
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  PositionCpp& node = sim.add<PositionCpp>("position_cpp");
  SimHardware hw{sim};

  // Starts on its home switch, so the motor zeroes without moving.
  PositionCppTest() { SetHome(true); }
  // Normally open to ground behind the input's pull-up: closed reads false.
  void SetHome(bool closed) {
    hw.set_digital_input("position_cpp", "position_cpp_home", !closed);
  }
  // Powered on off the switch, unzeroed, and moving when pushed.
  void StartOffHome() {
    SetHome(false);
    hw.set_motor("position_cpp", "position_cpp_motor", 0.0, -1.0);
  }
};

// REQ-SAFE-07
TEST_F(PositionCppTest, DrivesAtALiveLane) {
  hw.ideal_motors();
  Teleop();
  SetHome(false);  // zeroed on the switch, it leaves it
  Hold(30.0, 60ms);

  const auto request = hw.applied_motor("position_cpp", "position_cpp_motor");
  EXPECT_EQ(request.mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(request.demand, 30.0);
  const State state = Latest();
  EXPECT_TRUE(state.enabled());
  EXPECT_DOUBLE_EQ(state.position_rot(), 30.0);
  EXPECT_TRUE(state.connected());
}

// REQ-SAFE-02
TEST_F(PositionCppTest, DisabledRobotGoesNeutral) {
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  hw.enable(false);
  sim.step(10ms);
  Hold(30.0, 60ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Applied(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());  // state keeps publishing
}

// REQ-SAFE-07
TEST_F(PositionCppTest, StaleLaneGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kMotionMagic);

  sim.step(150ms);  // past command_timeout_us: the lane is stale
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Applied(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());  // state keeps publishing
}

// REQ-SAFE-07
TEST_F(PositionCppTest, ReleasedLaneGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kMotionMagic);

  ReleaseTarget<Target>(sim, "position_cpp", "teleop");
  sim.step(40ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
}

// REQ-SAFE-07
TEST_F(PositionCppTest, NeverSentGoesNeutral) {
  Teleop();
  sim.step(60ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());
}

// REQ-SAFE-07
TEST_F(PositionCppTest, DriverStationLossGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kMotionMagic);

  hw.stop_driver_station();  // the bridge stops forwarding DS packets
  Hold(30.0, 160ms);         // past command_timeout_us: the DS is stale
  EXPECT_EQ(Request(), Mode::kNeutral);
}

// REQ-CONTRACT-25
TEST_F(PositionCppTest, DisconnectedFeedbackGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kMotionMagic);

  hw.set_connected("position_cpp", "position_cpp_motor", false);
  Hold(30.0, 40ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  const State state = Latest();
  EXPECT_FALSE(state.connected());
  EXPECT_FALSE(state.enabled());
}

// REQ-SAFE-04
TEST_F(PositionCppTest, NonFiniteDemandGoesNeutral) {
  Teleop();
  Hold(std::numeric_limits<double>::quiet_NaN(), 40ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());
}

// REQ-CONTRACT-18
TEST(PositionCppParameters, BadParameterFailsConstruction) {
  // testdata/position_cpp.toml's values; each row drops or replaces one.
  const char* const valid[][2] = {
      {"max_position_rot", "100.0"},   {"homing_v", "-1.0"},
      {"homing_timeout_us", "500000"}, {"homing_stall_rps", "0.1"},
      {"homing_stall_us", "200000"},   {"home_margin_rot", "0.5"},
  };
  const struct {
    const char* key;
    const char* value;  // nullptr: the key is missing
  } rows[] = {
      {"max_position_rot", nullptr}, {"max_position_rot", "0.0"},
      {"max_position_rot", "inf"},   {"homing_v", nullptr},
      {"homing_v", "0.0"},           {"homing_timeout_us", nullptr},
      {"homing_timeout_us", "0"},    {"homing_stall_rps", nullptr},
      {"homing_stall_rps", "nan"},   {"homing_stall_us", nullptr},
      {"homing_stall_us", "-1"},     {"home_margin_rot", nullptr},
      {"home_margin_rot", "0.0"},    {"home_samples", "0"},
  };
  for (const auto& row : rows) {
    const std::string setting = std::string{row.key} + " = " +
                                (row.value != nullptr ? row.value : "missing");
    SCOPED_TRACE(setting);
    std::string toml =
        "[robot]\nperiod_us = 20000\ncommand_timeout_us = 100000\n"
        "[subsystems.position_cpp]\n";
    for (const auto& [key, value] : valid) {
      if (std::string{key} != row.key) {
        toml += std::string{key} + " = " + value + "\n";
      }
    }
    if (row.value != nullptr) toml += setting + "\n";
    toml +=
        "[subsystems.position_cpp.motors.position_cpp_motor]\n"
        "type = \"TalonFX\"\ncan_id = 1\nposition_frame = \"rezero\"\n"
        "[subsystems.position_cpp.digital_inputs.position_cpp_home]\n"
        "dio = 0\n";
    Sim sim = Sim::FromToml(toml);
    try {
      sim.add<PositionCpp>("position_cpp");
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string{error.what()}.find(row.key), std::string::npos)
          << error.what();
    }
  }
}

// REQ-SAFE-07
TEST_F(PositionCppTest, OtherModesLaneIsIgnored) {
  Teleop();
  Hold(30.0, 40ms, "auto");  // autonomous's lane, live, in teleop
  EXPECT_EQ(Request(), Mode::kNeutral);
  ReleaseTarget<Target>(sim, "position_cpp", "auto");

  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kAutonomous));
  sim.step(10ms);
  Hold(30.0, 40ms);  // teleop's lane, live, in autonomous
  EXPECT_EQ(Request(), Mode::kNeutral);
  Hold(20.0, 40ms, "auto");
  const auto request = hw.motor_request("position_cpp", "position_cpp_motor");
  EXPECT_EQ(request.mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(request.demand, 20.0);
}

// REQ-SAFE-09
TEST_F(PositionCppTest, EarlierGenerationTargetIsIgnored) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kMotionMagic);
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
  PublishTarget(sim, "position_cpp", "teleop", Target{30.0}, before);
  sim.step(20ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  // The same target stamped with the current generation drives at once.
  PublishTarget(sim, "position_cpp", "teleop", Target{30.0});
  sim.step(20ms);
  EXPECT_EQ(Request(), Mode::kMotionMagic);
}

// REQ-RT-06: the steady-state tick (target read, request, flush, state
// publish) allocates nothing. The target is published before the window and
// stays fresh through it (60 ms < command_timeout_us).
TEST_F(PositionCppTest, UpdateDoesNotAllocate) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  Teleop();
  Hold(30.0, 40ms);
  PublishTarget(sim, "position_cpp", "teleop", Target{30.0});
  AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(Request(), Mode::kMotionMagic);
}

// Homing. The values are testdata/position_cpp.toml's.
constexpr double kHomingVolts = -1.0;
constexpr double kMaxPositionRot = 100.0;

TEST_F(PositionCppTest, HomesBeforeTheFirstGoal) {
  StartOffHome();
  hw.ideal_motors();
  Teleop();
  Hold(30.0, 60ms);  // toward the switch, not to 30 in the power-on frame
  auto request = hw.motor_request("position_cpp", "position_cpp_motor");
  EXPECT_EQ(request.mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(request.demand, kHomingVolts);
  EXPECT_EQ(Latest().homing(), PositionCppHoming::HOMING);

  // The switch closes 0.5 rotations below where the motor powered on: that
  // is position 0, and targets go out in that frame, clamped to the travel.
  hw.set_motor("position_cpp", "position_cpp_motor", -0.5, 0.0);
  // It zeroes after home_samples (3) consecutive closed readings.
  SetHome(true);
  Hold(30.0, 60ms);
  SetHome(false);  // the motor leaves the switch
  Hold(30.0, 20ms);
  request = hw.motor_request("position_cpp", "position_cpp_motor");
  EXPECT_EQ(request.mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(request.demand, 30.0 - 0.5);
  EXPECT_EQ(Latest().homing(), PositionCppHoming::HOMED);
  Hold(1000.0, 40ms);
  request = hw.motor_request("position_cpp", "position_cpp_motor");
  EXPECT_DOUBLE_EQ(request.demand, kMaxPositionRot - 0.5);
  Hold(-5.0, 40ms);
  request = hw.motor_request("position_cpp", "position_cpp_motor");
  EXPECT_DOUBLE_EQ(request.demand, -0.5);  // position 0
}

// One closed reading mid-travel (a glitch on the switch wire) is not home:
// homing keeps driving and the motor stays unzeroed.
TEST_F(PositionCppTest, OneClosedSampleMidTravelDoesNotZero) {
  StartOffHome();
  Teleop();
  Hold(30.0, 60ms);
  SetHome(true);
  Hold(30.0, 20ms);  // one tick
  SetHome(false);
  Hold(30.0, 60ms);
  EXPECT_EQ(Request(), Mode::kVoltage);
  EXPECT_EQ(Latest().homing(), PositionCppHoming::HOMING);
}

TEST_F(PositionCppTest, HomingNeedsALiveFiniteTarget) {
  StartOffHome();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  hw.enable(false);
  sim.step(10ms);
  Hold(30.0, 60ms);
  EXPECT_EQ(Request(), Mode::kNeutral);  // disabled

  Teleop();
  Hold(std::numeric_limits<double>::quiet_NaN(), 40ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVoltage);
  sim.step(150ms);  // past command_timeout_us: the lane is stale
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Latest().homing(), PositionCppHoming::HOMING);
}

TEST_F(PositionCppTest, HomingFailsUntilTheDriverStationDisables) {
  StartOffHome();
  Teleop();
  Hold(30.0, 400ms);
  ASSERT_EQ(Request(), Mode::kVoltage);
  Hold(30.0, 200ms);  // past homing_timeout_us (500 ms), switch still open
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Latest().homing(), PositionCppHoming::FAILED);

  // A stale hardware link (robot_enabled false) does not clear it.
  hw.lose_states(8);
  Hold(30.0, 160ms);
  Hold(30.0, 60ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Latest().homing(), PositionCppHoming::FAILED);

  hw.set_driver_station(DsPacket(ds::kTeleop));  // disabled
  sim.step(40ms);
  Teleop();
  Hold(30.0, 40ms);
  EXPECT_EQ(Request(), Mode::kVoltage);
}

TEST_F(PositionCppTest, HomingThatDoesNotMoveFails) {
  SetHome(false);  // velocity stays 0
  Teleop();
  Hold(30.0, 100ms);
  ASSERT_EQ(Request(), Mode::kVoltage);
  Hold(30.0, 200ms);  // past homing_stall_us (200 ms)
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Latest().homing(), PositionCppHoming::FAILED);
}

TEST_F(PositionCppTest, AStuckHomeSwitchStopsTheFirstMove) {
  hw.ideal_motors();
  Teleop();
  Hold(30.0, 60ms);  // the switch still reads closed at 30
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Latest().homing(), PositionCppHoming::FAILED);
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

TEST(PositionCppBinary, LoggedSimRunReplaysWithoutDivergence) {
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
}  // namespace position_cpp
