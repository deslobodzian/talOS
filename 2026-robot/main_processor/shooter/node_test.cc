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
#include <string_view>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "shooter/shooter.h"
#include "talOS/devices/testing.h"
#include "talOS/node/allocation_probe.h"
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
  void Hold(double rps, std::chrono::milliseconds duration,
            const char* source = "teleop") {
    for (auto t = 0ms; t < duration; t += 20ms) {
      talos::testing::PublishTarget(sim, "shooter", source, ShooterTarget{rps});
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
    EXPECT_EQ(hw.applied_motor("shooter", "flywheel").mode, Mode::kNeutral);
    EXPECT_DOUBLE_EQ(State().target_velocity_rps(), 0.0);
    EXPECT_FALSE(State().at_speed());
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  Shooter& shooter = sim.add<Shooter>("shooter");
  SimHardware hw{sim};
};

// REQ-SAFE-07
TEST_F(ShooterTest, DrivesAtALiveLane) {
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

TEST_F(ShooterTest, EntryRequiresThreeConsecutiveTicksInsideTheEntryBand) {
  Teleop();
  hw.set_motor("shooter", "flywheel", 0.0, 80.0);
  sim.step(10ms);  // publish feedback before issuing a target
  talos::testing::PublishTarget(sim, "shooter", "teleop", ShooterTarget{80.0});
  sim.step(5ms);
  EXPECT_FALSE(State().at_speed());
  sim.step(5ms);
  EXPECT_FALSE(State().at_speed());
  sim.step(5ms);
  EXPECT_TRUE(State().at_speed());

  hw.set_motor("shooter", "flywheel", 0.0, 70.0);
  sim.step(10ms);
  ASSERT_FALSE(State().at_speed());
  hw.set_motor("shooter", "flywheel", 0.0, 80.0);
  sim.step(5ms);
  EXPECT_FALSE(State().at_speed());
  hw.set_motor("shooter", "flywheel", 0.0, 77.0);
  sim.step(10ms);  // a tick outside entry breaks the streak
  EXPECT_FALSE(State().at_speed());
  hw.set_motor("shooter", "flywheel", 0.0, 80.0);
  sim.step(5ms);  // hardware publishes after the node; allow feedback delivery
  sim.step(10ms);
  EXPECT_FALSE(State().at_speed());
  sim.step(5ms);
  EXPECT_TRUE(State().at_speed());
}

TEST_F(ShooterTest, ShotDipInsideExitBandKeepsReadinessButLargerDipDropsIt) {
  Teleop();
  hw.set_motor("shooter", "flywheel", 0.0, 80.0);
  Hold(80.0, 40ms);
  ASSERT_TRUE(State().at_speed());
  hw.set_motor("shooter", "flywheel", 0.0, 74.0);
  Hold(80.0, 20ms);
  EXPECT_TRUE(State().at_speed());  // 6 rps dip < 10% of 80 rps
  hw.set_motor("shooter", "flywheel", 0.0, 71.0);
  Hold(80.0, 20ms);
  EXPECT_FALSE(State().at_speed());
  hw.set_motor("shooter", "flywheel", 0.0, 74.0);
  Hold(80.0, 20ms);
  EXPECT_FALSE(State().at_speed());  // exit band alone cannot re-enter
}

TEST_F(ShooterTest, GoalAboveMotorLimitBecomesReadyAtClampedDemand) {
  Teleop();
  hw.ideal_motors();
  Hold(150.0, 60ms);
  EXPECT_DOUBLE_EQ(hw.motor_request("shooter", "flywheel").demand, 100.0);
  EXPECT_DOUBLE_EQ(State().target_velocity_rps(), 100.0);
  EXPECT_TRUE(State().at_speed());
  talos::testing::PublishTarget(sim, "shooter", "teleop", ShooterTarget{160.0});
  sim.step(5ms);
  EXPECT_FALSE(
      State().at_speed());  // changed even though demand clamps equally
}

TEST_F(ShooterTest, ChangedTargetAndZeroTargetClearReadiness) {
  Teleop();
  hw.set_motor("shooter", "flywheel", 0.0, 80.0);
  Hold(80.0, 40ms);
  ASSERT_TRUE(State().at_speed());
  talos::testing::PublishTarget(sim, "shooter", "teleop", ShooterTarget{81.0});
  sim.step(5ms);
  EXPECT_FALSE(
      State().at_speed());  // still inside entry, but must qualify again
  Hold(81.0, 20ms);
  ASSERT_TRUE(State().at_speed());
  Hold(0.0, 20ms);
  EXPECT_FALSE(State().at_speed());
}

// REQ-CONTRACT-25
TEST_F(ShooterTest, DisconnectedFeedbackGoesNeutral) {
  Teleop();
  hw.set_motor("shooter", "flywheel", 0.0, 80.0);
  Hold(80.0, 40ms);
  ASSERT_TRUE(State().at_speed());
  hw.set_connected("shooter", "flywheel", false);
  sim.step(5ms);  // publish the disconnected feedback after the node's timer
  sim.step(5ms);  // the first node tick receiving it must clear readiness
  ExpectNeutral();
  // The neighbor keeps the robot enabled: the flywheel's own gate decides.
  EXPECT_TRUE(State().enabled());
  hw.set_connected("shooter", "flywheel", true);
  sim.step(5ms);  // publish reconnected feedback before counting entry ticks
  sim.step(10ms);
  EXPECT_FALSE(State().at_speed());
  sim.step(5ms);
  EXPECT_TRUE(State().at_speed());
}

// REQ-SAFE-07
TEST_F(ShooterTest, NeverSentGoesNeutral) {
  Teleop();
  sim.step(60ms);
  ExpectNeutral();
  EXPECT_TRUE(State().enabled());
}

// REQ-SAFE-07
TEST_F(ShooterTest, StaleLaneGoesNeutral) {
  Teleop();
  hw.set_motor("shooter", "flywheel", 0.0, 80.0);
  Hold(80.0, 40ms);
  ASSERT_TRUE(State().at_speed());
  sim.step(75ms);  // last publication is 95 ms old: still fresh
  ASSERT_TRUE(State().at_speed());
  sim.step(5ms);  // exactly command_timeout_us: clear on the first stale tick
  ExpectNeutral();
}

// REQ-SAFE-07
TEST_F(ShooterTest, ReleasedLaneGoesNeutral) {
  Teleop();
  hw.set_motor("shooter", "flywheel", 0.0, 80.0);
  Hold(80.0, 40ms);
  ASSERT_TRUE(State().at_speed());
  talos::testing::ReleaseTarget<ShooterTarget>(sim, "shooter", "teleop");
  sim.step(5ms);
  ExpectNeutral();
}

// REQ-SAFE-02
TEST_F(ShooterTest, DisabledRobotGoesNeutral) {
  Teleop();
  hw.set_motor("shooter", "flywheel", 0.0, 80.0);
  Hold(80.0, 40ms);
  ASSERT_TRUE(State().at_speed());
  hw.enable(false);
  sim.step(5ms);  // publish disabled hardware feedback
  sim.step(5ms);  // clear immediately when the node receives that feedback
  ExpectNeutral();
  EXPECT_FALSE(State().enabled());
}

// REQ-SAFE-07
TEST_F(ShooterTest, DriverStationLossGoesNeutral) {
  Teleop();
  Hold(80.0, 40ms);
  ASSERT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kVelocity);
  // The Driver Station goes silent; its last packet ages out.
  hw.stop_driver_station();
  Hold(80.0, 160ms);
  ExpectNeutral();
}

// REQ-SAFE-07
TEST_F(ShooterTest, OtherModesLaneIsIgnored) {
  Teleop();
  Hold(80.0, 40ms, "auto");  // autonomous's lane, live, in teleop
  ExpectNeutral();
  talos::testing::ReleaseTarget<ShooterTarget>(sim, "shooter", "auto");
  Hold(80.0, 40ms);
  ASSERT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kVelocity);
  hw.set_driver_station(
      talos::testing::DsPacket(ds::kEnabled | ds::kAutonomous));
  sim.step(10ms);
  // Stamped with the autonomous generation: only the policy rejects it.
  Hold(80.0, 40ms);
  ExpectNeutral();
  Hold(60.0, 40ms, "auto");
  const auto request = hw.motor_request("shooter", "flywheel");
  EXPECT_EQ(request.mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(request.demand, 60.0);
}

// REQ-SAFE-09
TEST_F(ShooterTest, EarlierGenerationTargetIsIgnored) {
  Teleop();
  Hold(80.0, 40ms);
  ASSERT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kVelocity);
  const std::uint64_t before = talos::testing::DriverStationGeneration(sim);
  hw.set_driver_station(talos::testing::DsPacket(ds::kTeleop));  // disabled
  sim.step(10ms);
  // Re-enabled off the 5 ms tick grid: the packet lands just after the
  // shooter's tick and waits for its next drain, together with a target its
  // producer queued under the earlier enable.
  sim.step(1ms);
  hw.set_driver_station(talos::testing::DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(4ms);
  ASSERT_NE(talos::testing::DriverStationGeneration(sim), before);
  talos::testing::PublishTarget(sim, "shooter", "teleop", ShooterTarget{80.0},
                                before);
  sim.step(5ms);
  ExpectNeutral();
  // The same word stamped with the current generation drives at once.
  talos::testing::PublishTarget(sim, "shooter", "teleop", ShooterTarget{80.0});
  sim.step(5ms);
  EXPECT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kVelocity);
}

// REQ-SAFE-04
TEST_F(ShooterTest, NonFiniteDemandGoesNeutral) {
  Teleop();
  for (const double rps : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
    SCOPED_TRACE(rps);
    Hold(80.0, 40ms);
    ASSERT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kVelocity);
    Hold(rps, 40ms);
    ExpectNeutral();
  }
}

// REQ-RT-06 (RULES.md 3): the steady-state tick (target read, readiness,
// velocity request, flush, state publish) allocates nothing. The target is
// published before the window and stays fresh through it (60 ms <
// command_timeout_us).
TEST_F(ShooterTest, UpdateDoesNotAllocate) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  Teleop();
  hw.ideal_motors();
  Hold(80.0, 40ms);
  talos::testing::PublishTarget(sim, "shooter", "teleop", ShooterTarget{80.0});
  talos::testing::AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(hw.motor_request("shooter", "flywheel").mode, Mode::kVelocity);
  EXPECT_TRUE(State().at_speed());
}

// Each bad [flywheel] parameter throws at construction, naming the key.
// REQ-SAFE-37
TEST(ShooterParameters, BadParameterFailsConstruction) {
  for (const char* setting :
       {"at_speed_entry_rps = 0", "at_speed_entry_rps = -1",
        "at_speed_entry_rps = nan", "at_speed_entry_rps = inf",
        "at_speed_exit_frac = 0", "at_speed_exit_frac = -0.1",
        "at_speed_exit_frac = 1.1", "at_speed_exit_frac = nan",
        "at_speed_exit_frac = inf", "at_speed_entry_tick_count = 0",
        "at_speed_entry_tick_count = -1"}) {
    SCOPED_TRACE(setting);
    const std::string key =
        "flywheel." +
        std::string{setting}.substr(0, std::string_view{setting}.find(' '));
    auto sim = Sim::FromToml(std::string{R"(
[robot]
period_us = 5000
command_timeout_us = 100000
[subsystems.shooter.flywheel]
)"} + setting + R"(
[subsystems.shooter.motors.flywheel]
type = "TalonFX"
can_id = 9
max_velocity_rps = 100.0
[subsystems.shooter.digital_inputs.beam_break]
dio = 0
)");
    try {
      sim.add<Shooter>("shooter");
      ADD_FAILURE() << "accepted";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string{error.what()}.find(key), std::string::npos)
          << error.what();
    }
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
