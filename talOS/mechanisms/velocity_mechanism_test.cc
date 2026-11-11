// VelocityMechanism on a simulated robot (component tier): a node drives it
// from its teleop lane, as a robot node does, with SimHardware as the bridge.
//
// Node test matrix (docs/TESTING.md §7) for the library: 1, 2, 3, 7, 8, 9 and
// 12 below. n/a 4, 5, 6: released, never-sent and Driver-Station-lost lanes
// reach the library exactly as a stale one does, as stop(); the owning
// node's matrix covers its Target. n/a 10, 11: lane selection and
// generations are the owner's talos::Target.

#include "talOS/mechanisms/velocity_mechanism.h"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "control/control_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/devices/testing.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/messages.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/node/testing.h"

namespace talos::mechanisms {
namespace {

using namespace std::chrono_literals;
using hardware::Mode;
using testing::DsPacket;
using testing::PublishTarget;
using testing::Sim;
using testing::SimHardware;
namespace ds = driver_station;

// Exit band: max(2, 0.1 * |goal|), 6 rps at 60.
constexpr const char* kRobotToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.shooter.motors.flywheel]
can_id = 1
max_velocity_rps = 80.0
[subsystems.shooter.flywheel]
at_speed_entry_rps = 2.0
at_speed_exit_frac = 0.1
at_speed_entry_tick_count = 3
spin_down_rps2 = 100.0
)";

// A robot node around one VelocityMechanism, driven by its teleop lane.
struct Shooter {
  explicit Shooter(Node& node) : flywheel{node, "flywheel"}, target{node} {}
  void update() {
    if (const control::VelocityTarget* t = target.get()) {
      flywheel.set_goal(t->velocity_rps(), t->feedforward_v());
    } else {
      flywheel.stop();
    }
    flywheel.update();
  }
  VelocityMechanism flywheel;
  Target<control::VelocityTarget> target;
};

class VelocityMechanismTest : public ::testing::Test {
 protected:
  VelocityMechanismTest() {
    hw.enable();
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(10ms);
  }
  // The teleop lane, republished every 5 ms (one tick) for `duration`.
  void Drive(double velocity_rps, std::chrono::milliseconds duration,
             double feedforward_v = 0) {
    for (auto t = 0ms; t < duration; t += 5ms) {
      PublishTarget(sim, "shooter", "teleop",
                    control::VelocityTarget{velocity_rps, feedforward_v});
      sim.step(5ms);
    }
  }
  // Feedback, published before the next tick reads it.
  void SetVelocity(double velocity_rps) {
    hw.set_motor("shooter", "flywheel", 0.0, velocity_rps);
    sim.step(5ms);
  }
  hardware::MotorRequest Applied() const {
    return hw.applied_motor("shooter", "flywheel");
  }
  void ExpectNeutral() {
    EXPECT_EQ(Applied().mode, Mode::kNeutral);
    EXPECT_FALSE(flywheel().state().enabled());
    EXPECT_DOUBLE_EQ(flywheel().state().requested_rps(), 0.0);
    EXPECT_FALSE(flywheel().at_speed());
  }
  VelocityMechanism& flywheel() { return shooter.flywheel; }

  Sim sim = Sim::FromToml(kRobotToml);
  Shooter& shooter = sim.add<Shooter>("shooter");
  SimHardware hw{sim};
};

// REQ-SAFE-01: a live goal drives the velocity loop with its feedforward
// and reaches at_speed.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, DrivesAtALiveLane) {
  hw.ideal_motors();
  Drive(60.0, 40ms, 1.5);
  EXPECT_EQ(Applied().mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(Applied().demand, 60.0);
  EXPECT_DOUBLE_EQ(Applied().feedforward_v, 1.5);
  EXPECT_TRUE(flywheel().at_speed());
  const control::VelocityState state = flywheel().state();
  EXPECT_DOUBLE_EQ(state.velocity_rps(), 60.0);
  EXPECT_DOUBLE_EQ(state.requested_rps(), 60.0);
  EXPECT_TRUE(state.enabled());
  EXPECT_TRUE(state.connected());
}

// Goals clamp to the motor's own max_velocity_rps, and readiness is judged
// against the clamped goal.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, ClampsToTheMotorMaxAndIsReadyThere) {
  hw.ideal_motors();
  Drive(120.0, 40ms);
  EXPECT_DOUBLE_EQ(Applied().demand, 80.0);
  EXPECT_DOUBLE_EQ(flywheel().goal_rps(), 80.0);
  EXPECT_TRUE(flywheel().at_speed());
}

// Entry takes at_speed_entry_tick_count consecutive ticks in the entry band.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, AtSpeedNeedsConsecutiveEntryTicks) {
  SetVelocity(59.0);  // inside at_speed_entry_rps
  Drive(60.0, 10ms);  // two ticks
  EXPECT_FALSE(flywheel().at_speed());
  EXPECT_EQ(flywheel().phase(), VelocityPhase::kSpinningUp);
  Drive(60.0, 5ms);  // the third
  EXPECT_TRUE(flywheel().at_speed());
}

// Once at speed it stays inside the exit band, leaves past it, and must
// re-enter the entry band for the full count.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, ExitBandHysteresis) {
  SetVelocity(60.0);
  Drive(60.0, 20ms);
  ASSERT_TRUE(flywheel().at_speed());
  SetVelocity(55.0);  // 5 off: outside entry, inside exit (6)
  Drive(60.0, 20ms);
  EXPECT_TRUE(flywheel().at_speed());
  SetVelocity(53.0);  // 7 off: past exit
  Drive(60.0, 5ms);
  EXPECT_FALSE(flywheel().at_speed());
  SetVelocity(55.0);  // inside exit but not entry: stays out
  Drive(60.0, 30ms);
  EXPECT_FALSE(flywheel().at_speed());
  SetVelocity(59.0);
  Drive(60.0, 5ms);  // at most two ticks in the entry band
  EXPECT_FALSE(flywheel().at_speed());
  Drive(60.0, 10ms);
  EXPECT_TRUE(flywheel().at_speed());
}

// A changed goal restarts readiness, even inside the entry band.
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, ChangedGoalRestartsReadiness) {
  SetVelocity(60.0);
  Drive(60.0, 20ms);
  ASSERT_TRUE(flywheel().at_speed());
  Drive(61.0, 5ms);
  EXPECT_FALSE(flywheel().at_speed());
  Drive(61.0, 10ms);
  EXPECT_TRUE(flywheel().at_speed());
}

// A live goal that drops winds down at spin_down_rps2; losing the goal is
// neutral at once.
TEST_F(VelocityMechanismTest, SpinDownLimitsALiveDropButNotNeutral) {
  hw.ideal_motors();
  Drive(60.0, 40ms);
  ASSERT_DOUBLE_EQ(Applied().demand, 60.0);
  Drive(0.0, 100ms);  // 100 rps/s for 0.1 s: about 50 rps
  EXPECT_EQ(Applied().mode, Mode::kVelocity);
  EXPECT_NEAR(Applied().demand, 50.0, 1.0) << "spin_down_rps2 = 100";
  EXPECT_FALSE(flywheel().at_speed());
  EXPECT_DOUBLE_EQ(flywheel().goal_rps(), 0.0);
  sim.step(110ms);  // stale
  ExpectNeutral();
}

// REQ-SAFE-02
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, DisabledRobotGoesNeutral) {
  hw.ideal_motors();
  Drive(60.0, 40ms);
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  hw.enable(false);
  Drive(60.0, 30ms);
  ExpectNeutral();
}

// REQ-SAFE-07
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, StaleLaneGoesNeutral) {
  hw.ideal_motors();
  Drive(60.0, 40ms);
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  sim.step(110ms);
  ExpectNeutral();
  EXPECT_EQ(flywheel().phase(), VelocityPhase::kStopped);
}

// REQ-SAFE-37
TEST_F(VelocityMechanismTest, DisconnectedFeedbackGoesNeutral) {
  hw.set_connected("shooter", "flywheel", false);
  Drive(60.0, 30ms);
  ExpectNeutral();
  EXPECT_FALSE(flywheel().state().connected());
}

// REQ-SAFE-04
// REQ-SAFE-37
TEST_F(VelocityMechanismTest, NonFiniteDemandGoesNeutral) {
  Drive(std::numeric_limits<double>::quiet_NaN(), 30ms);
  ExpectNeutral();
  Drive(60.0, 30ms, std::numeric_limits<double>::infinity());
  ExpectNeutral();
}

// A request is refused outside update() (here every()), naming the
// mechanism, and lasts only the update() it was made in.
// REQ-SAFE-37
TEST(VelocityMechanismRequest, RequestIsTiedToItsUpdate) {
  struct TimerNode {
    explicit TimerNode(Node& node) : flywheel{node, "flywheel"} {
      node.every(20ms, [this] { flywheel.set_goal(60.0); });
    }
    void update() { flywheel.update(); }
    VelocityMechanism flywheel;
  };
  {
    auto sim = Sim::FromToml(kRobotToml);
    sim.add<TimerNode>("shooter");
    try {
      sim.step(50ms);
      ADD_FAILURE() << "set_goal() from every() was accepted";
    } catch (const std::logic_error& e) {
      EXPECT_NE(std::string{e.what()}.find("VelocityMechanism 'flywheel'"),
                std::string::npos)
          << e.what();
    }
  }
  struct SkippingNode {
    explicit SkippingNode(Node& node) : flywheel{node, "flywheel"} {}
    void update() {
      if (armed && step++ == 0) {
        flywheel.set_goal(60.0);  // update() skipped this tick
        return;
      }
      flywheel.update();
    }
    VelocityMechanism flywheel;
    bool armed = false;
    int step = 0;
  };
  auto sim = Sim::FromToml(kRobotToml);
  auto& node = sim.add<SkippingNode>("shooter");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(10ms);
  node.armed = true;
  sim.step(15ms);
  ASSERT_GE(node.step, 2);
  EXPECT_FALSE(node.flywheel.driving());
  EXPECT_EQ(hw.applied_motor("shooter", "flywheel").mode, Mode::kNeutral);
}

// REQ-RT-06
TEST_F(VelocityMechanismTest, UpdateDoesNotAllocate) {
  if (const auto why = testing::AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  hw.ideal_motors();
  Drive(60.0, 40ms);
  PublishTarget(sim, "shooter", "teleop", control::VelocityTarget{60.0, 0.0});
  testing::AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_TRUE(flywheel().at_speed());
}

// Each bad parameter throws at construction, naming the key.
// REQ-SAFE-37
TEST(VelocityMechanismConfig, BadParameterFailsConstruction) {
  const auto add = [](const std::string& table) {
    auto sim = Sim::FromToml(std::string{R"(
[robot]
period_us = 5000
command_timeout_us = 100000
[subsystems.shooter.motors.flywheel]
can_id = 1
[subsystems.shooter.flywheel]
)"} + table);
    sim.add<Shooter>("shooter");
  };
  EXPECT_NO_THROW(add(""));
  const std::vector<std::pair<std::string, std::string>> bad{
      {"at_speed_entry_rps", "at_speed_entry_rps = 0.0\n"},
      {"at_speed_entry_rps", "at_speed_entry_rps = nan\n"},
      {"at_speed_exit_frac", "at_speed_exit_frac = 1.5\n"},
      {"at_speed_entry_tick_count", "at_speed_entry_tick_count = 0\n"},
      {"spin_down_rps2", "spin_down_rps2 = -1.0\n"},
  };
  for (const auto& [key, table] : bad) {
    SCOPED_TRACE(table);
    try {
      add(table);
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string{e.what()}.find(key), std::string::npos) << e.what();
    }
  }
}

}  // namespace
}  // namespace talos::mechanisms
