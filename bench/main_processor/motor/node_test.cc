#include "talOS/teleop/node.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "motor/motor.h"
#include "talOS/devices/testing.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/node/testing.h"

namespace motor {
namespace {
using namespace std::chrono_literals;
using talos::hardware::Mode;
using talos::testing::AllocationProbe;
using talos::testing::DriverStationGeneration;
using talos::testing::DsPacket;
using talos::testing::PublishTarget;
using talos::testing::ReleaseTarget;
namespace ds = talos::driver_station;
// The bench config plus a neighbor motor that stays connected (see the file).
constexpr const char* kConfig =
    "bench/main_processor/motor/testdata/robot.toml";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// A live 10 rps velocity target.
MotorTarget Velocity() { return {ControlMode::VELOCITY, 0.0, 10.0, 0.0}; }

// The node test matrix, docs/TESTING.md §7: the case names are the matrix's,
// which //tools:check_node_matrix looks for. This node drives the bench
// Kraken, so the safety cases assert what the gateway applies, after its
// gates, besides what the node requested.
class MotorTest : public ::testing::Test {
 protected:
  void Enable() {
    hw.enable();
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(50ms);
  }
  void Send(MotorTarget target) {
    PublishTarget(sim, "motor", "teleop", target);
    sim.step(10ms);
  }
  // Holds a lane the way a producer does: republished every 10 ms.
  void Hold(MotorTarget target, std::chrono::milliseconds duration,
            const char* source = "teleop") {
    for (auto t = 0ms; t < duration; t += 10ms) {
      PublishTarget(sim, "motor", source, target);
      sim.step(10ms);
    }
  }
  MotorState State() { return sim.latest<MotorState>(topics::kState).value(); }
  talos::hardware::MotorRequest Applied() const {
    return hw.applied_motor("motor", "kraken");
  }
  void Neutral() {
    EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kNeutral);
    EXPECT_EQ(Applied().mode, Mode::kNeutral);
    EXPECT_EQ(State().mode(), ControlMode::NEUTRAL);
    EXPECT_EQ(State().source(), TargetSource::NONE);
  }
  talos::testing::Sim sim{kConfig};
  BenchMotor& motor = sim.add<BenchMotor>("motor");
  talos::testing::SimHardware hw{sim};
};

// REQ-SAFE-07
TEST_F(MotorTest, DrivesAtALiveLane) {
  Enable();
  Send(Velocity());
  EXPECT_EQ(Applied().mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(Applied().demand, 10.0);
  EXPECT_EQ(State().mode(), ControlMode::VELOCITY);
  EXPECT_EQ(State().source(), TargetSource::TELEOP);
  EXPECT_TRUE(State().enabled());
}

// REQ-SAFE-02
TEST_F(MotorTest, DisabledRobotGoesNeutral) {
  Enable();
  Send(Velocity());
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  hw.enable(false);
  Hold(Velocity(), 30ms);
  Neutral();
  EXPECT_FALSE(State().enabled());  // state keeps publishing
}

// REQ-SAFE-07
TEST_F(MotorTest, StaleLaneGoesNeutral) {
  Enable();
  Send(Velocity());
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  // The lane was published 10 ms ago; 95 ms on it is past
  // command_timeout_us (100 ms) by one period (5 ms).
  sim.step(95ms);
  Neutral();
  EXPECT_TRUE(State().enabled());  // the robot is enabled; the lane is not
}

// REQ-SAFE-07
TEST_F(MotorTest, ReleasedLaneGoesNeutral) {
  Enable();
  Send(MotorTarget{ControlMode::VOLTAGE, 1.0, 0.0, 0.0});
  ASSERT_EQ(Applied().mode, Mode::kVoltage);
  ReleaseTarget<MotorTarget>(sim, "motor", "teleop");
  sim.step(10ms);
  Neutral();
}

// REQ-SAFE-07
TEST_F(MotorTest, NeverSentGoesNeutral) {
  Enable();
  sim.step(50ms);
  Neutral();
  EXPECT_TRUE(State().enabled());
}

// REQ-SAFE-07
TEST_F(MotorTest, DriverStationLossGoesNeutral) {
  Enable();
  Send(Velocity());
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  hw.stop_driver_station();  // the bridge stops forwarding DS packets
  Hold(Velocity(), 150ms);   // past command_timeout_us: the DS is stale
  Neutral();
}

// REQ-CONTRACT-25
TEST_F(MotorTest, DisconnectedFeedbackGoesNeutral) {
  Enable();
  Send(Velocity());
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  hw.set_connected("motor", "kraken", false);
  Hold(Velocity(), 20ms);
  Neutral();
  // The neighbor keeps the robot enabled: the node's own gate decides.
  EXPECT_TRUE(State().enabled());
  EXPECT_FALSE(State().connected());
}

// REQ-SAFE-04
TEST_F(MotorTest, NonFiniteDemandGoesNeutral) {
  Enable();
  for (const MotorTarget& target :
       {MotorTarget{ControlMode::VOLTAGE, kNaN, 0.0, 0.0},
        MotorTarget{ControlMode::VOLTAGE, 0.0, 0.0, kNaN},
        MotorTarget{ControlMode::VELOCITY, 0.0,
                    std::numeric_limits<double>::infinity(), 0.0}}) {
    SCOPED_TRACE(::testing::Message()
                 << "mode " << static_cast<int>(target.mode()) << " voltage_v "
                 << target.voltage_v() << " voltage_frac "
                 << target.voltage_frac() << " velocity_rps "
                 << target.velocity_rps());
    Send(Velocity());
    ASSERT_EQ(Applied().mode, Mode::kVelocity);
    Send(target);
    Neutral();
    EXPECT_TRUE(std::isfinite(State().applied_voltage_v()));
    EXPECT_TRUE(std::isfinite(State().velocity_rps()));
  }
}

// REQ-CONTRACT-18
TEST(MotorParameters, BadParameterFailsConstruction) {
  for (const char* value : {"0.0", "-1.0", "nan", "inf", "'two'"}) {
    SCOPED_TRACE(value);
    auto sim = talos::testing::Sim::FromToml(
        std::string{"[robot]\nperiod_us = 5000\ncommand_timeout_us = 100000\n"
                    "[subsystems.motor]\nmax_bench_voltage_v = "} +
        value +
        "\n[subsystems.motor.motors.kraken]\ntype = 'TalonFX'\ncan_id = 1\n");
    try {
      sim.add<BenchMotor>("motor");
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string{error.what()}.find("max_bench_voltage_v"),
                std::string::npos)
          << error.what();
    }
  }
}

// REQ-SAFE-07
TEST_F(MotorTest, OtherModesLaneIsIgnored) {
  Enable();
  // The bench policy lists teleop's lane in teleop and none in autonomous.
  // An autonomous lane, live, in teleop:
  Hold(Velocity(), 30ms, "auto");
  Neutral();
  ReleaseTarget<MotorTarget>(sim, "motor", "auto");
  // The teleop lane, driving in teleop, then live in autonomous:
  Send(Velocity());
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kAutonomous));
  sim.step(20ms);
  Hold(Velocity(), 30ms);
  Neutral();
}

// REQ-SAFE-09
TEST_F(MotorTest, EarlierGenerationTargetIsIgnored) {
  Enable();
  Send(Velocity());
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  const std::uint64_t before = DriverStationGeneration(sim);
  hw.set_driver_station(DsPacket(ds::kTeleop));  // disabled
  sim.step(40ms);
  // Re-enabled. SimHardware publishes the packet at its next 20 ms hook,
  // after the node's tick at that instant, so the packet waits for the
  // node's next drain; a target its producer queued under the earlier
  // enable, published before that drain, arrives in the same one.
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(20ms);
  ASSERT_NE(DriverStationGeneration(sim), before);
  PublishTarget(sim, "motor", "teleop", Velocity(), before);
  sim.step(10ms);
  Neutral();
  // The same target stamped with the current generation drives at once.
  Send(Velocity());
  EXPECT_EQ(Applied().mode, Mode::kVelocity);
}

// REQ-RT-06: the steady-state tick (target read, request, flush, state
// publish) allocates nothing. The target stays fresh through the window
// (60 ms < command_timeout_us).
TEST_F(MotorTest, UpdateDoesNotAllocate) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  Enable();
  Hold(Velocity(), 30ms);
  PublishTarget(sim, "motor", "teleop", Velocity());
  AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(Applied().mode, Mode::kVelocity);
}

// The node's own cap (max_bench_voltage_v, 2 V) holds below the motor's
// configured max_voltage (4 V), and scales the teleop axis.
TEST_F(MotorTest, VoltageIsClampedAndAxisUsesNodeScale) {
  Enable();
  Send(MotorTarget{ControlMode::VOLTAGE, 20.0, 0.0, 0.0});
  EXPECT_EQ(Applied().mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(Applied().demand, 2.0);
  Send(MotorTarget{ControlMode::VOLTAGE, -20.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(Applied().demand, -2.0);
  Send(MotorTarget{ControlMode::VOLTAGE, 0.0, 0.0, 0.25});
  EXPECT_DOUBLE_EQ(Applied().demand, 0.5);
  EXPECT_EQ(State().source(), TargetSource::TELEOP);
}

TEST_F(MotorTest, NeutralModeTargetGoesNeutral) {
  Enable();
  Send(Velocity());
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  Send(MotorTarget{ControlMode::NEUTRAL, 2.0, 30.0, 0.0});
  Neutral();
}

TEST_F(MotorTest, ReportsInjectedFeedbackAndAcquisitionAges) {
  Enable();
  hw.set_motor("motor", "kraken", 4.5, -7.0, 2000, 1000, 42.5, 6.25);
  sim.step(10ms);
  EXPECT_DOUBLE_EQ(State().position_rot(), 4.5);
  EXPECT_DOUBLE_EQ(State().velocity_rps(), -7.0);
  EXPECT_DOUBLE_EQ(State().temperature_degc(), 42.5);
  EXPECT_DOUBLE_EQ(State().supply_current_a(), 6.3);
  EXPECT_TRUE(State().connected());
  EXPECT_TRUE(State().acquisition_valid());
  EXPECT_EQ(State().position_age_us() - State().velocity_age_us(), 1000);
  hw.set_connected("motor", "kraken", false);
  sim.step(10ms);
  EXPECT_FALSE(State().connected());
  EXPECT_FALSE(State().acquisition_valid());
  EXPECT_DOUBLE_EQ(State().temperature_degc(), 0.0);
  EXPECT_DOUBLE_EQ(State().supply_current_a(), 0.0);
}

TEST_F(MotorTest, HealthFeedbackRemainsVisibleWithStaleTargetAndWhileDisabled) {
  Enable();
  hw.set_motor("motor", "kraken", 0.0, 0.0, 0, 0, 48.5, 3.25);
  Send(Velocity());
  ASSERT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(State().temperature_degc(), 48.5);
  EXPECT_DOUBLE_EQ(State().supply_current_a(), 3.3);
  sim.step(110ms);  // the target is stale
  EXPECT_DOUBLE_EQ(State().temperature_degc(), 48.5);
  EXPECT_DOUBLE_EQ(State().supply_current_a(), 3.3);
  hw.enable(false);
  sim.step(10ms);
  EXPECT_TRUE(State().connected());
  EXPECT_DOUBLE_EQ(State().temperature_degc(), 48.5);
  EXPECT_DOUBLE_EQ(State().supply_current_a(), 3.3);
}

TEST_F(MotorTest, ReportsLimitSwitchesAndClearsThemOnStaleState) {
  Enable();
  hw.set_limit_switches("motor", "kraken", true, false);
  sim.step(10ms);
  EXPECT_TRUE(State().forward_limit());
  EXPECT_FALSE(State().reverse_limit());
  hw.set_limit_switches("motor", "kraken", false, true);
  sim.step(10ms);
  EXPECT_FALSE(State().forward_limit());
  EXPECT_TRUE(State().reverse_limit());
  hw.set_limit_switches("motor", "kraken", true, true);
  hw.lose_states(1000);
  sim.step(200ms);
  EXPECT_FALSE(State().forward_limit());
  EXPECT_FALSE(State().reverse_limit());
}

TEST(BenchTeleop, ConfiguredAxisButtonsReleaseAndDisable) {
  talos::testing::Sim sim{kConfig};
  sim.add<BenchMotor>("motor");
  sim.add<talos::teleop::Teleop>("teleop");
  talos::testing::SimHardware hw{sim};
  hw.enable();
  auto packet = DsPacket(ds::kEnabled | ds::kTeleop);
  auto& stick = packet.joysticks[0];
  stick.connected = true;
  stick.axis_count = 2;
  stick.button_count = 2;
  const auto push = [&] {
    hw.set_driver_station(packet);
    sim.step(20ms);
  };
  push();  // Neutral evaluation rearms the producer.
  stick.axes[1] = -1.0;
  push();
  EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(hw.motor_request("motor", "kraken").demand, 2.0);
  stick.buttons = 2;
  push();
  EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(hw.motor_request("motor", "kraken").demand, 30.0);
  stick.buttons = 3;
  push();
  EXPECT_DOUBLE_EQ(hw.motor_request("motor", "kraken").demand, 10.0);
  stick.buttons = 0;
  stick.axes[1] = 0.0;
  push();
  EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kNeutral);
  stick.buttons = 1;
  push();
  packet.fms.flags = ds::kTeleop | ds::kDsAttached;
  hw.enable(false);
  push();
  EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kNeutral);
}

}  // namespace
}  // namespace motor
