#include "talOS/teleop/node.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>

#include "motor/motor.h"
#include "swerve_module/module_message_generated.h"
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
// The bench config plus a neighbor motor that stays connected, with torque
// mode on (foc, max_torque_current_a 5 A, max_bench_torque_current_a 2 A)
// and max_voltage 4 V, above the node's 2 V cap.
constexpr const char* kConfig =
    "bench/main_processor/motor/testdata/robot.toml";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
// A live 10 rps velocity target.
MotorTarget Velocity() { return {ControlMode::VELOCITY, 0.0, 10.0, 0.0, 0.0}; }
MotorTarget Torque(double amperes) {
  return {ControlMode::TORQUE_CURRENT, 0.0, 0.0, 0.0, amperes};
}
MotorTarget Voltage(double volts) {
  return {ControlMode::VOLTAGE, volts, 0.0, 0.0, 0.0};
}

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

// A live target of each mode that drives, and what the gateway applies for
// it: cases 1 and 12 run once per mode, since each mode has its own setter
// and clamp. The other cases' code (the gate before the mode branch) is the
// same for every mode, so they run with a velocity target only.
struct Driving {
  const char* name;
  MotorTarget target;
  Mode applied;
  double demand;
};
void PrintTo(const Driving& driving, std::ostream* out) {
  *out << driving.name;
}
class DrivingTest : public MotorTest,
                    public ::testing::WithParamInterface<Driving> {};
INSTANTIATE_TEST_SUITE_P(
    Modes, DrivingTest,
    ::testing::Values(Driving{"Velocity", Velocity(), Mode::kVelocity, 10.0},
                      // Within max_bench_torque_current_a (2 A).
                      Driving{"TorqueCurrent", Torque(1.5),
                              Mode::kTorqueCurrent, 1.5}),
    [](const auto& info) { return std::string{info.param.name}; });

// REQ-SAFE-07, REQ-SAFE-43 (stator amperes reach the request as sent)
TEST_P(DrivingTest, DrivesAtALiveLane) {
  Enable();
  Send(GetParam().target);
  EXPECT_EQ(Applied().mode, GetParam().applied);
  EXPECT_DOUBLE_EQ(Applied().demand, GetParam().demand);
  EXPECT_EQ(State().mode(), GetParam().target.mode());
  EXPECT_EQ(State().source(), TargetSource::TELEOP);
  EXPECT_TRUE(State().enabled());
}

// REQ-CONTRACT-25: matrix case 2, `state` still published with
// `enabled == false` while disabled. The neutral request is the
// devices flush's (REQ-SAFE-02, proved in talOS/devices), not this
// node's evidence.
TEST_F(MotorTest, DisabledRobotGoesNeutral) {
  Enable();
  Send(Velocity());
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  hw.enable(false);
  Hold(Velocity(), 30ms);
  Neutral();
  EXPECT_FALSE(State().enabled());  // state keeps publishing
}

// REQ-CONTRACT-25: matrix case 3, the node's `state` reports no mode and no
// source once its lane is stale, the robot still enabled. The neutral request
// is the devices flush's (Target::get returns no target: REQ-SAFE-41, proved in
// talOS/devices), not this node's evidence.
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

// REQ-CONTRACT-25: matrix case 4, the node's `state` reports no mode and no
// source after a release. The neutral request is the devices flush's
// (Target::get returns no target: REQ-SAFE-41, proved in talOS/devices), not
// this node's evidence.
TEST_F(MotorTest, ReleasedLaneGoesNeutral) {
  Enable();
  Send(Voltage(1.0));
  ASSERT_EQ(Applied().mode, Mode::kVoltage);
  ReleaseTarget<MotorTarget>(sim, "motor", "teleop");
  sim.step(10ms);
  Neutral();
}

// REQ-CONTRACT-25: matrix case 5, the node's `state` reports no mode and no
// source while no target was ever sent. The neutral request is the devices
// flush's (Target::get returns no target: REQ-SAFE-41, proved in
// talOS/devices), not this node's evidence.
TEST_F(MotorTest, NeverSentGoesNeutral) {
  Enable();
  sim.step(50ms);
  Neutral();
  EXPECT_TRUE(State().enabled());
}

// Matrix case 6: the node keeps running once the Driver Station view is stale.
// The neutral request is the devices flush's (Target::get returns no
// target: REQ-SAFE-41, proved in talOS/devices), not this node's evidence.
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

// REQ-SAFE-04. An infinite torque current would otherwise clamp to the
// node's cap and drive.
TEST_F(MotorTest, NonFiniteDemandGoesNeutral) {
  Enable();
  for (const MotorTarget& target :
       {MotorTarget{ControlMode::VOLTAGE, kNaN, 0.0, 0.0, 0.0},
        MotorTarget{ControlMode::VOLTAGE, kInf, 0.0, 0.0, 0.0},
        MotorTarget{ControlMode::VOLTAGE, 0.0, 0.0, kNaN, 0.0},
        MotorTarget{ControlMode::VOLTAGE, 0.0, 0.0, kInf, 0.0},
        MotorTarget{ControlMode::VELOCITY, 0.0, kNaN, 0.0, 0.0},
        MotorTarget{ControlMode::VELOCITY, 0.0, kInf, 0.0, 0.0}, Torque(kNaN),
        Torque(kInf), Torque(-kInf)}) {
    SCOPED_TRACE(::testing::Message()
                 << "mode " << static_cast<int>(target.mode()) << " voltage_v "
                 << target.voltage_v() << " voltage_frac "
                 << target.voltage_frac() << " velocity_rps "
                 << target.velocity_rps() << " torque_current_a "
                 << target.torque_current_a());
    Send(Velocity());
    ASSERT_EQ(Applied().mode, Mode::kVelocity);
    Send(target);
    Neutral();
    EXPECT_TRUE(std::isfinite(State().applied_voltage_v()));
    EXPECT_TRUE(std::isfinite(State().velocity_rps()));
  }
}

// REQ-CONTRACT-18. A positive torque cap is refused unless the motor
// allows torque current (REQ-SAFE-43), so a misconfigured bench fails at
// launch, not at the first press of button 3.
TEST(MotorParameters, BadParameterFailsConstruction) {
  constexpr const char* kTorque = "foc = true\nmax_torque_current_a = 5.0\n";
  struct Bad {
    const char* key;     // the node parameter
    const char* value;   // its value
    const char* kraken;  // extra [motors.kraken] lines
    const char* names;   // what the error names besides the key
  };
  for (const Bad& bad : {
           Bad{"max_bench_voltage_v", "0.0", "", ""},
           Bad{"max_bench_voltage_v", "-1.0", "", ""},
           Bad{"max_bench_voltage_v", "nan", "", ""},
           Bad{"max_bench_voltage_v", "inf", "", ""},
           Bad{"max_bench_voltage_v", "'two'", "", ""},
           Bad{"max_bench_torque_current_a", "-1.0", kTorque, ""},
           Bad{"max_bench_torque_current_a", "nan", kTorque, ""},
           Bad{"max_bench_torque_current_a", "inf", kTorque, ""},
           Bad{"max_bench_torque_current_a", "'two'", kTorque, ""},
           // A motor that allows no torque current: no foc, or no limit.
           Bad{"max_bench_torque_current_a", "2.0", "", "foc = true"},
           Bad{"max_bench_torque_current_a", "2.0", "foc = true\n",
               "max_torque_current_a > 0"},
       }) {
    SCOPED_TRACE(::testing::Message()
                 << bad.key << " = " << bad.value << ", " << bad.kraken);
    auto sim = talos::testing::Sim::FromToml(
        std::string{"[robot]\nperiod_us = 5000\ncommand_timeout_us = 100000\n"
                    "[subsystems.motor]\n"} +
        bad.key + " = " + bad.value +
        "\n[subsystems.motor.motors.kraken]\ntype = 'TalonFX'\ncan_id = 1\n" +
        bad.kraken);
    try {
      sim.add<BenchMotor>("motor");
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& error) {
      const std::string what{error.what()};
      EXPECT_NE(what.find(bad.key), std::string::npos) << what;
      EXPECT_NE(what.find(bad.names), std::string::npos) << what;
    }
  }
}

// REQ-CONTRACT-25: matrix case 10, the node's `state` reports no mode and no
// source for another mode's lane, and for its own lane in a mode whose policy
// lists none. The neutral request is the devices flush's (Target::get returns
// no target: REQ-SAFE-41, proved in talOS/devices), not this node's evidence.
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

// REQ-CONTRACT-25: matrix case 11, the node's `state` reports no mode and no
// source for an earlier generation's target, and it drives again on the current
// one. The neutral request is the devices flush's (Target::get returns no
// target: REQ-SAFE-41, proved in talOS/devices), not this node's evidence.
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
TEST_P(DrivingTest, UpdateDoesNotAllocate) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  Enable();
  Hold(GetParam().target, 30ms);
  PublishTarget(sim, "motor", "teleop", GetParam().target);
  AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(Applied().mode, GetParam().applied);
}

// The node's own cap (max_bench_voltage_v, 2 V) holds below the test
// config's max_voltage (4 V), and scales the teleop axis.
TEST_F(MotorTest, VoltageIsClampedAndAxisUsesNodeScale) {
  Enable();
  Send(Voltage(20.0));
  EXPECT_EQ(Applied().mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(Applied().demand, 2.0);
  Send(Voltage(-20.0));
  EXPECT_DOUBLE_EQ(Applied().demand, -2.0);
  Send(MotorTarget{ControlMode::VOLTAGE, 0.0, 0.0, 0.25, 0.0});
  EXPECT_DOUBLE_EQ(Applied().demand, 0.5);
  EXPECT_EQ(State().source(), TargetSource::TELEOP);
}

// REQ-SAFE-43: the node's cap (max_bench_torque_current_a, 2 A) holds below
// the motor's max_torque_current_a (5 A), and 0 A is sent as zero torque,
// not neutral (docs/BENCH.md's 0 A step depends on it).
TEST_F(MotorTest, TorqueCurrentIsClampedToTheNodeCapAndZeroIsSent) {
  Enable();
  Send(Torque(8.0));
  EXPECT_EQ(Applied().mode, Mode::kTorqueCurrent);
  EXPECT_DOUBLE_EQ(Applied().demand, 2.0);
  Send(Torque(-8.0));
  EXPECT_DOUBLE_EQ(Applied().demand, -2.0);
  EXPECT_EQ(State().mode(), ControlMode::TORQUE_CURRENT);
  Send(Torque(0.0));
  EXPECT_EQ(Applied().mode, Mode::kTorqueCurrent);
  EXPECT_DOUBLE_EQ(Applied().demand, 0.0);
  EXPECT_EQ(State().mode(), ControlMode::TORQUE_CURRENT);
}

// REQ-SAFE-43: the committed bench config (max_bench_torque_current_a 0, no
// foc) drives nothing for a torque target, so its button 3 moves nothing,
// while a voltage target still drives.
TEST(MotorTorque, TorqueModeOffAtTheCommittedCap) {
  talos::testing::Sim sim{"bench/main_processor/configuration/robot.toml"};
  sim.add<BenchMotor>("motor");
  talos::testing::SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(50ms);
  const auto send = [&](const MotorTarget& target) {
    PublishTarget(sim, "motor", "teleop", target);
    sim.step(10ms);
  };
  send(Torque(1.5));
  EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kNeutral);
  EXPECT_EQ(hw.applied_motor("motor", "kraken").mode, Mode::kNeutral);
  EXPECT_EQ(sim.latest<MotorState>(topics::kState).value().mode(),
            ControlMode::NEUTRAL);
  send(Voltage(1.0));
  EXPECT_EQ(hw.applied_motor("motor", "kraken").mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(hw.applied_motor("motor", "kraken").demand, 1.0);
}

TEST_F(MotorTest, NeutralModeTargetGoesNeutral) {
  Enable();
  Send(Velocity());
  ASSERT_EQ(Applied().mode, Mode::kVelocity);
  Send(MotorTarget{ControlMode::NEUTRAL, 2.0, 30.0, 0.0, 1.0});
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
  // An invalid sample zeroes what was measured (hardware::MarkInvalid); the
  // temperature signal keeps its last value until it is 500 ms old, then
  // reads 0, as PhoenixBackend reports it.
  EXPECT_DOUBLE_EQ(State().temperature_degc(), 42.5);
  EXPECT_DOUBLE_EQ(State().supply_current_a(), 0.0);
  sim.step(500ms);
  EXPECT_DOUBLE_EQ(State().temperature_degc(), 0.0);
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

// The bench config binds the swerve module's lane too, and teleop needs a
// reader of every bound lane; this stands in for the swerve module node,
// whose package this one may not depend on.
struct ModuleLaneReader {
  explicit ModuleLaneReader(talos::Node& node) : target{node} {}
  void update() {}
  talos::Target<swerve_module::ModuleTarget> target;
};

TEST(BenchTeleop, ConfiguredAxisButtonsReleaseAndDisable) {
  talos::testing::Sim sim{kConfig};
  sim.add<BenchMotor>("motor");
  sim.add<ModuleLaneReader>("swerve_module");
  sim.add<talos::teleop::Teleop>("teleop");
  talos::testing::SimHardware hw{sim};
  hw.enable();
  auto packet = DsPacket(ds::kEnabled | ds::kTeleop);
  auto& stick = packet.joysticks[0];
  stick.connected = true;
  stick.axis_count = 2;
  stick.button_count = 3;
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
  // Button 3: TORQUE_CURRENT 2 A (the test config turns torque mode on);
  // button 1 wins over it.
  stick.buttons = 4;
  push();
  EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kTorqueCurrent);
  EXPECT_DOUBLE_EQ(hw.motor_request("motor", "kraken").demand, 2.0);
  stick.buttons = 5;
  push();
  EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kVelocity);
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
