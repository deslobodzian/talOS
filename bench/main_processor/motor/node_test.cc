#include "talOS/teleop/node.h"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>

#include "motor/motor.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"

namespace motor {
namespace {
using namespace std::chrono_literals;
using talos::hardware::Mode;
class MotorTest : public ::testing::Test {
 protected:
  void Enable() {
    hw.enable();
    hw.set_driver_station(talos::testing::DsPacket(
        talos::driver_station::kEnabled | talos::driver_station::kTeleop));
    sim.step(50ms);
  }
  void Send(MotorTarget target) {
    talos::testing::PublishTarget(sim, "motor", "teleop", target);
    sim.step(10ms);
  }
  MotorState State() { return sim.latest<MotorState>(topics::kState).value(); }
  void Neutral() {
    EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kNeutral);
    EXPECT_EQ(State().mode(), ControlMode::NEUTRAL);
    EXPECT_EQ(State().source(), TargetSource::NONE);
  }
  talos::testing::Sim sim{"bench/main_processor/configuration/robot.toml"};
  BenchMotor& motor = sim.add<BenchMotor>("motor");
  talos::testing::SimHardware hw{sim};
};

TEST_F(MotorTest, DisabledAndNeverSentAreNeutral) {
  Enable();
  Neutral();
  Send(MotorTarget{ControlMode::VOLTAGE, 1.0, 0.0, 0.0});
  hw.enable(false);
  sim.step(10ms);
  Neutral();
}

TEST_F(MotorTest, VoltageIsClampedAndAxisUsesNodeScale) {
  Enable();
  Send(MotorTarget{ControlMode::VOLTAGE, 20.0, 0.0, 0.0});
  EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(hw.motor_request("motor", "kraken").demand, 2.0);
  Send(MotorTarget{ControlMode::VOLTAGE, -20.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(hw.motor_request("motor", "kraken").demand, -2.0);
  Send(MotorTarget{ControlMode::VOLTAGE, 0.0, 0.0, 0.25});
  EXPECT_DOUBLE_EQ(hw.motor_request("motor", "kraken").demand, 0.5);
  EXPECT_EQ(State().source(), TargetSource::TELEOP);
}

TEST_F(MotorTest, VelocityAndExplicitNeutral) {
  Enable();
  Send(MotorTarget{ControlMode::VELOCITY, 0.0, 10.0, 0.0});
  EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(hw.motor_request("motor", "kraken").demand, 10.0);
  EXPECT_EQ(State().mode(), ControlMode::VELOCITY);
  Send(MotorTarget{ControlMode::NEUTRAL, 2.0, 30.0, 0.0});
  Neutral();
}

TEST_F(MotorTest, StaleTargetNeutralizes) {
  Enable();
  Send(MotorTarget{ControlMode::VELOCITY, 0.0, 30.0, 0.0});
  sim.step(110ms);
  Neutral();
}

TEST_F(MotorTest, ReleaseNeutralizes) {
  Enable();
  Send(MotorTarget{ControlMode::VOLTAGE, 1.0, 0.0, 0.0});
  talos::testing::ReleaseTarget<MotorTarget>(sim, "motor", "teleop");
  sim.step(10ms);
  Neutral();
}

TEST_F(MotorTest, ReportsInjectedFeedbackAndAcquisitionAges) {
  Enable();
  hw.set_motor("motor", "kraken", 4.5, -7.0, 2000, 1000);
  sim.step(10ms);
  EXPECT_DOUBLE_EQ(State().position_rot(), 4.5);
  EXPECT_DOUBLE_EQ(State().velocity_rps(), -7.0);
  EXPECT_DOUBLE_EQ(State().applied_voltage_v(), 0.0);
  EXPECT_DOUBLE_EQ(State().stator_current_a(), 0.0);
  EXPECT_TRUE(State().connected());
  EXPECT_TRUE(State().acquisition_valid());
  EXPECT_EQ(State().position_age_us() - State().velocity_age_us(), 1000);
  hw.set_connected("motor", "kraken", false);
  sim.step(10ms);
  EXPECT_FALSE(State().connected());
  EXPECT_FALSE(State().acquisition_valid());
}

TEST_F(MotorTest, NonFiniteAndDisconnectedInputsNeutralize) {
  Enable();
  Send(MotorTarget{ControlMode::VOLTAGE,
                   std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  Neutral();
  hw.set_connected("motor", "kraken", false);
  Send(MotorTarget{ControlMode::VELOCITY, 0.0, 10.0, 0.0});
  Neutral();
}
TEST(BenchTeleop, ConfiguredAxisButtonsReleaseAndDisable) {
  talos::testing::Sim sim{"bench/main_processor/configuration/robot.toml"};
  sim.add<BenchMotor>("motor");
  sim.add<talos::teleop::Teleop>("teleop");
  talos::testing::SimHardware hw{sim};
  hw.enable();
  auto packet = talos::testing::DsPacket(talos::driver_station::kEnabled |
                                         talos::driver_station::kTeleop);
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
  packet.fms.flags =
      talos::driver_station::kTeleop | talos::driver_station::kDsAttached;
  hw.enable(false);
  push();
  EXPECT_EQ(hw.motor_request("motor", "kraken").mode, Mode::kNeutral);
}

}  // namespace
}  // namespace motor
