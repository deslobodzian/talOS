#include "talOS/devices/testing.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "talOS/configuration/config_parser.h"
#include "talOS/devices/access.h"
#include "talOS/hardware/packet.h"
#include "talOS/node/sim_access.h"

namespace talos::testing {
namespace {

using namespace std::chrono_literals;
using hardware::Mode;
using Kind = detail::DeviceKind;

constexpr std::string_view kRobotToml = R"(
[robot]
period_us = 5000
command_timeout_us = 30000

[subsystems.mech.motors.braked]
can_id = 1
brake = true
[subsystems.mech.motors.coasting]
can_id = 2
brake = false
[subsystems.mech.digital_outputs.led]
dio = 0
[subsystems.mech.pwm_outputs.servo]
channel = 0

[subsystems.other.motors.motor]
can_id = 3
[subsystems.other.digital_outputs.other_led]
dio = 1
[subsystems.other.pwm_outputs.other_servo]
channel = 1
)";

class SimHardwareTest : public ::testing::Test {
 protected:
  void SetUp() override {
    sim.step(0ms);
    hw.enable();
  }

  std::uint16_t Id(std::string_view node, Kind kind,
                   std::string_view device) const {
    return detail::HardwareAccess::device_id(config, node, kind, device);
  }

  hardware::State State() const {
    const auto packet = sim.latest<hardware::Packet>(hardware::kStateTopic);
    EXPECT_TRUE(packet.has_value());
    hardware::State state{};
    if (packet) EXPECT_TRUE(hardware::Decode(packet->bytes(), state));
    return state;
  }

  hardware::MotorSample Motor(std::string_view device) const {
    const auto state = State();
    const auto id = Id("mech", Kind::kMotor, device);
    for (std::size_t i = 0; i < state.motor_count; ++i) {
      if (state.motors[i].id == id) return state.motors[i];
    }
    ADD_FAILURE() << "No state for motor " << device;
    return {};
  }

  hardware::Command Drive(std::string_view node = "mech") const {
    const auto state = State();
    hardware::Command command{};
    command.config_id = state.config_id;
    command.boot_id = state.boot_id;
    command.epoch = state.epoch;
    command.observed_time_us = state.sample_time_us;
    const auto& devices = config.subsystems.at(std::string{node});
    for (const auto id : devices.motors) {
      command.motors[command.count++] = {
          .id = id, .mode = Mode::kVelocity, .demand = 40.0};
    }
    for (const auto id : devices.digital_outputs) {
      command.digital_outputs[command.digital_output_count++] = {id, true};
    }
    for (const auto id : devices.pwm_outputs) {
      command.pwm_outputs[command.pwm_output_count++] = {id, 0.75};
    }
    return command;
  }

  // Stand in for a node that can stop publishing entirely. No hosted node
  // remains alive to keep flushing neutral requests on its behalf.
  void Send(std::string_view node, const hardware::Command& command) {
    hardware::Packet packet{};
    packet.size =
        static_cast<std::uint32_t>(hardware::Encode(command, packet.data));
    ASSERT_GT(packet.size, 0u);
    sim.publish(hardware::RequestTopic(node), packet);
  }

  Sim sim = Sim::FromToml(kRobotToml);
  config::RobotConfig config =
      config::ParseRobotConfig(detail::SimAccess::config_path(sim));
  SimHardware hw{sim};
};

TEST_F(SimHardwareTest, SilenceExpiresAllAppliedOutputsAtConfiguredDeadline) {
  hw.ideal_motors();
  Send("mech", Drive());
  sim.step(29ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);
  EXPECT_TRUE(hw.applied_digital_output("mech", "led"));
  EXPECT_DOUBLE_EQ(hw.applied_pwm("mech", "servo"), 0.75);
  EXPECT_DOUBLE_EQ(Motor("braked").velocity_rps, 40.0);
  EXPECT_DOUBLE_EQ(Motor("coasting").velocity_rps, 40.0);

  sim.step(1ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  EXPECT_EQ(hw.applied_motor("mech", "coasting").mode, Mode::kNeutral);
  EXPECT_FALSE(hw.applied_digital_output("mech", "led"));
  EXPECT_DOUBLE_EQ(hw.applied_pwm("mech", "servo"), 0.0);
  EXPECT_NEAR(Motor("braked").velocity_rps, 40.0 * (1.0 - 0.005 / 0.05), 1e-12);
  EXPECT_NEAR(Motor("coasting").velocity_rps, 40.0 * (1.0 - 0.005 / 1.0),
              1e-12);
  const auto state = State();
  EXPECT_FALSE(state.digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(state.pwm_outputs[0].output, 0.0);

  sim.step(5ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  EXPECT_LT(Motor("braked").velocity_rps, 36.0);
  EXPECT_EQ(hw.motor_request("mech", "braked").mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(hw.motor_request("mech", "braked").demand, 40.0);
  EXPECT_TRUE(hw.digital_output("mech", "led"));
  EXPECT_DOUBLE_EQ(hw.pwm("mech", "servo"), 0.75);
}

TEST_F(SimHardwareTest, OneNodesPublicationsCannotRefreshAnotherNodesRequest) {
  Send("mech", Drive());
  sim.step(20ms);
  Send("other", Drive("other"));
  sim.step(10ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  EXPECT_EQ(hw.applied_motor("other", "motor").mode, Mode::kVelocity);
  EXPECT_TRUE(hw.applied_digital_output("other", "other_led"));
  EXPECT_DOUBLE_EQ(hw.applied_pwm("other", "other_servo"), 0.75);

  sim.step(19ms);
  EXPECT_EQ(hw.applied_motor("other", "motor").mode, Mode::kVelocity);
  sim.step(1ms);
  EXPECT_EQ(hw.applied_motor("other", "motor").mode, Mode::kNeutral);
  EXPECT_FALSE(hw.applied_digital_output("other", "other_led"));
  EXPECT_DOUBLE_EQ(hw.applied_pwm("other", "other_servo"), 0.0);
}

TEST_F(SimHardwareTest, IdenticalPublicationRenewsOnlyItsOwnDeadline) {
  const auto command = Drive();
  Send("mech", command);
  sim.step(25ms);
  Send("mech", command);
  sim.step(29ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);
  EXPECT_TRUE(hw.applied_digital_output("mech", "led"));
  EXPECT_DOUBLE_EQ(hw.applied_pwm("mech", "servo"), 0.75);
  sim.step(1ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  EXPECT_FALSE(hw.applied_digital_output("mech", "led"));
  EXPECT_DOUBLE_EQ(hw.applied_pwm("mech", "servo"), 0.0);
}

TEST_F(SimHardwareTest,
       LateCollectionAndInspectionDoNotRefreshPublicationTime) {
  sim.step(
      1ms);  // Publish between hardware ticks; first collection is at 5 ms.
  Send("mech", Drive());
  sim.step(28ms);
  EXPECT_EQ(hw.motor_request("mech", "braked").mode, Mode::kVelocity);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);
  sim.step(1ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);
  sim.step(1ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  sim.step(4ms);  // First hardware tick after the deadline also sees neutral.
  EXPECT_FALSE(State().digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(State().pwm_outputs[0].output, 0.0);
}

TEST_F(SimHardwareTest, LatestRequestNeutralizesOmittedOutputsButPreservesRaw) {
  Send("mech", Drive());
  sim.step(5ms);
  ASSERT_EQ(hw.applied_motor("mech", "coasting").mode, Mode::kVelocity);
  ASSERT_TRUE(hw.applied_digital_output("mech", "led"));
  ASSERT_DOUBLE_EQ(hw.applied_pwm("mech", "servo"), 0.75);

  auto command = Drive();
  command.count = 1;
  command.motors[0].demand = 11.0;
  command.digital_output_count = 0;
  command.pwm_output_count = 0;
  Send("mech", command);
  sim.step(5ms);
  EXPECT_DOUBLE_EQ(hw.applied_motor("mech", "braked").demand, 11.0);
  EXPECT_EQ(hw.applied_motor("mech", "coasting").mode, Mode::kNeutral);
  EXPECT_FALSE(hw.applied_digital_output("mech", "led"));
  EXPECT_DOUBLE_EQ(hw.applied_pwm("mech", "servo"), 0.0);
  EXPECT_EQ(hw.motor_request("mech", "coasting").mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(hw.motor_request("mech", "coasting").demand, 40.0);
  EXPECT_TRUE(hw.digital_output("mech", "led"));
  EXPECT_DOUBLE_EQ(hw.pwm("mech", "servo"), 0.75);
  EXPECT_FALSE(State().digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(State().pwm_outputs[0].output, 0.0);

  command.count = 0;
  Send("mech", command);
  sim.step(5ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  EXPECT_DOUBLE_EQ(hw.motor_request("mech", "braked").demand, 11.0);
}

// Disabled hardware applies neutral, so even ideal motors decay instead of
// following the raw request it keeps.
TEST_F(SimHardwareTest, DisableNeutralizesAppliedOutputsWithoutChangingRaw) {
  hw.ideal_motors();
  Send("mech", Drive());
  sim.step(5ms);
  ASSERT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);
  ASSERT_DOUBLE_EQ(Motor("braked").velocity_rps, 40.0);
  ASSERT_DOUBLE_EQ(Motor("coasting").velocity_rps, 40.0);
  hw.enable(false);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  EXPECT_FALSE(hw.applied_digital_output("mech", "led"));
  EXPECT_DOUBLE_EQ(hw.applied_pwm("mech", "servo"), 0.0);
  EXPECT_EQ(hw.motor_request("mech", "braked").mode, Mode::kVelocity);
  EXPECT_TRUE(hw.digital_output("mech", "led"));
  EXPECT_DOUBLE_EQ(hw.pwm("mech", "servo"), 0.75);
  sim.step(5ms);
  EXPECT_NEAR(Motor("braked").velocity_rps, 36.0, 1e-12);
  EXPECT_NEAR(Motor("coasting").velocity_rps, 39.8, 1e-12);
  EXPECT_FALSE(State().digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(State().pwm_outputs[0].output, 0.0);
}

TEST_F(SimHardwareTest, ExplicitNeutralDecaysBrakeFasterThanCoast) {
  hw.ideal_motors();
  Send("mech", Drive());
  sim.step(5ms);
  const auto braked = Motor("braked");
  const auto coasting = Motor("coasting");
  ASSERT_DOUBLE_EQ(braked.velocity_rps, 40.0);
  ASSERT_DOUBLE_EQ(coasting.velocity_rps, 40.0);

  auto command = Drive();
  for (std::size_t i = 0; i < command.count; ++i) {
    command.motors[i].mode = Mode::kNeutral;
    command.motors[i].demand = 0.0;
  }
  Send("mech", command);
  sim.step(5ms);
  EXPECT_NEAR(Motor("braked").velocity_rps, 40.0 * (1.0 - 0.005 / 0.05), 1e-12);
  EXPECT_NEAR(Motor("coasting").velocity_rps, 40.0 * (1.0 - 0.005 / 1.0),
              1e-12);
  EXPECT_NEAR(Motor("braked").position_rot, braked.position_rot + 36.0 * 0.005,
              1e-12);
  EXPECT_GT(Motor("coasting").position_rot, coasting.position_rot);
  EXPECT_EQ(hw.motor_request("mech", "braked").mode, Mode::kNeutral);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  sim.step(500ms);  // 101 brake ticks: 40 * 0.9^101 < 0.01 rps snaps to 0.
  EXPECT_DOUBLE_EQ(Motor("braked").velocity_rps, 0.0);
  EXPECT_GT(Motor("coasting").velocity_rps, 20.0);  // ~40 * 0.995^101
}

TEST_F(SimHardwareTest, NeutralDecaysEvenBeforeAnyMotorRequest) {
  hw.ideal_motors();
  hw.set_motor("mech", "braked", 1.0, -40.0);
  hw.set_motor("mech", "coasting", 1.0, 40.0);
  sim.step(5ms);
  EXPECT_NEAR(Motor("braked").velocity_rps, -36.0, 1e-12);
  EXPECT_NEAR(Motor("braked").position_rot, 1.0 - 36.0 * 0.005, 1e-12);
  EXPECT_GT(Motor("coasting").velocity_rps, 39.0);
  EXPECT_GT(Motor("coasting").position_rot, 1.0);
}

TEST_F(SimHardwareTest, ZeroVelocityIsAnActiveStopRatherThanNeutralDecay) {
  hw.ideal_motors();
  Send("mech", Drive());
  sim.step(5ms);
  const auto position = Motor("braked").position_rot;
  auto command = Drive();
  command.motors[0].demand = 0.0;
  command.motors[1].mode = Mode::kNeutral;
  command.motors[1].demand = 0.0;
  Send("mech", command);
  sim.step(5ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(Motor("braked").velocity_rps, 0.0);
  EXPECT_DOUBLE_EQ(Motor("braked").position_rot, position);
  EXPECT_GT(Motor("coasting").velocity_rps, 39.0);
}

}  // namespace
}  // namespace talos::testing
