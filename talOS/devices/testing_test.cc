#include "talOS/devices/testing.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/devices/access.h"
#include "talOS/devices/devices.h"
#include "talOS/driver_station/driver_station.h"
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
command_timeout_us = 50000

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
[subsystems.mech.sensors.angle]
type = "CANcoder"
can_id = 4
[subsystems.mech.motors.remote]
can_id = 5
feedback = "RemoteCANcoder"
feedback_sensor = "angle"
[subsystems.mech.motors.follower]
can_id = 6
follow = "braked"

[subsystems.other.motors.motor]
can_id = 3
[subsystems.other.digital_outputs.other_led]
dio = 1
[subsystems.other.pwm_outputs.other_servo]
channel = 1
)";

hardware::State LatestState(const Sim& sim) {
  const auto packet = sim.latest<hardware::Packet>(hardware::kStateTopic);
  EXPECT_TRUE(packet.has_value());
  hardware::State state{};
  if (packet) {
    EXPECT_TRUE(hardware::Decode(packet->bytes(), state));
  }
  return state;
}

// A request observing `state` that drives every output of `devices`.
hardware::Command DriveAll(const hardware::State& state,
                           const hardware::Devices& devices) {
  hardware::Command command{};
  command.config_id = state.config_id;
  command.boot_id = state.boot_id;
  command.epoch = state.epoch;
  command.observed_time_us = state.sample_time_us;
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
void SendRequest(Sim& sim, std::string_view node,
                 const hardware::Command& command) {
  hardware::Packet packet{};
  packet.size =
      static_cast<std::uint32_t>(hardware::Encode(command, packet.data));
  ASSERT_GT(packet.size, 0u);
  sim.publish(hardware::RequestTopic(node), packet);
}

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

  hardware::State State() const { return LatestState(sim); }

  hardware::MotorSample Motor(std::string_view device) const {
    const auto state = State();
    const auto id = Id("mech", Kind::kMotor, device);
    for (std::size_t i = 0; i < state.motor_count; ++i) {
      if (state.motors[i].id == id) return state.motors[i];
    }
    ADD_FAILURE() << "No state for motor " << device;
    return {};
  }

  hardware::SensorSample Sensor(std::string_view device) const {
    const auto state = State();
    const auto id = Id("mech", Kind::kCANcoder, device);
    for (std::size_t i = 0; i < state.sensor_count; ++i) {
      if (state.sensors[i].id == id) return state.sensors[i];
    }
    ADD_FAILURE() << "No state for sensor " << device;
    return {};
  }

  hardware::Command Drive(std::string_view node = "mech") const {
    return DriveAll(State(), config.subsystems.at(std::string{node}));
  }

  void Send(std::string_view node, const hardware::Command& command) {
    SendRequest(sim, node, command);
  }

  Sim sim = Sim::FromToml(kRobotToml);
  config::RobotConfig config =
      config::ParseRobotConfig(detail::SimAccess::config_path(sim));
  SimHardware hw{sim};
};

// The bridge's owner timeout here is max(2 x 5 ms, 50 ms / 2) = 25 ms: a
// silent node is held while its last request is at most that old.
TEST_F(SimHardwareTest, SilenceExpiresAllAppliedOutputsAtTheOwnerTimeout) {
  hw.ideal_motors();
  Send("mech", Drive());
  sim.step(25ms);
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
  sim.step(4ms);  // the first hardware tick after the deadline
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
  sim.step(6ms);
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
  sim.step(10ms);
  Send("mech", command);
  sim.step(25ms);
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
  sim.step(24ms);
  EXPECT_EQ(hw.motor_request("mech", "braked").mode, Mode::kVelocity);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);
  sim.step(1ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);
  sim.step(1ms);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  sim.step(3ms);  // First hardware tick after the deadline also sees neutral.
  EXPECT_FALSE(State().digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(State().pwm_outputs[0].output, 0.0);
}

// The merged command carries the oldest observation of any node still merged,
// so a silent node whose last request observed an old State stops every
// output once that observation is command_timeout_us old, as the gateway
// does; only requests built from a State of the new epoch drive again.
TEST_F(SimHardwareTest, SilentOwnerWithAnOldObservationStopsEveryOutput) {
  const auto old = Drive();  // observed at 0 ms
  sim.step(30ms);
  Send("mech", old);  // admitted: 30 ms old, merged until 55 ms
  for (int tick = 0; tick < 3; ++tick) {
    Send("other", Drive("other"));
    sim.step(5ms);
  }
  ASSERT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);
  ASSERT_EQ(hw.applied_motor("other", "motor").mode, Mode::kVelocity);
  const auto before = Drive("other");
  Send("other", before);
  sim.step(5ms);  // 50 ms: the merged observation is 50 ms old
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  EXPECT_EQ(hw.applied_motor("other", "motor").mode, Mode::kNeutral);
  EXPECT_FALSE(hw.applied_digital_output("other", "other_led"));
  EXPECT_DOUBLE_EQ(hw.applied_pwm("other", "other_servo"), 0.0);

  Send("other", before);
  EXPECT_EQ(hw.applied_motor("other", "motor").mode, Mode::kNeutral);
  Send("other", Drive("other"));
  EXPECT_EQ(hw.applied_motor("other", "motor").mode, Mode::kVelocity);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
}

// The merged-observation check is the gateway's, on its tick: an inspection
// at the instant a merged observation turns command_timeout_us old, between
// ticks, stops nothing when that owner has left the merge by the next tick.
TEST_F(SimHardwareTest, InspectionBetweenTicksNeverInvalidates) {
  sim.step(5ms);
  hw.set_gateway_clock(0ms, 3ms);  // States sampled 3 ms before each tick
  sim.step(5ms);
  const auto old = Drive();  // observed at gateway 7 ms
  sim.step(22ms);
  Send("mech", old);  // 32 ms: merged through 57 ms, when 7 ms is 50 ms old
  sim.step(25ms);
  Send("other", Drive("other"));
  EXPECT_EQ(hw.applied_motor("other", "motor").mode, Mode::kVelocity);
  sim.step(3ms);  // 60 ms: the tick finds mech already out of the merge
  EXPECT_EQ(State().epoch, 1u);
  EXPECT_EQ(hw.applied_motor("other", "motor").mode, Mode::kVelocity);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
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

TEST_F(SimHardwareTest, ResetAndRebootRefuseRequestsAsTheGatewayDoes) {
  Send("mech", Drive());
  sim.step(5ms);
  ASSERT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);
  const hardware::Command before_reset = Drive();
  hw.reset_device("mech", "braked");
  sim.step(5ms);  // the first of two masking States
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  EXPECT_EQ(hw.applied_motor("mech", "coasting").mode, Mode::kVelocity);
  sim.step(5ms);  // the second
  const hardware::Command while_masked = Drive();
  sim.step(5ms);  // restored
  ASSERT_TRUE(Motor("braked").valid);
  // A request observed before the reset, or on the last State that masked
  // the motor, never drives it.
  Send("mech", before_reset);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  EXPECT_EQ(hw.applied_motor("mech", "coasting").mode, Mode::kVelocity);
  Send("mech", while_masked);
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  Send("mech", Drive());
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kVelocity);

  // A request built from the previous boot's State is ignored.
  hardware::Command old_boot = Drive();
  old_boot.motors[0].demand = 10.0;
  hw.reboot();  // the rebooted gateway starts neutral
  EXPECT_EQ(hw.applied_motor("mech", "braked").mode, Mode::kNeutral);
  sim.step(5ms);
  Send("mech", old_boot);
  EXPECT_DOUBLE_EQ(hw.motor_request("mech", "braked").demand, 40.0);

  EXPECT_THROW(hw.reset_device("mech", "led"), std::invalid_argument);
}

// REQ-SAFE-22: the gateway's feedback age bound (hardware::FeedbackFresh),
// applied here too: a motor or CAN sensor whose position or velocity sample
// is command_timeout_us (50 ms here) old or older, or of unknown age, is
// invalid in the State; a microsecond younger is valid. Only that device.
TEST_F(SimHardwareTest, FeedbackAsOldAsTheCommandTimeoutIsInvalid) {
  constexpr std::uint32_t kUnknown = std::numeric_limits<std::uint32_t>::max();
  struct Case {
    std::uint32_t position_age_us;
    std::uint32_t velocity_age_us;
    bool valid;
  };
  for (const Case c : {Case{49'999, 49'999, true}, Case{50'000, 0, false},
                       Case{0, 50'000, false}, Case{kUnknown, 0, false},
                       Case{0, kUnknown, false}}) {
    SCOPED_TRACE(std::to_string(c.position_age_us) + " " +
                 std::to_string(c.velocity_age_us));
    hw.set_motor("mech", "braked", 1.0, 2.0, c.position_age_us,
                 c.velocity_age_us);
    hw.set_cancoder("mech", "angle", 0.25, 0.5, c.position_age_us,
                    c.velocity_age_us);
    sim.step(5ms);
    EXPECT_EQ(Motor("braked").valid, c.valid);
    EXPECT_EQ(Sensor("angle").valid, c.valid);
    EXPECT_TRUE(Motor("coasting").valid);
  }
}

// REQ-SAFE-22: as on the gateway, a motor that cannot drive in a State (its
// feedback disconnected or too old, its remote CANcoder's, or a follower's
// master's) applies neutral from that tick, with no new request, and so does
// a request built while it could not; the others keep driving in the same
// epoch. A follower's sample reads invalid while its master's does, as the
// gateway publishes it. After recovery the request in force, or one observed
// on the last State that showed it unable, still leaves it neutral; one
// observing a later State drives it.
TEST_F(SimHardwareTest, AMotorThatCannotDriveIsNeutralFromThatTick) {
  struct Case {
    const char* name;
    std::function<void(bool)> lose;
    std::vector<std::string_view> unable;
  };
  const std::vector<Case> cases = {
      {"disconnected",
       [&](bool lost) { hw.set_connected("mech", "braked", !lost); },
       {"braked", "follower"}},
      {"aged",
       [&](bool lost) {
         hw.set_motor("mech", "braked", 0, 0, lost ? 50'000 : 0, 0);
       },
       {"braked", "follower"}},
      {"remote CANcoder aged",
       [&](bool lost) {
         hw.set_cancoder("mech", "angle", 0, 0, lost ? 50'000 : 0, 0);
       },
       {"remote"}},
  };
  const auto expect = [&](std::string_view motor, Mode mode) {
    EXPECT_EQ(hw.applied_motor("mech", motor).mode, mode) << motor;
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    Send("mech", Drive());
    for (const auto motor : {"braked", "follower", "remote"}) {
      expect(motor, Mode::kVelocity);
    }
    const std::uint64_t epoch = State().epoch;
    c.lose(true);
    sim.step(5ms);  // the first State that shows it
    EXPECT_EQ(Motor("follower").valid, Motor("braked").valid);
    for (const auto motor : c.unable) expect(motor, Mode::kNeutral);
    expect("coasting", Mode::kVelocity);
    EXPECT_EQ(State().epoch, epoch);
    const hardware::Command while_unable = Drive();
    Send("mech", while_unable);
    for (const auto motor : c.unable) expect(motor, Mode::kNeutral);
    expect("coasting", Mode::kVelocity);

    c.lose(false);
    sim.step(5ms);  // recovered, the last request still in force
    for (const auto motor : c.unable) expect(motor, Mode::kNeutral);
    Send("mech", while_unable);
    for (const auto motor : c.unable) expect(motor, Mode::kNeutral);
    Send("mech", Drive());
    for (const auto motor : c.unable) expect(motor, Mode::kVelocity);
  }
}

// The RoboRIO reports a CANcoder's absolute position with
// AbsoluteSensorDiscontinuityPoint = 0.5, so in [-0.5, 0.5) rotations;
// set_cancoder's position goes out wrapped the same way.
TEST_F(SimHardwareTest, CancoderPositionWrapsAsTheRoboRioReportsIt) {
  struct Case {
    double set;
    double reported;
  };
  for (const Case c : {Case{0.25, 0.25}, Case{0.75, -0.25}, Case{0.5, -0.5},
                       Case{-0.5, -0.5}, Case{-0.75, 0.25}, Case{2.25, 0.25}}) {
    SCOPED_TRACE(c.set);
    hw.set_cancoder("mech", "angle", c.set, 1.0);
    sim.step(5ms);
    EXPECT_DOUBLE_EQ(Sensor("angle").position_rot, c.reported);
    EXPECT_DOUBLE_EQ(Sensor("angle").velocity_rps, 1.0);
  }
}

// REQ-SAFE-22, REQ-SAFE-18: as on the gateway (Gateway::Tick), the State
// that finds every motor and CAN sensor invalid raises kHardwareFault and
// starts a new epoch, every output goes neutral, including digital and PWM
// outputs no feedback gates, and the node's robot_enabled() falls. An invalid
// sample reads zero. When feedback returns, the fault clears in the same
// epoch and a request built from a State after it drives again.
TEST(SimHardwareTotalLoss, LosingEveryFeedbackSampleFaultsAndNeutralizesAll) {
  struct Other {
    explicit Other(Node& n)
        : node{n},
          motor{n, "motor"},
          led{n, "other_led"},
          servo{n, "other_servo"} {}
    void update() {
      enabled = robot_enabled(node);
      motor.set_voltage(3.0);
      led.set(true);
      servo.set(0.5);
    }
    Node& node;
    Motor motor;
    DigitalOutput led;
    PwmOutput servo;
    bool enabled = false;
  };
  auto sim = Sim::FromToml(kRobotToml);
  auto& other = sim.add<Other>("other");
  SimHardware hw{sim};
  hw.enable();
  hw.set_motor("other", "motor", 2.0, 3.0, 0, 0, 30.0, 4.0, 5.0);
  const auto state = [&] {
    const auto packet = sim.latest<hardware::Packet>(hardware::kStateTopic);
    hardware::State s{};
    if (packet) {
      EXPECT_TRUE(hardware::Decode(packet->bytes(), s));
    }
    return s;
  };
  const auto faulted = [&] {
    return (state().flags & hardware::kHardwareFault) != 0;
  };
  const auto expect_outputs = [&](bool driving) {
    EXPECT_EQ(hw.applied_motor("other", "motor").mode,
              driving ? Mode::kVoltage : Mode::kNeutral);
    EXPECT_EQ(hw.applied_digital_output("other", "other_led"), driving);
    EXPECT_DOUBLE_EQ(hw.applied_pwm("other", "other_servo"),
                     driving ? 0.5 : 0.0);
  };
  sim.step(20ms);
  ASSERT_TRUE(other.enabled);
  ASSERT_FALSE(faulted());
  expect_outputs(true);
  const std::uint64_t epoch = state().epoch;

  // Every motor and the CANcoder: one aged, the rest disconnected.
  hw.set_motor("other", "motor", 2.0, 3.0, 50'000, 0, 30.0, 4.0, 5.0);
  for (const auto device :
       {"braked", "coasting", "remote", "follower", "angle"}) {
    hw.set_connected("mech", device, false);
  }
  sim.step(5ms);  // the State that finds total loss
  EXPECT_TRUE(faulted());
  EXPECT_EQ(state().epoch, epoch + 1);
  const std::uint16_t aged_id = detail::HardwareAccess::device_id(
      config::ParseRobotConfig(detail::SimAccess::config_path(sim)), "other",
      Kind::kMotor, "motor");
  hardware::MotorSample aged{.valid = true};
  for (std::size_t i = 0; i < state().motor_count; ++i) {
    if (state().motors[i].id == aged_id) aged = state().motors[i];
  }
  EXPECT_FALSE(aged.valid);
  EXPECT_EQ(aged.position_rot, 0.0);
  EXPECT_EQ(aged.velocity_rps, 0.0);
  EXPECT_EQ(aged.stator_current_a, 0.0);
  EXPECT_EQ(aged.supply_current_a, 0.0);
  expect_outputs(false);
  sim.step(10ms);  // every output stays neutral while the fault holds
  EXPECT_FALSE(other.enabled);
  expect_outputs(false);
  for (const auto& sample : state().digital_outputs) EXPECT_FALSE(sample.value);
  for (const auto& sample : state().pwm_outputs) {
    EXPECT_DOUBLE_EQ(sample.output, 0.0);
  }
  EXPECT_EQ(state().epoch, epoch + 1);  // one new epoch, not one per tick

  hw.set_motor("other", "motor", 2.0, 3.0);
  sim.step(5ms);  // recovered: no fault, same epoch
  EXPECT_FALSE(faulted());
  EXPECT_EQ(state().epoch, epoch + 1);
  sim.step(10ms);
  EXPECT_TRUE(other.enabled);
  expect_outputs(true);
}

// REQ-SAFE-22, REQ-DEV-07: the gateway judges total loss on the samples the
// RoboRIO reports, before it masks a follower of an invalid master or the
// motor a reset CANcoder feeds (Gateway::Tick judges, PollResets masks
// after). With such a pair as the only CAN devices, disconnecting the master
// or resetting the CANcoder leaves the other sample valid where the gateway
// judges: the dependent reads invalid, but there is no kHardwareFault, no new
// epoch, and the digital output keeps its request. The gateway's side of the
// same two scenarios: Gateway.AMaskedDependentIsNotTotalLoss.
TEST(SimHardwareTotalLoss, AMaskedDependentIsNotTotalLoss) {
  struct Case {
    const char* name;
    std::string_view toml;
    const char* lost;       // disconnected (a motor) or reset (a CANcoder)
    const char* dependent;  // masked with it, after the judgment
  };
  constexpr std::string_view kFollowed = R"(
[robot]
period_us = 5000
command_timeout_us = 50000
[subsystems.mech.motors.master]
can_id = 1
[subsystems.mech.motors.follower]
can_id = 2
follow = "master"
[subsystems.mech.digital_outputs.led]
dio = 0
)";
  constexpr std::string_view kRemote = R"(
[robot]
period_us = 5000
command_timeout_us = 50000
[subsystems.mech.sensors.angle]
type = "CANcoder"
can_id = 4
[subsystems.mech.motors.remote]
can_id = 5
feedback = "RemoteCANcoder"
feedback_sensor = "angle"
[subsystems.mech.digital_outputs.led]
dio = 0
)";
  for (const Case& c :
       {Case{"master disconnected", kFollowed, "master", "follower"},
        Case{"remote CANcoder reset", kRemote, "angle", "remote"}}) {
    SCOPED_TRACE(c.name);
    auto sim = Sim::FromToml(c.toml);
    const auto config =
        config::ParseRobotConfig(detail::SimAccess::config_path(sim));
    const auto& devices = config.subsystems.at("mech");
    SimHardware hw{sim};
    sim.step(0ms);
    hw.enable();
    SendRequest(sim, "mech", DriveAll(LatestState(sim), devices));
    sim.step(5ms);
    ASSERT_TRUE(hw.applied_digital_output("mech", "led"));
    const std::uint64_t epoch = LatestState(sim).epoch;

    if (std::string_view{c.lost} == "master") {
      hw.set_connected("mech", c.lost, false);
    } else {
      hw.reset_device("mech", c.lost);
    }
    sim.step(5ms);
    const auto state = LatestState(sim);
    const auto dependent = detail::HardwareAccess::device_id(
        config, "mech", Kind::kMotor, c.dependent);
    for (std::size_t i = 0; i < state.motor_count; ++i) {
      if (state.motors[i].id == dependent) {
        EXPECT_FALSE(state.motors[i].valid) << c.dependent;
      }
    }
    EXPECT_EQ(state.flags & hardware::kHardwareFault, 0u);
    EXPECT_EQ(state.epoch, epoch);
    EXPECT_TRUE(hw.applied_digital_output("mech", "led"));
  }
}

// The RoboRIO observes the mode every tick, delivered or not: a disable lost
// in transit still ends the generation, so the next enabled packet is not
// mistaken for a continuation of the one before the outage.
TEST_F(SimHardwareTest, ModeChangeOnALostTickStartsANewGeneration) {
  const auto mode_since_us = [&] {
    const auto packet =
        sim.latest<hardware::Packet>(hardware::kDriverStationTopic);
    EXPECT_TRUE(packet.has_value());
    driver_station::DriverStationData data{};
    if (packet) {
      EXPECT_TRUE(driver_station::Decode(packet->bytes(), data));
    }
    return data.mode_since_us;
  };
  driver_station::DriverStationData teleop{};
  teleop.fms.flags = driver_station::kEnabled | driver_station::kTeleop |
                     driver_station::kDsAttached;
  driver_station::DriverStationData disabled = teleop;
  disabled.fms.flags &= ~driver_station::kEnabled;

  hw.set_driver_station(teleop);
  sim.step(10ms);
  const std::uint64_t before = mode_since_us();
  ASSERT_NE(before, 0u);
  hw.set_driver_station(disabled);
  hw.lose_states(1);
  sim.step(5ms);
  hw.set_driver_station(teleop);
  sim.step(5ms);
  EXPECT_GT(mode_since_us(), before);
}

}  // namespace
}  // namespace talos::testing
