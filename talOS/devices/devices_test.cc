#include "talOS/devices/devices.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "talOS/configuration/config_parser.h"
#include "talOS/devices/testing.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/node/testing.h"

namespace talos {
namespace {

using namespace std::chrono_literals;
using hardware::Mode;
using hardware::Packet;
using testing::Sim;
using testing::SimHardware;

constexpr const char* kRobotToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.mech.motors.a]
can_id = 1
[subsystems.mech.motors.b]
can_id = 2
[subsystems.mech.digital_outputs.led]
dio = 1
[subsystems.mech.pwm_outputs.servo]
channel = 0
[subsystems.mech.digital_inputs.beam]
dio = 0
[subsystems.mech.analog_inputs.pressure]
channel = 0
[subsystems.mech.encoders.enc]
type = "DutyCycleEncoder"
dio = 2
[subsystems.mech.sensors.cc]
type = "CANcoder"
can_id = 3
[subsystems.mech.sensors.gyro]
type = "Pigeon2"
can_id = 4

[subsystems.watcher.digital_inputs.switch]
dio = 3
)";

constexpr const char* kRequestTopic = "/hw/request/mech";

// Every handle kind, with what update() commands switched from the test.
struct Mech {
  explicit Mech(Node& n)
      : node{n},
        a{n, "a"},
        b{n, "b"},
        led{n, "led"},
        servo{n, "servo"},
        beam{n, "beam"},
        pressure{n, "pressure"},
        enc{n, "enc"},
        cc{n, "cc"},
        gyro{n, "gyro"} {}

  void update() {
    if (a_velocity) a.set_velocity(*a_velocity, 0.5, 1);
    if (b_motion_magic) b.set_motion_magic(*b_motion_magic);
    if (led_on) led.set(*led_on);
    if (servo_out) servo.set(*servo_out);
    enabled_in_update = robot_enabled(node);
  }

  Node& node;
  Motor a, b;
  DigitalOutput led;
  PwmOutput servo;
  DigitalInput beam;
  AnalogInput pressure;
  Encoder enc;
  CANcoder cc;
  Gyro gyro;

  std::optional<double> a_velocity, b_motion_magic, servo_out;
  std::optional<bool> led_on;
  bool enabled_in_update = false;
};

// Requests flushed on `topic` so far.
std::uint64_t Sent(const Sim& sim, std::string_view topic = kRequestTopic) {
  return detail::SimAccess::message_count(sim, topic, sizeof(Packet));
}

std::optional<hardware::Command> LastCommand(const Sim& sim) {
  const auto packet = sim.latest<Packet>(kRequestTopic);
  if (!packet) return std::nullopt;
  hardware::Command command{};
  EXPECT_TRUE(hardware::Decode(packet->bytes(), command));
  return command;
}

TEST(Devices, NothingIsSentBeforeTheFirstAcceptedState) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& mech = sim.add<Mech>("mech");
  mech.a_velocity = 10;
  sim.step(50ms);
  EXPECT_EQ(Sent(sim), 0u);
  EXPECT_FALSE(mech.a.connected());
  EXPECT_EQ(mech.a.position(), 0.0);

  SimHardware hw{sim};  // the first state goes out now
  sim.step(10ms);
  EXPECT_GT(Sent(sim), 0u);
  EXPECT_TRUE(mech.a.connected());
}

TEST(Devices, MotorsAreNeutralUntilEnabledAndEchoTheStateIdentity) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& mech = sim.add<Mech>("mech");
  SimHardware hw{sim};
  mech.a_velocity = 12.5;
  sim.step(20ms);

  EXPECT_FALSE(mech.enabled_in_update);
  EXPECT_EQ(hw.motor_request("mech", "a").mode, Mode::kNeutral);
  const auto command = LastCommand(sim);
  if (!command.has_value()) {
    FAIL() << "no command sent";
  }
  EXPECT_EQ(command->count, 2);  // every owned motor, every flush
  EXPECT_EQ(command->boot_id, 1u);
  EXPECT_EQ(command->epoch, 1u);
  const auto& link = sim.node("mech").extension<HardwareLink>();
  EXPECT_EQ(command->config_id,
            hardware::ConfigurationId(link.config().hardware));
  EXPECT_GT(command->observed_time_us, 0u);
  EXPECT_LE(command->observed_time_us, 20000u);

  hw.enable();
  sim.step(10ms);
  EXPECT_TRUE(mech.enabled_in_update);
  const auto a = hw.motor_request("mech", "a");
  EXPECT_EQ(a.mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(a.demand, 12.5);
  EXPECT_DOUBLE_EQ(a.feedforward_v, 0.5);
  EXPECT_EQ(a.slot, 1);
  EXPECT_EQ(hw.motor_request("mech", "b").mode, Mode::kNeutral);  // unset

  mech.a_velocity.reset();  // no longer set: neutral on the next flush
  sim.step(10ms);
  EXPECT_EQ(hw.motor_request("mech", "a").mode, Mode::kNeutral);

  hw.enable(false);
  mech.a_velocity = 3;
  sim.step(10ms);
  EXPECT_EQ(hw.motor_request("mech", "a").mode, Mode::kNeutral);
}

TEST(Devices, StaleStateForcesNeutralButKeepsFlushing) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& mech = sim.add<Mech>("mech");
  auto hw = std::make_unique<SimHardware>(sim);
  hw->enable();
  mech.a_velocity = 20;
  sim.step(20ms);
  const auto first = LastCommand(sim);
  if (!first.has_value()) {
    FAIL() << "no command sent";
  }
  ASSERT_EQ(first->motors[0].mode, Mode::kVelocity);

  hw.reset();       // the bridge goes silent
  sim.step(150ms);  // past [robot] command_timeout_us
  EXPECT_FALSE(robot_enabled(sim.node("mech")));
  const std::uint64_t sent = Sent(sim);
  sim.step(10ms);
  EXPECT_EQ(Sent(sim), sent + 2);
  const auto command = LastCommand(sim);
  if (!command.has_value()) {
    FAIL() << "no command sent";
  }
  EXPECT_EQ(command->motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(command->motors[0].demand, 0.0);
}

TEST(Devices, DigitalAndPwmOutputsRideAlongOnlyWhenSet) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& mech = sim.add<Mech>("mech");
  SimHardware hw{sim};
  hw.enable();
  mech.led_on = true;
  mech.servo_out = 0.25;
  sim.step(10ms);

  EXPECT_TRUE(hw.digital_output("mech", "led"));
  EXPECT_DOUBLE_EQ(hw.pwm("mech", "servo"), 0.25);
  auto command = LastCommand(sim);
  if (!command.has_value()) {
    FAIL() << "no command sent";
  }
  EXPECT_EQ(command->digital_output_count, 1);
  EXPECT_EQ(command->pwm_output_count, 1);
  EXPECT_DOUBLE_EQ(command->pwm_outputs[0].output, 0.25);

  mech.led_on.reset();
  mech.servo_out.reset();
  sim.step(10ms);
  command = LastCommand(sim);
  if (!command.has_value()) {
    FAIL() << "no command sent";
  }
  EXPECT_EQ(command->digital_output_count, 0);
  EXPECT_EQ(command->pwm_output_count, 0);
}

TEST(Devices, GettersReadTheLatchedFeedback) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& mech = sim.add<Mech>("mech");
  SimHardware hw{sim};
  hw.set_motor("mech", "a", 1.5, -2.0);
  hw.set_digital_input("mech", "beam", true);
  hw.set_analog_input("mech", "pressure", 3.3);
  hw.set_encoder("mech", "enc", 0.75, 0.1);
  hw.set_cancoder("mech", "cc", 0.2, 0.3);
  hw.set_gyro("mech", "gyro", 0.125, -0.5);
  sim.step(10ms);

  EXPECT_TRUE(mech.a.connected());
  EXPECT_DOUBLE_EQ(mech.a.position(), 1.5);
  EXPECT_DOUBLE_EQ(mech.a.velocity(), -2.0);
  EXPECT_TRUE(mech.beam.get());
  EXPECT_TRUE(mech.beam.connected());
  EXPECT_DOUBLE_EQ(mech.pressure.voltage(), 3.3);
  EXPECT_DOUBLE_EQ(mech.enc.position(), 0.75);
  EXPECT_DOUBLE_EQ(mech.enc.velocity(), 0.1);
  EXPECT_DOUBLE_EQ(mech.cc.position(), 0.2);
  EXPECT_DOUBLE_EQ(mech.cc.velocity(), 0.3);
  EXPECT_DOUBLE_EQ(mech.gyro.yaw(), 0.125);
  EXPECT_DOUBLE_EQ(mech.gyro.yaw_rate(), -0.5);
}

TEST(Devices, IdealMotorsReachTheirRequests) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& mech = sim.add<Mech>("mech");
  SimHardware hw{sim};
  hw.enable();
  hw.ideal_motors();
  mech.a_velocity = 40;
  mech.b_motion_magic = 3.0;
  sim.step(30ms);
  EXPECT_DOUBLE_EQ(mech.a.velocity(), 40);
  EXPECT_GT(mech.a.position(), 0.0);
  EXPECT_DOUBLE_EQ(mech.b.position(), 3.0);
}

TEST(Devices, NonFiniteDemandIsNeutral) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& mech = sim.add<Mech>("mech");
  SimHardware hw{sim};
  hw.enable();
  mech.a_velocity = std::numeric_limits<double>::quiet_NaN();
  sim.step(10ms);
  EXPECT_EQ(hw.motor_request("mech", "a").mode, Mode::kNeutral);
}

struct Watcher {
  explicit Watcher(Node& n) : node{n}, sw{n, "switch"} {}
  Node& node;
  DigitalInput sw;
};

Packet StatePacket(std::uint64_t config_id, std::uint64_t sample_time_us,
                   bool enabled) {
  hardware::State state{};
  state.config_id = config_id;
  state.boot_id = 7;
  state.sample_time_us = sample_time_us;
  state.flags = hardware::kConfigured | (enabled ? hardware::kEnabled : 0u);
  Packet packet{};
  packet.size =
      static_cast<std::uint32_t>(hardware::Encode(state, packet.data));
  return packet;
}

TEST(Devices, AcceptStateRejectsAnotherConfigAndStaleSamples) {
  auto sim = Sim::FromToml(kRobotToml);
  sim.add<Watcher>("watcher");
  Node& node = sim.node("watcher");
  const std::uint64_t id = hardware::ConfigurationId(
      node.extension<HardwareLink>().config().hardware);

  sim.publish(hardware::kStateTopic, StatePacket(id + 1, 1000, true));
  sim.step(2ms);
  EXPECT_FALSE(robot_enabled(node)) << "another config's state was accepted";

  sim.publish(hardware::kStateTopic, StatePacket(id, 1000, true));
  sim.step(2ms);
  EXPECT_TRUE(robot_enabled(node));

  sim.publish(hardware::kStateTopic, StatePacket(id, 1000, false));  // dup
  sim.publish(hardware::kStateTopic, StatePacket(id, 500, false));   // older
  sim.step(2ms);
  EXPECT_TRUE(robot_enabled(node)) << "a stale sample replaced a newer one";

  sim.publish(hardware::kStateTopic, StatePacket(id, 2000, false));
  sim.step(2ms);
  EXPECT_FALSE(robot_enabled(node));
}

TEST(Devices, InputOnlyNodesSendNoRequest) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& watcher = sim.add<Watcher>("watcher");
  SimHardware hw{sim};
  hw.enable();
  hw.set_digital_input("watcher", "switch", true);
  sim.step(20ms);
  EXPECT_TRUE(watcher.sw.get());
  EXPECT_EQ(Sent(sim, "/hw/request/watcher"), 0u);
}

struct Misnamed {
  explicit Misnamed(Node& n) : motor{n, "c"} {}
  Motor motor;
};

TEST(Devices, UnknownDeviceThrowsNamingIt) {
  auto sim = Sim::FromToml(kRobotToml);
  try {
    sim.add<Misnamed>("mech");
    FAIL() << "an unknown device must throw";
  } catch (const std::invalid_argument& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("motor named 'c'"), std::string::npos) << what;
    EXPECT_NE(what.find("it has: a, b"), std::string::npos) << what;
  }
  EXPECT_THROW(SimHardware{sim}.motor_request("mech", "c"),
               std::invalid_argument);
}

TEST(Devices, DisconnectedDevicesReadInvalidAndZero) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& mech = sim.add<Mech>("mech");
  SimHardware hw{sim};
  hw.set_motor("mech", "a", 1.5, -2.0);
  hw.set_motor("mech", "b", 0.5, 0.0);
  hw.set_cancoder("mech", "cc", 0.2, 0.3);
  hw.set_gyro("mech", "gyro", 0.125, -0.5);
  hw.set_connected("mech", "a", false);
  hw.set_connected("mech", "cc", false);
  hw.set_connected("mech", "beam", false);
  sim.step(10ms);

  EXPECT_FALSE(mech.a.connected());
  EXPECT_EQ(mech.a.position(), 0.0);
  EXPECT_EQ(mech.a.velocity(), 0.0);
  EXPECT_TRUE(mech.b.connected());
  EXPECT_DOUBLE_EQ(mech.b.position(), 0.5);
  EXPECT_FALSE(mech.cc.connected());
  EXPECT_EQ(mech.cc.position(), 0.0);
  EXPECT_TRUE(mech.gyro.connected());  // the same sensors table, its own id
  EXPECT_DOUBLE_EQ(mech.gyro.yaw(), 0.125);
  EXPECT_FALSE(mech.beam.connected());
  EXPECT_TRUE(mech.pressure.connected());

  hw.set_connected("mech", "a", true);
  sim.step(10ms);
  EXPECT_TRUE(mech.a.connected());
  EXPECT_DOUBLE_EQ(mech.a.position(), 1.5);

  try {
    hw.set_connected("mech", "c", false);
    FAIL() << "an unknown device must throw";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string{e.what()}.find("'c'"), std::string::npos) << e.what();
  }
}

// Two handles on one node share its subscription and both hear each packet.
struct Pilot {
  explicit Pilot(Node& n)
      : ds{n, [this] { ++packets; }}, second{n, [this] { ++second_packets; }} {}
  DriverStation ds;
  DriverStation second;
  int packets = 0;
  int second_packets = 0;
};

driver_station::DriverStationData TeleopStick() {
  driver_station::DriverStationData data{};
  data.fms.flags = driver_station::kDsAttached | driver_station::kEnabled |
                   driver_station::kTeleop;
  data.fms.alliance = driver_station::Alliance::kBlue;
  data.fms.station = 3;
  data.fms.match_time_s = 42.5f;
  auto& stick = data.joysticks[1];
  stick.connected = true;
  stick.axis_count = 2;
  stick.axes = {0.5f, -1.0f, 0.75f};  // the third is past axis_count
  stick.button_count = 4;
  stick.buttons = 0b1001;  // buttons 1 and 4
  stick.pov_count = 1;
  stick.povs[0] = 270;
  std::strncpy(stick.name, "Xbox", sizeof stick.name);
  return data;
}

TEST(Devices, DriverStationReadsTheForwardedPacket) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& pilot = sim.add<Pilot>("watcher");
  auto hw = std::make_unique<SimHardware>(sim);
  sim.step(10ms);
  const DriverStation& ds = pilot.ds;
  EXPECT_EQ(pilot.packets, 0);  // nothing set: nothing forwarded
  EXPECT_FALSE(ds.connected());
  EXPECT_FALSE(ds.enabled());
  EXPECT_EQ(ds.alliance(), driver_station::Alliance::kUnknown);

  hw->set_driver_station(TeleopStick());
  sim.step(10ms);
  EXPECT_GT(pilot.packets, 0);
  EXPECT_EQ(pilot.second_packets, pilot.packets);
  EXPECT_TRUE(ds.connected());
  EXPECT_TRUE(ds.enabled());
  EXPECT_TRUE(ds.teleop());
  EXPECT_FALSE(ds.autonomous());
  EXPECT_FALSE(ds.test());
  EXPECT_FALSE(ds.estop());
  EXPECT_FALSE(ds.fms_attached());
  EXPECT_EQ(ds.alliance(), driver_station::Alliance::kBlue);
  EXPECT_EQ(ds.station(), 3);
  EXPECT_DOUBLE_EQ(ds.match_time(), 42.5);
  EXPECT_GT(ds.data().sample_time_us, 0u);  // stamped like the Rio's

  EXPECT_FALSE(ds.joystick(0).connected());
  const Joystick stick = ds.joystick(1);
  EXPECT_TRUE(stick.connected());
  EXPECT_EQ(stick.axis_count(), 2);
  EXPECT_DOUBLE_EQ(stick.axis(0), 0.5);
  EXPECT_DOUBLE_EQ(stick.axis(1), -1.0);
  EXPECT_EQ(stick.axis(2), 0.0);
  EXPECT_EQ(stick.axis(-1), 0.0);
  EXPECT_FALSE(stick.button(0));
  EXPECT_TRUE(stick.button(1));
  EXPECT_FALSE(stick.button(2));
  EXPECT_TRUE(stick.button(4));
  EXPECT_FALSE(stick.button(5));
  EXPECT_EQ(stick.pov(), 270);
  EXPECT_EQ(stick.pov(1), -1);
  EXPECT_EQ(stick.name(), "Xbox");
  EXPECT_THROW(ds.joystick(driver_station::kMaxJoysticks), std::out_of_range);

  // A Driver Station that is not attached reads as disabled, sticks and all.
  auto detached = TeleopStick();
  detached.fms.flags &= ~driver_station::kDsAttached;
  hw->set_driver_station(detached);
  sim.step(10ms);
  EXPECT_FALSE(ds.connected());
  EXPECT_FALSE(ds.enabled());
  EXPECT_FALSE(ds.joystick(1).connected());
  EXPECT_EQ(ds.joystick(1).axis(0), 0.0);

  // Nor does a silent one stay enabled.
  hw->set_driver_station(TeleopStick());
  sim.step(10ms);
  ASSERT_TRUE(ds.enabled());
  hw.reset();
  sim.step(150ms);  // past [robot] command_timeout_us
  EXPECT_FALSE(ds.connected());
  EXPECT_FALSE(ds.enabled());
  EXPECT_FALSE(ds.teleop());
  EXPECT_EQ(ds.data().fms.alliance, driver_station::Alliance::kBlue);  // kept
}

TEST(Devices, DriverStationDropsACorruptPacket) {
  auto sim = Sim::FromToml(kRobotToml);
  auto& pilot = sim.add<Pilot>("watcher");
  Packet corrupt{};
  corrupt.size = 2;  // too small to decode
  corrupt.data[0] = 0xFF;
  sim.publish(hardware::kDriverStationTopic, corrupt);
  sim.step(10ms);
  EXPECT_EQ(pilot.packets, 0);
  EXPECT_FALSE(pilot.ds.connected());
}

}  // namespace
}  // namespace talos
