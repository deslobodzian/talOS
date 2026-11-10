#include "talOS/launcher/hardware_record.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"

namespace {

using talos::launcher::DeviceNames;
using talos::launcher::HardwareRecord;
using talos::launcher::HardwareRecordJson;
using talos::launcher::HardwareRecordOf;

constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.drivetrain]
period_us = 5000

[subsystems.drivetrain.motors.drive]
type = "TalonFX"
can_id = 1

[subsystems.shooter]
period_us = 10000

[subsystems.shooter.motors.flywheel]
type = "TalonFX"
can_id = 2
)";

std::vector<std::byte> Record(const talos::hardware::Packet& packet) {
  const auto bytes = std::as_bytes(std::span{&packet, 1});
  return {bytes.begin(), bytes.end()};
}

template <typename Message>
std::vector<std::byte> Encoded(const Message& message) {
  talos::hardware::Packet packet{};
  packet.size =
      static_cast<std::uint32_t>(talos::hardware::Encode(message, packet.data));
  return Record(packet);
}

// The members of one JSON object, raw value text by key, at its top level
// only. Enough for this decoder's output: no whitespace, no braces inside
// strings.
std::map<std::string, std::string> Members(std::string_view object) {
  std::map<std::string, std::string> members;
  int depth = 0;
  std::size_t value_start = 0;
  std::string key;
  bool in_string = false;
  std::size_t string_start = 0;
  for (std::size_t i = 0; i < object.size(); ++i) {
    const char c = object[i];
    if (in_string) {
      if (c == '\\') {
        ++i;
      } else if (c == '"') {
        in_string = false;
        if (depth == 1 && i + 1 < object.size() && object[i + 1] == ':') {
          key = std::string{object.substr(string_start, i - string_start)};
          value_start = i + 2;
        }
      }
      continue;
    }
    if (c == '"') {
      in_string = true;
      string_start = i + 1;
    } else if (c == '{' || c == '[') {
      ++depth;
    } else if (c == '}' || c == ']' || c == ',') {
      if (depth == 1 && !key.empty()) {
        members[key] = std::string{object.substr(value_start, i - value_start)};
        key.clear();
      }
      if (c != ',') --depth;
    }
  }
  return members;
}

// The top-level elements of a JSON array's raw text.
std::vector<std::string> Elements(std::string_view array) {
  std::vector<std::string> elements;
  int depth = 0;
  bool in_string = false;
  std::size_t start = 1;
  for (std::size_t i = 0; i < array.size(); ++i) {
    const char c = array[i];
    if (in_string) {
      if (c == '\\') ++i;
      if (c == '"') in_string = false;
      continue;
    }
    if (c == '"') {
      in_string = true;
    } else if (c == '{' || c == '[') {
      ++depth;
    } else if (c == '}' || c == ']' || c == ',') {
      if (depth == 1 && i > start) {
        elements.emplace_back(array.substr(start, i - start));
      }
      if (depth == 1) start = i + 1;
      if (c != ',') --depth;
    }
  }
  return elements;
}

// The element of `array` whose `key` member is `value`.
std::map<std::string, std::string> Find(std::string_view array,
                                        const std::string& key,
                                        const std::string& value) {
  for (const auto& element : Elements(array)) {
    auto members = Members(element);
    if (members[key] == value) return members;
  }
  return {};
}

class HardwareRecordTest : public testing::Test {
 protected:
  void SetUp() override {
    config_ = talos::config::ParseRobotConfigString(kToml).hardware;
    names_ = DeviceNames::FromConfig(config_);
    for (const auto& motor : config_.motors) {
      (motor.name == "flywheel" ? flywheel_ : drive_) = motor.id;
    }
  }

  talos::hardware::Config config_;
  DeviceNames names_;
  std::uint16_t flywheel_{};
  std::uint16_t drive_{};
};

// After a match the per-motor demand and the applied output, current, faults
// and limit bits must be readable by device name, not by byte offset.
TEST_F(HardwareRecordTest, StateAndCommandNameDevicesOfTheRunConfig) {
  ASSERT_EQ(HardwareRecordOf("/hw/state", "talos.hardware.Packet"),
            HardwareRecord::kState);
  ASSERT_EQ(HardwareRecordOf("/hw/request/shooter", "talos.hardware.Packet"),
            HardwareRecord::kCommand);

  talos::hardware::State state;
  state.config_id = talos::hardware::ConfigurationId(config_);
  state.flags = talos::hardware::kConfigured | talos::hardware::kEnabled;
  state.motor_count = 1;
  auto& sample = state.motors[0];
  sample.id = flywheel_;
  sample.valid = true;
  sample.velocity_rps = 42.5;
  sample.voltage = 11.25;
  sample.stator_current_a = 30.5;
  sample.fault_flags = talos::hardware::kMotorForwardHardLimitFlag;
  const auto state_json = Members(
      HardwareRecordJson(HardwareRecord::kState, Encoded(state), &names_));
  EXPECT_EQ(state_json.at("type"), "\"state\"");
  EXPECT_EQ(state_json.at("enabled"), "true");
  auto motor = Find(state_json.at("motors"), "name", "\"flywheel\"");
  EXPECT_EQ(motor["valid"], "true");
  EXPECT_EQ(motor["velocity_rps"], "42.5");
  EXPECT_EQ(motor["voltage"], "11.25");
  EXPECT_EQ(motor["stator_current_a"], "30.5");
  EXPECT_EQ(motor["faulted"], "false");
  EXPECT_EQ(motor["forward_limit"], "true");
  EXPECT_EQ(motor["reverse_limit"], "false");

  // The bridge commissions the config in memory under --sim.
  auto commissioned = config_;
  commissioned.commissioned = true;
  talos::hardware::Command command;
  command.config_id = talos::hardware::ConfigurationId(commissioned);
  command.count = 2;
  command.motors[0] = {flywheel_, talos::hardware::Mode::kVelocity, 1, 40.0,
                       0.5};
  command.motors[1] = {drive_, talos::hardware::Mode::kNeutral, 0, 0.0, 0.0};
  const auto command_json = Members(
      HardwareRecordJson(HardwareRecord::kCommand, Encoded(command), &names_));
  motor = Find(command_json.at("motors"), "name", "\"flywheel\"");
  EXPECT_EQ(motor["id"], std::to_string(flywheel_));
  EXPECT_EQ(motor["mode"], "\"velocity\"");
  EXPECT_EQ(motor["slot"], "1");
  EXPECT_EQ(motor["demand"], "40");
  EXPECT_EQ(motor["feedforward_v"], "0.5");
  EXPECT_EQ(Find(command_json.at("motors"), "name", "\"drive\"")["mode"],
            "\"neutral\"");

  // A packet of another config (edited since the match) keeps ids only.
  command.config_id += 1;
  const auto other = Members(
      HardwareRecordJson(HardwareRecord::kCommand, Encoded(command), &names_));
  motor = Find(other.at("motors"), "id", std::to_string(flywheel_));
  EXPECT_EQ(motor.count("name"), 0U);
  EXPECT_EQ(motor["mode"], "\"velocity\"");
}

// Driver Station strings are cut at a fixed length, possibly inside a UTF-8
// character; the row must stay valid UTF-8 (JSONL readers decode it as such).
TEST_F(HardwareRecordTest, DriverStationShowsModeJoysticksAndValidText) {
  namespace ds = talos::driver_station;
  ds::DriverStationData data;
  data.mode_since_us = 7;
  data.fms.flags = ds::kDsAttached | ds::kEnabled | ds::kTeleop;
  data.fms.alliance = ds::Alliance::kBlue;
  data.fms.station = 2;
  std::string_view{"Caf\xC3\xA9"}.copy(data.fms.event_name, 5);
  auto& stick = data.joysticks[1];
  stick.connected = true;
  stick.axis_count = 2;
  stick.axes = {0.5F, -1.0F};
  stick.button_count = 4;
  stick.buttons = 0b0101;
  stick.pov_count = 1;
  stick.povs = {90};
  // 30 ASCII bytes and the first byte of a two-byte character.
  const std::string cut = std::string(30, 'a') + "\xC3";
  cut.copy(stick.name, cut.size());
  talos::hardware::Packet packet{};
  packet.size = static_cast<std::uint32_t>(ds::Encode(data, packet.data));
  const auto json = Members(HardwareRecordJson(HardwareRecord::kDriverStation,
                                               Record(packet), nullptr));
  EXPECT_EQ(json.at("mode"), "\"teleop\"");
  EXPECT_EQ(json.at("enabled"), "true");
  EXPECT_EQ(json.at("estop"), "false");
  EXPECT_EQ(json.at("alliance"), "\"blue\"");
  EXPECT_EQ(json.at("station"), "2");
  EXPECT_EQ(json.at("event_name"), "\"Caf\xC3\xA9\"");
  const auto sticks = Elements(json.at("joysticks"));
  ASSERT_EQ(sticks.size(), 1U);
  auto joystick = Members(sticks[0]);
  EXPECT_EQ(joystick["port"], "1");
  EXPECT_EQ(joystick["axes"], "[0.5,-1]");
  EXPECT_EQ(joystick["buttons"], "5");
  EXPECT_EQ(joystick["povs"], "[90]");
  EXPECT_EQ(joystick["name"], "\"" + std::string(30, 'a') + "\\u00c3\"");
}

// A truncated or corrupt record must not stop an export: it reads as an
// error member, and the record's payload_hex still carries its bytes.
TEST_F(HardwareRecordTest, MalformedPayloadYieldsAnError) {
  const auto record = Encoded(talos::hardware::State{});
  talos::hardware::Packet oversize{};
  oversize.size = 0xFFFFFFFF;
  const std::vector<std::pair<HardwareRecord, std::string>> cases{
      {HardwareRecord::kState,
       HardwareRecordJson(HardwareRecord::kState, std::span{record}.first(100),
                          &names_)},
      {HardwareRecord::kCommand,
       HardwareRecordJson(HardwareRecord::kCommand, Record(oversize), &names_)},
      // A State's bytes are not a Command or a Driver Station packet.
      {HardwareRecord::kCommand,
       HardwareRecordJson(HardwareRecord::kCommand, record, &names_)},
      {HardwareRecord::kDriverStation,
       HardwareRecordJson(HardwareRecord::kDriverStation, record, &names_)},
  };
  for (const auto& [kind, text] : cases) {
    const auto json = Members(text);
    EXPECT_EQ(json.count("error"), 1U) << text;
    EXPECT_EQ(json.count("motors") + json.count("joysticks"), 0U) << text;
  }
}

}  // namespace
