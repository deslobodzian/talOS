#include "talOS/launcher/hardware_record.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/launcher/session.h"
#include "talOS/protocol/types.h"

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

// Each State reports one CAN bus; the record names it from the config the
// session ran, by its slot, with its read state and counters.
// REQ-OPS-20, REQ-OPS-15
TEST_F(HardwareRecordTest, StateNamesItsCanBusSlotFromTheRunConfig) {
  for (auto& motor : config_.motors) {
    if (motor.name == "flywheel") motor.bus = "Shooter";
  }
  names_ = DeviceNames::FromConfig(config_);
  talos::hardware::State state;
  state.config_id = talos::hardware::ConfigurationId(config_);
  state.health.can_bus_count = 2;
  state.health.can_bus_index = 1;
  state.health.can_bus = {
      talos::hardware::CanBusReadState::kFailed, 0.25F, 3, 4, 255, 7, 9};
  const auto health =
      Members(Members(HardwareRecordJson(HardwareRecord::kState, Encoded(state),
                                         &names_))
                  .at("health"));
  EXPECT_EQ(health.at("can_bus_count"), "2");
  const auto bus = Members(health.at("can_bus"));
  EXPECT_EQ(bus.at("index"), "1");
  EXPECT_EQ(bus.at("name"), "\"Shooter\"");
  EXPECT_EQ(bus.at("read_state"), "\"failed\"");
  EXPECT_EQ(bus.at("utilization_frac"), "0.25");
  EXPECT_EQ(bus.at("bus_off_count"), "3");
  EXPECT_EQ(bus.at("receive_error_count"), "255");
  EXPECT_EQ(bus.at("read_failure_count"), "9");
}

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
  state.build = {0x00ab0123456789cdULL,
                 talos::hardware::GatewayProgram::kRoboRio, true};
  const auto state_json = Members(
      HardwareRecordJson(HardwareRecord::kState, Encoded(state), &names_));
  EXPECT_EQ(state_json.at("type"), "\"state\"");
  EXPECT_EQ(state_json.at("enabled"), "true");
  // The RoboRIO build that drove the match, as its git revision reads.
  EXPECT_EQ(state_json.at("gateway_program"), "\"roborio\"");
  EXPECT_EQ(state_json.at("gateway_build_id"), "\"00ab0123456789cd\"");
  EXPECT_EQ(state_json.at("gateway_build_dirty"), "true");
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

// Guards: which sessions' hardware packets a build decodes. A manifest
// recording another protocol refuses them all, though every packet decodes
// under this build (the equal-length case), and the line names both
// versions; without a recorded version a packet the codec rejects refuses
// them; with this build's version recorded such a packet still refuses them
// and the line names no other version; this build's version with packets
// that decode, or no version with packets that decode, refuses none.
// REQ-OPS-51, REQ-OPS-15
TEST(HardwareProtocolCheck, RecordedVersionFirstThenTheCodecs) {
  constexpr std::uint16_t kBuild = talos::protocol::kProtocolVersion;
  constexpr std::uint16_t kOther = kBuild - 1;
  const auto decodes = Encoded(talos::hardware::State{});
  const auto rejected = std::span{decodes}.first(100);
  struct Case {
    const char* label;
    std::optional<std::uint16_t> recorded;
    bool reject;
    bool foreign;
    std::vector<std::uint16_t> named;
  };
  const std::vector<Case> cases{
      {"another version, packets decode",
       kOther,
       false,
       true,
       {kOther, kBuild}},
      {"no version, a packet rejected", std::nullopt, true, true, {kBuild}},
      {"this version, a packet rejected", kBuild, true, true, {kBuild}},
      {"this version, packets decode", kBuild, false, false, {}},
      {"no version, packets decode", std::nullopt, false, false, {}},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.label);
    talos::launcher::HardwareProtocolCheck check{c.recorded};
    check.Add(HardwareRecord::kState, "/hw/state", decodes);
    if (c.reject) check.Add(HardwareRecord::kState, "/hw/state", rejected);
    EXPECT_EQ(check.foreign(), c.foreign);
    const std::string line = check.Warning();
    EXPECT_EQ(line.empty(), !c.foreign) << line;
    for (const std::uint16_t version : {kOther, kBuild}) {
      const bool named =
          line.find("protocol " + std::to_string(version)) !=
              std::string::npos ||
          line.find("is " + std::to_string(version)) != std::string::npos;
      EXPECT_EQ(named, std::count(c.named.begin(), c.named.end(), version) > 0)
          << version << ": " << line;
    }
  }
}

// Guards: the version the launcher writes into a session's manifest.json is
// the one the log tools read back from that session's directory, and a
// manifest written without it (as before the field) reads as none.
// REQ-OPS-51, REQ-OPS-15
TEST(HardwareProtocolCheck, ManifestRecordsThisBuildsVersion) {
  const std::string dir = testing::TempDir() + "/recorded_protocol";
  std::filesystem::create_directories(dir);
  talos::launcher::SessionManifest manifest;
  manifest.config_path = "robot.toml";
  std::ofstream{dir + "/manifest.json"} << manifest.ToJson();
  EXPECT_EQ(talos::launcher::RecordedProtocolVersion({dir}),
            talos::protocol::kProtocolVersion);
  manifest.hardware_protocol_version.reset();
  std::ofstream{dir + "/manifest.json"} << manifest.ToJson();
  EXPECT_EQ(talos::launcher::RecordedProtocolVersion({dir}), std::nullopt);
}

// Guards: a .tlog named on its own (log_dump <session>/node.tlog) is read
// under the version its session's manifest records: the manifest beside it,
// when that manifest's session_id is the log's. A manifest of another
// session beside it, or none, records nothing for it.
// REQ-OPS-51
TEST(HardwareProtocolCheck, ALogNamedAloneTakesItsSessionsManifest) {
  const std::string dir = testing::TempDir() + "/log_named_alone";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const std::string log = dir + "/node.tlog";
  {
    talos::event::log::LogWriter writer{
        log, "node", talos::event::log::LogWriterOptions{4096, 1, false}, 42};
    writer.start({}, talos::event::MonotonicTime::from_nanos(1'000'000'000));
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  const std::uint16_t other = talos::protocol::kProtocolVersion - 1;
  EXPECT_EQ(talos::launcher::RecordedProtocolVersion({log}), std::nullopt);
  talos::launcher::SessionManifest manifest;
  manifest.hardware_protocol_version = other;
  for (const std::uint64_t session_id : {42, 43}) {
    SCOPED_TRACE(session_id);
    manifest.session_id = session_id;
    std::ofstream{dir + "/manifest.json"} << manifest.ToJson();
    EXPECT_EQ(talos::launcher::RecordedProtocolVersion({log}),
              session_id == 42 ? std::optional{other} : std::nullopt);
  }
  std::filesystem::remove_all(dir);
}

}  // namespace
