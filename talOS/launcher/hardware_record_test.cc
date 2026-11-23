#include "talOS/launcher/hardware_record.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "flatbuffers/reflection.h"
#include "flatbuffers/reflection_generated.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/manifest.h"
#include "talOS/events/time.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/launcher/session.h"
#include "talOS/protocol/types.h"
#include "talOS/schema/identity.h"

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

[subsystems.shooter.pwm_outputs.hood]
channel = 0
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

// The registration of /hw/status as `kind` with this build's BridgeStatus
// schema.
talos::event::Registration StatusSource(std::uint16_t id,
                                        talos::event::SourceKind kind) {
  talos::event::Registration source;
  source.id = id;
  source.kind = kind;
  source.name = std::string{bridge::topics::kStatus};
  source.message_bytes = sizeof(bridge::BridgeStatus);
  source.schema = talos::schema::Of<bridge::BridgeStatus>();
  return source;
}

// A node's log at `path` of session `session_id` that reads /hw/status, as
// every node's devices layer does, with a reader's copy of a status naming
// protocol 1: not the bridge's own record.
void WriteNodeLog(const std::string& path, std::uint64_t session_id) {
  talos::event::log::LogWriter writer{
      path, "drivetrain", talos::event::log::LogWriterOptions{4096, 1, false},
      session_id};
  writer.start({StatusSource(0, talos::event::SourceKind::WATCHER)},
               talos::event::MonotonicTime::from_nanos(1'000'000'000));
  bridge::BridgeStatus status{};
  status.mutate_protocol_version_id(1);
  talos::event::Context ctx{};
  ctx.kind = talos::event::EventKind::MESSAGE;
  ctx.source_id = 0;
  ctx.dispatch_index = 1;
  ctx.event_time = talos::event::MonotonicTime::from_nanos(1'000'000'000);
  ctx.now = ctx.event_time;
  ctx.publish_time = ctx.event_time;
  writer.dispatch(ctx, std::as_bytes(std::span{&status, 1}));
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();
}

// Builds a synthetic verified reflection.Schema `.bfbs` for a struct named
// `bridge.BridgeStatus` of `bytesize` bytes: either carrying an integer
// `protocol_version_id` at `version_offset` (modified layout) or lacking the
// field altogether (pre-field layout).
std::string SyntheticBridgeStatusBfbs(bool with_protocol_version,
                                      std::uint16_t version_offset,
                                      std::int32_t bytesize) {
  flatbuffers::FlatBufferBuilder builder;
  const auto obj_name = builder.CreateString("bridge.BridgeStatus");
  std::vector<flatbuffers::Offset<reflection::Field>> fields_vec;
  {
    const auto name = builder.CreateString("states_received_count");
    reflection::TypeBuilder tb(builder);
    tb.add_base_type(reflection::BaseType::ULong);
    const auto type = tb.Finish();
    reflection::FieldBuilder fb(builder);
    fb.add_name(name);
    fb.add_type(type);
    fb.add_id(0);
    fb.add_offset(0);
    fields_vec.push_back(fb.Finish());
  }
  if (with_protocol_version) {
    const auto name = builder.CreateString("protocol_version_id");
    reflection::TypeBuilder tb(builder);
    tb.add_base_type(reflection::BaseType::UShort);
    const auto type = tb.Finish();
    reflection::FieldBuilder fb(builder);
    fb.add_name(name);
    fb.add_type(type);
    fb.add_id(1);
    fb.add_offset(version_offset);
    fields_vec.push_back(fb.Finish());
  }
  const auto fields = builder.CreateVectorOfSortedTables(&fields_vec);
  reflection::ObjectBuilder ob(builder);
  ob.add_name(obj_name);
  ob.add_fields(fields);
  ob.add_is_struct(true);
  ob.add_bytesize(bytesize);
  const auto obj = ob.Finish();
  std::vector<flatbuffers::Offset<reflection::Object>> objects_vec{obj};
  const auto objects = builder.CreateVectorOfSortedTables(&objects_vec);
  const auto enums = builder.CreateVector(
      std::vector<flatbuffers::Offset<reflection::Enum>>{});
  reflection::SchemaBuilder sb(builder);
  sb.add_objects(objects);
  sb.add_enums(enums);
  reflection::FinishSchemaBuffer(builder, sb.Finish());
  return std::string{reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize()};
}

// The bridge's log at `path` of session `session_id`: a State, then, `late`
// after it, two /hw/status SENDs carrying `status_payload` under
// `status_source`.
void WriteBridgeLogWithStatus(const std::string& path, std::uint64_t session_id,
                              const talos::event::Registration& status_source,
                              std::span<const std::byte> status_payload,
                              std::chrono::nanoseconds late) {
  using Packet = talos::Descriptor<talos::hardware::Packet>;
  talos::event::Registration state;
  state.id = 0;
  state.kind = talos::event::SourceKind::SENDER;
  state.name = std::string{talos::hardware::kStateTopic};
  state.message_bytes = sizeof(talos::hardware::Packet);
  state.schema = {std::string{Packet::kName}, Packet::kHash, {}};
  talos::event::log::LogWriter writer{
      path, "hardware_node",
      talos::event::log::LogWriterOptions{1 << 14, 1, false}, session_id};
  writer.start({state, status_source},
               talos::event::MonotonicTime::from_nanos(1'000'000'000));
  talos::event::Context ctx{};
  ctx.kind = talos::event::EventKind::SEND;
  ctx.event_time = talos::event::MonotonicTime::from_nanos(1'000'000'000);
  ctx.now = ctx.event_time;
  ctx.publish_time = ctx.event_time;
  writer.send(ctx, 0, Encoded(talos::hardware::State{}), 0, 1);
  ctx.event_time = talos::event::MonotonicTime::from_nanos(
      ctx.event_time.nanos() + late.count());
  ctx.now = ctx.event_time;
  ctx.publish_time = ctx.event_time;
  for (const std::uint64_t sequence : {1, 2}) {
    writer.send(ctx, 1, status_payload, 0, sequence);
  }
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();
}

// The bridge's log at `path` of session `session_id`: a State, then, `late`
// after it, two /hw/status SENDs whose protocol_version_id is `version`, the
// topic registered as a BridgeStatus layout of `schema_hash`.
void WriteBridgeLog(const std::string& path, std::uint64_t session_id,
                    std::uint16_t version, std::uint64_t schema_hash,
                    std::chrono::nanoseconds late) {
  talos::event::Registration status_source =
      StatusSource(1, talos::event::SourceKind::SENDER);
  status_source.schema.hash = schema_hash;
  bridge::BridgeStatus status{};
  status.mutate_protocol_version_id(version);
  WriteBridgeLogWithStatus(path, session_id, status_source,
                           std::as_bytes(std::span{&status, 1}), late);
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
// and limit bits must be readable by device name, not by byte offset, and a
// PWM request says whether it pulsed (a pulse of 0 centres a servo) or sent
// no pulse.
// REQ-OPS-15
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
  const std::uint16_t hood = config_.pwm_outputs.at(0).id;
  command.pwm_output_count = 1;
  command.pwm_outputs[0] = {hood, talos::hardware::PwmMode::kPulse, 0.0};
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
  // REQ-SAFE-43: a torque-current demand reads as amperes in its own mode.
  auto torque = command;
  torque.motors[1] = {drive_, talos::hardware::Mode::kTorqueCurrent, 0, -2.5,
                      0.0};
  motor = Find(Members(HardwareRecordJson(HardwareRecord::kCommand,
                                          Encoded(torque), &names_))
                   .at("motors"),
               "name", "\"drive\"");
  EXPECT_EQ(motor["mode"], "\"torque_current\"");
  EXPECT_EQ(motor["demand"], "-2.5");
  auto pwm = Find(command_json.at("pwm_outputs"), "name", "\"hood\"");
  EXPECT_EQ(pwm["mode"], "\"pulse\"");
  EXPECT_EQ(pwm["output"], "0");
  command.pwm_outputs[0] = {.id = hood};
  pwm = Find(Members(HardwareRecordJson(HardwareRecord::kCommand,
                                        Encoded(command), &names_))
                 .at("pwm_outputs"),
             "name", "\"hood\"");
  EXPECT_EQ(pwm["mode"], "\"off\"");

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

// Guards: which sessions' hardware packets a build decodes. A recorded
// version other than this build's refuses them all, though every packet
// decodes under this build (the equal-length case), and the line names both
// versions; so does a bridge that recorded two; the bridge's version decides
// over the manifest's either way; without a recorded version a packet the
// codec rejects refuses them; with this build's version recorded such a
// packet still refuses them and the line names no other version; this
// build's version with packets that decode, or no version with packets that
// decode, refuses none.
// REQ-OPS-51, REQ-OPS-15
TEST(HardwareProtocolCheck, RecordedVersionFirstThenTheCodecs) {
  constexpr std::uint16_t kBuild = talos::protocol::kProtocolVersion;
  constexpr std::uint16_t kOther = kBuild - 1;
  const auto decodes = Encoded(talos::hardware::State{});
  const auto rejected = std::span{decodes}.first(100);
  struct Case {
    const char* label;
    talos::launcher::RecordedProtocol recorded;
    bool reject;
    bool foreign;
    std::vector<std::uint16_t> named;
  };
  const std::vector<Case> cases{
      {"another version, packets decode",
       {kOther, {}},
       false,
       true,
       {kOther, kBuild}},
      {"the bridge's another, the manifest's this",
       {kBuild, {kOther}},
       false,
       true,
       {kOther, kBuild}},
      {"the bridge's two versions",
       {std::nullopt, {kBuild, kOther}},
       false,
       true,
       {kOther, kBuild}},
      {"the bridge's this, the manifest's another",
       {kOther, {kBuild}},
       false,
       false,
       {}},
      {"no version, a packet rejected", {}, true, true, {kBuild}},
      {"this version, a packet rejected", {kBuild, {}}, true, true, {kBuild}},
      {"this version, packets decode", {kBuild, {}}, false, false, {}},
      {"no version, packets decode", {}, false, false, {}},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.label);
    talos::launcher::HardwareProtocolCheck check{c.recorded};
    check.Add(HardwareRecord::kState, "/hw/state", false, decodes);
    if (c.reject)
      check.Add(HardwareRecord::kState, "/hw/state", false, rejected);
    EXPECT_EQ(check.foreign(), c.foreign);
    const std::string line = check.Warning();
    EXPECT_EQ(line.empty(), !c.foreign) << line;
    // The whole numbers the line names.
    std::vector<std::string> numbers;
    for (std::size_t at = line.find_first_of("0123456789");
         at != std::string::npos;) {
      const std::size_t end = line.find_first_not_of("0123456789", at);
      numbers.push_back(line.substr(at, end - at));
      at = end == std::string::npos ? end
                                    : line.find_first_of("0123456789", end);
    }
    for (const std::uint16_t version : {kOther, kBuild}) {
      const bool named = std::count(numbers.begin(), numbers.end(),
                                    std::to_string(version)) > 0;
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
  EXPECT_EQ(talos::launcher::RecordedProtocolVersion({dir}).manifest,
            talos::protocol::kProtocolVersion);
  manifest.hardware_protocol_version.reset();
  std::ofstream{dir + "/manifest.json"} << manifest.ToJson();
  EXPECT_EQ(talos::launcher::RecordedProtocolVersion({dir}).manifest,
            std::nullopt);
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
  EXPECT_EQ(talos::launcher::RecordedProtocolVersion({log}).version(),
            std::nullopt);
  talos::launcher::SessionManifest manifest;
  manifest.hardware_protocol_version = other;
  for (const std::uint64_t session_id : {42, 43}) {
    SCOPED_TRACE(session_id);
    manifest.session_id = session_id;
    std::ofstream{dir + "/manifest.json"} << manifest.ToJson();
    EXPECT_EQ(talos::launcher::RecordedProtocolVersion({log}).version(),
              session_id == 42 ? std::optional{other} : std::nullopt);
  }
  std::filesystem::remove_all(dir);
}

// Guards: the manifest's version is used only when no log records a
// /hw/status SEND whose .bfbs schema carries an integer `protocol_version_id`
// within its first second: none at all, a node's copy, one whose .bfbs lacks
// `protocol_version_id`, or one past the first second (the search is bounded);
// a modified-layout BridgeStatus .bfbs carrying `protocol_version_id` at a
// different offset/size and schema hash IS read by field name; a bridge
// version other than this build's refuses the session though the manifest's is
// this build's; bridge runs of two versions in one session refuse it. A log
// named alone is read with its session's bridge log. (A real bridge's log read
// under its version over the manifest's: //talOS/bridge:node_test
// ASessionIsReadUnderTheProtocolItsBridgeLogged.)
// REQ-OPS-51
TEST(HardwareProtocolCheck, TheBridgesRecordedVersionComesFirst) {
  constexpr std::uint16_t kBuild = talos::protocol::kProtocolVersion;
  constexpr std::uint16_t kOther = kBuild - 1;
  constexpr std::uint64_t kHash =
      talos::Descriptor<bridge::BridgeStatus>::kHash;
  using std::chrono::milliseconds;
  enum class SchemaKind { kCurrent, kModifiedWithField, kWithoutField };
  struct Bridge {
    const char* file;
    std::uint16_t version;
    std::uint64_t schema_hash;
    // The status SENDs' time after the log's first record.
    milliseconds late{20};
    SchemaKind schema_kind{SchemaKind::kCurrent};
  };
  struct Case {
    const char* label;
    std::uint16_t manifest;
    std::vector<Bridge> bridge;
    std::optional<std::uint16_t> version;
    bool refused;
    // RecordedProtocol::bridge, ascending.
    std::vector<std::uint16_t> bridge_versions;
    bool disagreement;
  };
  const std::vector<Case> cases{
      {"the launcher's this, the bridge's another",
       kBuild,
       {{"hardware_node.tlog", kOther, kHash}},
       kOther,
       true,
       {kOther},
       true},
      {"modified-layout BridgeStatus bfbs carrying protocol_version_id",
       kBuild,
       {{"hardware_node.tlog", kOther, kHash ^ 1, milliseconds{20},
         SchemaKind::kModifiedWithField}},
       kOther,
       true,
       {kOther},
       true},
      {"no /hw/status", kOther, {}, kOther, true, {}, false},
      {"a BridgeStatus bfbs lacking protocol_version_id",
       kOther,
       {{"hardware_node.tlog", kBuild, kHash ^ 2, milliseconds{20},
         SchemaKind::kWithoutField}},
       kOther,
       true,
       {},
       false},
      {"a /hw/status past the first second",
       kOther,
       {{"hardware_node.tlog", kBuild, kHash, milliseconds{1001}}},
       kOther,
       true,
       {},
       false},
      {"bridge runs of two versions",
       kBuild,
       {{"hardware_node.tlog", kBuild, kHash},
        {"hardware_node.restart1.tlog", kOther, kHash}},
       std::nullopt,
       true,
       {kOther, kBuild},
       false},
  };
  const std::string dir = testing::TempDir() + "/bridge_protocol";
  for (const auto& c : cases) {
    SCOPED_TRACE(c.label);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    talos::launcher::SessionManifest manifest;
    manifest.session_id = 42;
    manifest.hardware_protocol_version = c.manifest;
    std::ofstream{dir + "/manifest.json"} << manifest.ToJson();
    WriteNodeLog(dir + "/drivetrain.tlog", 42);
    for (const auto& bridge : c.bridge) {
      const std::string log_path = dir + "/" + bridge.file;
      if (bridge.schema_kind == SchemaKind::kCurrent) {
        WriteBridgeLog(log_path, 42, bridge.version, bridge.schema_hash,
                       bridge.late);
      } else if (bridge.schema_kind == SchemaKind::kModifiedWithField) {
        constexpr std::size_t kModifiedBytes = 16;
        constexpr std::uint16_t kVersionOffset = 10;
        talos::event::Registration status_source;
        status_source.id = 1;
        status_source.kind = talos::event::SourceKind::SENDER;
        status_source.name = std::string{bridge::topics::kStatus};
        status_source.message_bytes = kModifiedBytes;
        status_source.schema = {
            "bridge.BridgeStatus", bridge.schema_hash,
            SyntheticBridgeStatusBfbs(/*with_protocol_version=*/true,
                                      kVersionOffset, kModifiedBytes)};
        std::array<std::byte, kModifiedBytes> payload{};
        std::memcpy(payload.data() + kVersionOffset, &bridge.version,
                    sizeof(bridge.version));
        WriteBridgeLogWithStatus(log_path, 42, status_source, payload,
                                 bridge.late);
      } else {
        constexpr std::size_t kOldBytes = 8;
        talos::event::Registration status_source;
        status_source.id = 1;
        status_source.kind = talos::event::SourceKind::SENDER;
        status_source.name = std::string{bridge::topics::kStatus};
        status_source.message_bytes = kOldBytes;
        status_source.schema = {
            "bridge.BridgeStatus", bridge.schema_hash,
            SyntheticBridgeStatusBfbs(/*with_protocol_version=*/false, 0,
                                      kOldBytes)};
        std::array<std::byte, kOldBytes> payload{};
        WriteBridgeLogWithStatus(log_path, 42, status_source, payload,
                                 bridge.late);
      }
    }
    for (const auto& input : {dir, dir + "/drivetrain.tlog"}) {
      SCOPED_TRACE(input);
      const auto recorded = talos::launcher::RecordedProtocolVersion({input});
      std::vector<std::uint16_t> bridge_versions = recorded.bridge;
      std::sort(bridge_versions.begin(), bridge_versions.end());
      EXPECT_EQ(bridge_versions, c.bridge_versions);
      EXPECT_EQ(recorded.version(), c.version);
      EXPECT_EQ(recorded.refused(), c.refused);
      EXPECT_EQ(recorded.Refusal().empty(), !c.refused);
      const std::string disagreement = recorded.Disagreement();
      EXPECT_EQ(disagreement.empty(), !c.disagreement);
      if (c.disagreement) {
        EXPECT_NE(disagreement.find(std::to_string(kBuild)), std::string::npos)
            << disagreement;
        EXPECT_NE(disagreement.find(std::to_string(kOther)), std::string::npos)
            << disagreement;
      }
    }
  }
  std::filesystem::remove_all(dir);
}

}  // namespace
