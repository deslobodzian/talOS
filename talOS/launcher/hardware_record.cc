#include "talOS/launcher/hardware_record.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <system_error>
#include <utility>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/context.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/manifest.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/protocol/types.h"
#include "talOS/schema/identity.h"

namespace talos::launcher {
namespace {

namespace ds = driver_station;

// The length of the well-formed UTF-8 sequence (RFC 3629) that `text`
// starts with a non-ASCII byte of, or 0 if it does not start one.
std::size_t Utf8Length(std::string_view text) {
  const auto at = [&](std::size_t i) {
    return static_cast<unsigned char>(text[i]);
  };
  const unsigned char lead = at(0);
  std::size_t length = 0;
  unsigned char low = 0x80;
  unsigned char high = 0xBF;
  if (lead >= 0xC2 && lead <= 0xDF) {
    length = 2;
  } else if (lead >= 0xE0 && lead <= 0xEF) {
    length = 3;
    if (lead == 0xE0) low = 0xA0;   // Overlong.
    if (lead == 0xED) high = 0x9F;  // Surrogates.
  } else if (lead >= 0xF0 && lead <= 0xF4) {
    length = 4;
    if (lead == 0xF0) low = 0x90;   // Overlong.
    if (lead == 0xF4) high = 0x8F;  // Past U+10FFFF.
  } else {
    return 0;
  }
  if (text.size() < length) return 0;
  if (at(1) < low || at(1) > high) return 0;
  for (std::size_t i = 2; i < length; ++i) {
    if (at(i) < 0x80 || at(i) > 0xBF) return 0;
  }
  return length;
}

// Appends `"key":` members to an object under construction; the caller adds
// the braces and brackets.
class Json {
 public:
  explicit Json(std::string& out) : out_(out) {}

  Json& Key(std::string_view key) {
    if (!out_.empty() && out_.back() != '{' && out_.back() != '[') out_ += ',';
    Text(key);
    out_ += ':';
    return *this;
  }
  Json& Item() {
    if (!out_.empty() && out_.back() != '{' && out_.back() != '[') out_ += ',';
    return *this;
  }
  // A JSON string that stays valid UTF-8 whatever the bytes: Driver Station
  // names are cut at a fixed length, possibly inside a character, so a byte
  // that does not begin a complete UTF-8 sequence is written as \u00XX.
  Json& Text(std::string_view value) {
    constexpr char kHex[] = "0123456789abcdef";
    const auto escape = [&](unsigned char c) {
      out_ += "\\u00";
      out_ += kHex[c >> 4];
      out_ += kHex[c & 15];
    };
    out_ += '"';
    for (std::size_t i = 0; i < value.size();) {
      const auto c = static_cast<unsigned char>(value[i]);
      if (c < 0x80) {
        if (c == '"' || c == '\\') {
          out_ += '\\';
          out_ += static_cast<char>(c);
        } else if (c < 0x20) {
          escape(c);
        } else {
          out_ += static_cast<char>(c);
        }
        ++i;
        continue;
      }
      const std::size_t length = Utf8Length(value.substr(i));
      if (length == 0) {
        escape(c);
        ++i;
      } else {
        out_ += value.substr(i, length);
        i += length;
      }
    }
    out_ += '"';
    return *this;
  }
  Json& Bool(bool value) {
    out_ += value ? "true" : "false";
    return *this;
  }
  Json& Int(std::int64_t value) {
    out_ += std::to_string(value);
    return *this;
  }
  Json& UInt(std::uint64_t value) {
    out_ += std::to_string(value);
    return *this;
  }
  // A 64-bit id as a decimal string: a JSON number is a double.
  Json& Id(std::uint64_t value) {
    out_ += '"';
    out_ += std::to_string(value);
    out_ += '"';
    return *this;
  }
  Json& Number(double value, int digits = 17) {
    if (!std::isfinite(value)) {
      out_ += "null";
      return *this;
    }
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.*g", digits, value);
    out_ += buffer;
    return *this;
  }
  Json& Raw(std::string_view text) {
    out_ += text;
    return *this;
  }

 private:
  std::string& out_;
};

void Name(Json& json, const std::map<std::uint16_t, std::string>* names,
          std::uint16_t id) {
  json.Key("id").Int(id);
  if (names == nullptr) return;
  const auto it = names->find(id);
  if (it != names->end()) json.Key("name").Text(it->second);
}

// One array member of `count` samples, each written by `item`.
template <typename Array, typename Item>
void Samples(Json& json, std::string_view key, const Array& samples,
             std::size_t count, Item item) {
  json.Key(key).Raw("[");
  for (std::size_t i = 0; i < count && i < samples.size(); ++i) {
    json.Item().Raw("{");
    item(samples[i]);
    json.Raw("}");
  }
  json.Raw("]");
}

void StateJson(Json& json, const hardware::State& s, const DeviceNames* n) {
  json.Key("config_id").Id(s.config_id);
  json.Key("boot_id").Id(s.boot_id);
  // The build that sent it: the program, the first 16 hex digits of its git
  // commit (null when unknown) and whether that checkout was dirty.
  json.Key("gateway_program")
      .Text(hardware::GatewayProgramName(s.build.program));
  if (s.build.revision == 0) {
    json.Key("gateway_build_id").Raw("null");
  } else {
    char revision[17];
    std::snprintf(revision, sizeof(revision), "%016llx",
                  static_cast<unsigned long long>(s.build.revision));
    json.Key("gateway_build_id").Text(revision);
  }
  json.Key("gateway_build_dirty").Bool(s.build.dirty);
  json.Key("epoch").UInt(s.epoch);
  json.Key("sample_time_us").UInt(s.sample_time_us);
  json.Key("sequence").UInt(s.sequence);
  json.Key("last_command_sequence").UInt(s.last_command_sequence);
  json.Key("configured").Bool((s.flags & hardware::kConfigured) != 0);
  json.Key("enabled").Bool((s.flags & hardware::kEnabled) != 0);
  json.Key("command_active").Bool((s.flags & hardware::kCommandActive) != 0);
  json.Key("hardware_fault").Bool((s.flags & hardware::kHardwareFault) != 0);
  Samples(
      json, "motors", s.motors, s.motor_count,
      [&](const hardware::MotorSample& m) {
        Name(json, n ? &n->motors : nullptr, m.id);
        json.Key("valid").Bool(m.valid);
        json.Key("position_rot").Number(m.position_rot);
        json.Key("velocity_rps").Number(m.velocity_rps);
        json.Key("voltage").Number(m.voltage);
        json.Key("stator_current_a").Number(m.stator_current_a);
        json.Key("supply_current_a").Number(m.supply_current_a);
        json.Key("temperature_c").Number(m.temperature_c);
        json.Key("position_age_us").Int(m.position_age_us);
        json.Key("velocity_age_us").Int(m.velocity_age_us);
        json.Key("fault_flags").Int(m.fault_flags);
        json.Key("faulted").Bool(hardware::MotorFaulted(m.fault_flags));
        json.Key("forward_limit")
            .Bool((m.fault_flags & hardware::kMotorForwardHardLimitFlag) != 0);
        json.Key("reverse_limit")
            .Bool((m.fault_flags & hardware::kMotorReverseHardLimitFlag) != 0);
      });
  Samples(json, "sensors", s.sensors, s.sensor_count,
          [&](const hardware::SensorSample& x) {
            Name(json, n ? &n->sensors : nullptr, x.id);
            json.Key("valid").Bool(x.valid);
            json.Key("position_rot").Number(x.position_rot);
            json.Key("velocity_rps").Number(x.velocity_rps);
            json.Key("position_age_us").Int(x.position_age_us);
            json.Key("velocity_age_us").Int(x.velocity_age_us);
          });
  Samples(json, "digital_inputs", s.digital_inputs, s.digital_input_count,
          [&](const hardware::DigitalInputSample& x) {
            Name(json, n ? &n->digital_inputs : nullptr, x.id);
            json.Key("valid").Bool(x.valid);
            json.Key("value").Bool(x.value);
          });
  Samples(json, "digital_outputs", s.digital_outputs, s.digital_output_count,
          [&](const hardware::DigitalOutputSample& x) {
            Name(json, n ? &n->digital_outputs : nullptr, x.id);
            json.Key("valid").Bool(x.valid);
            json.Key("value").Bool(x.value);
          });
  Samples(json, "analog_inputs", s.analog_inputs, s.analog_input_count,
          [&](const hardware::AnalogInputSample& x) {
            Name(json, n ? &n->analog_inputs : nullptr, x.id);
            json.Key("valid").Bool(x.valid);
            json.Key("voltage").Number(x.voltage);
            json.Key("age_us").Int(x.age_us);
          });
  Samples(json, "encoders", s.encoders, s.encoder_count,
          [&](const hardware::EncoderSample& x) {
            Name(json, n ? &n->encoders : nullptr, x.id);
            json.Key("valid").Bool(x.valid);
            json.Key("position_rot").Number(x.position_rot);
            json.Key("velocity_rps").Number(x.velocity_rps);
            json.Key("age_us").Int(x.age_us);
          });
  Samples(json, "pwm_outputs", s.pwm_outputs, s.pwm_output_count,
          [&](const hardware::PwmSample& x) {
            Name(json, n ? &n->pwm_outputs : nullptr, x.id);
            json.Key("valid").Bool(x.valid);
            json.Key("output").Number(x.output);
          });
  json.Key("health").Raw("{");
  json.Key("valid").Bool(s.health.valid);
  json.Key("battery_v").Number(s.health.battery_v);
  json.Key("brownout").Bool(s.health.brownout);
  // The one CAN bus slot this State reports (each State reports the next).
  json.Key("can_bus_count").Int(s.health.can_bus_count);
  const hardware::CanBusSample& bus = s.health.can_bus;
  json.Key("can_bus").Raw("{");
  json.Key("index").Int(s.health.can_bus_index);
  if (n != nullptr && s.health.can_bus_index < n->can_buses.size()) {
    json.Key("name").Text(n->can_buses[s.health.can_bus_index]);
  }
  json.Key("read_state").Text(CanBusReadStateName(bus.read_state));
  json.Key("utilization_frac").Number(bus.utilization_frac, 9);
  json.Key("bus_off_count").Int(bus.bus_off_count);
  json.Key("transmit_buffer_full_count").Int(bus.transmit_buffer_full_count);
  json.Key("receive_error_count").Int(bus.receive_error_count);
  json.Key("transmit_error_count").Int(bus.transmit_error_count);
  json.Key("read_failure_count").Int(bus.read_failure_count);
  json.Raw("}");
  json.Raw("}");
  json.Key("counters").Raw("{");
  json.Key("rejected_command_count").Int(s.counters.rejected_command_count);
  json.Key("neutralized_motor_count").Int(s.counters.neutralized_motor_count);
  json.Key("apply_failure_count").Int(s.counters.apply_failure_count);
  json.Key("device_reset_count").Int(s.counters.device_reset_count);
  json.Raw("}");
}

void CommandJson(Json& json, const hardware::Command& c, const DeviceNames* n) {
  json.Key("config_id").Id(c.config_id);
  json.Key("boot_id").Id(c.boot_id);
  json.Key("epoch").UInt(c.epoch);
  json.Key("observed_time_us").UInt(c.observed_time_us);
  Samples(json, "motors", c.motors, c.count,
          [&](const hardware::MotorRequest& m) {
            Name(json, n ? &n->motors : nullptr, m.id);
            json.Key("mode").Text(HardwareModeName(m.mode));
            json.Key("slot").Int(m.slot);
            json.Key("demand").Number(m.demand);
            json.Key("feedforward_v").Number(m.feedforward_v);
          });
  Samples(json, "digital_outputs", c.digital_outputs, c.digital_output_count,
          [&](const hardware::DigitalOutputRequest& x) {
            Name(json, n ? &n->digital_outputs : nullptr, x.id);
            json.Key("value").Bool(x.value);
          });
  Samples(json, "pwm_outputs", c.pwm_outputs, c.pwm_output_count,
          [&](const hardware::PwmRequest& x) {
            Name(json, n ? &n->pwm_outputs : nullptr, x.id);
            json.Key("output").Number(x.output);
          });
}

std::string_view Bounded(const char* text, std::size_t capacity) {
  return {text, strnlen(text, capacity)};
}

void DriverStationJson(Json& json, const ds::DriverStationData& d) {
  const std::uint32_t flags = d.fms.flags;
  json.Key("sample_time_us").UInt(d.sample_time_us);
  json.Key("mode_since_us").UInt(d.mode_since_us);
  json.Key("boot_id").Id(d.boot_id);
  json.Key("flags").Int(flags);
  json.Key("mode").Text(DriverStationModeName(flags));
  json.Key("ds_attached").Bool((flags & ds::kDsAttached) != 0);
  json.Key("fms_attached").Bool((flags & ds::kFmsAttached) != 0);
  json.Key("enabled").Bool((flags & ds::kEnabled) != 0);
  json.Key("estop").Bool((flags & ds::kEStop) != 0);
  json.Key("alliance").Text(AllianceName(d.fms.alliance));
  json.Key("station").Int(d.fms.station);
  json.Key("match_type").Text(MatchTypeName(d.fms.match_type));
  json.Key("match_number").Int(d.fms.match_number);
  json.Key("replay_number").Int(d.fms.replay_number);
  json.Key("match_time_s").Number(d.fms.match_time_s, 9);
  json.Key("game_specific_message")
      .Text(Bounded(d.fms.game_specific_message,
                    sizeof d.fms.game_specific_message));
  json.Key("event_name")
      .Text(Bounded(d.fms.event_name, sizeof d.fms.event_name));
  json.Key("joysticks").Raw("[");
  for (std::size_t port = 0; port < d.joysticks.size(); ++port) {
    const ds::JoystickData& j = d.joysticks[port];
    if (!j.connected) continue;
    json.Item().Raw("{");
    json.Key("port").Int(static_cast<std::int64_t>(port));
    json.Key("name").Text(Bounded(j.name, sizeof j.name));
    json.Key("axes").Raw("[");
    for (std::size_t i = 0; i < j.axis_count && i < j.axes.size(); ++i) {
      json.Item().Number(j.axes[i], 9);
    }
    json.Raw("]");
    // Bit i is button i + 1, as the Driver Station numbers them.
    json.Key("buttons").Int(j.buttons);
    json.Key("button_count").Int(j.button_count);
    json.Key("povs").Raw("[");
    for (std::size_t i = 0; i < j.pov_count && i < j.povs.size(); ++i) {
      json.Item().Int(j.povs[i]);
    }
    json.Raw("]}");
  }
  json.Raw("]");
}

const char* TypeName(HardwareRecord kind) {
  switch (kind) {
    case HardwareRecord::kState:
      return "state";
    case HardwareRecord::kCommand:
      return "command";
    case HardwareRecord::kDriverStation:
      return "driver_station";
    case HardwareRecord::kNone:
      break;
  }
  return "none";
}

std::string ManifestText(const std::filesystem::path& session) {
  std::ifstream stream{session / "manifest.json"};
  return {std::istreambuf_iterator<char>{stream}, {}};
}

// Where the value of member `key` of a manifest.json starts (past its colon
// and any white space), or npos.
std::size_t ManifestValue(const std::string& text, std::string_view key) {
  const std::string quoted = "\"" + std::string{key} + "\"";
  auto at = text.find(quoted);
  if (at == std::string::npos) return at;
  at = text.find(':', at + quoted.size());
  if (at == std::string::npos) return at;
  return text.find_first_not_of(" \t\r\n", at + 1);
}

// The unsigned integer member `key` of a manifest.json's text; nullopt when
// it has none, or one that is not a decimal integer.
std::optional<std::uint64_t> ManifestInteger(const std::string& text,
                                             std::string_view key) {
  const std::size_t at = ManifestValue(text, key);
  if (at == std::string::npos || text[at] < '0' || text[at] > '9') {
    return std::nullopt;
  }
  char* end = nullptr;
  errno = 0;
  const unsigned long long value = std::strtoull(text.c_str() + at, &end, 10);
  if (errno != 0 ||
      (*end != ',' && *end != '\n' && *end != ' ' && *end != '}')) {
    return std::nullopt;
  }
  return value;
}

// The "hardware_protocol_version" of a manifest.json's text (session.cc
// SessionManifest::ToJson): nullopt when it has none, or a value that is not
// a version (an integer from 1 to 65535).
std::optional<std::uint16_t> ManifestProtocolVersion(const std::string& text) {
  const auto value = ManifestInteger(text, "hardware_protocol_version");
  if (!value || *value < 1 || *value > 0xFFFF) return std::nullopt;
  return static_cast<std::uint16_t>(*value);
}

// The session id in the header of the event log at `path`; nullopt when it
// has no readable header of a talOS log.
std::optional<std::uint64_t> LogSessionId(const std::filesystem::path& path) {
  std::ifstream stream{path, std::ios::binary};
  event::log::FileHeader header{};
  if (!stream.read(reinterpret_cast<char*>(&header), sizeof header) ||
      std::memcmp(header.magic, event::log::FILE_MAGIC, sizeof header.magic) !=
          0) {
    return std::nullopt;
  }
  return header.session_id;
}

// The "config_path" string of a session's manifest.json (session.cc
// SessionManifest::ToJson, escaped by introspect::AppendJsonString), or empty.
std::string ManifestConfigPath(const std::filesystem::path& session) {
  const std::string text = ManifestText(session);
  const std::size_t at = ManifestValue(text, "config_path");
  if (at == std::string::npos || text[at] != '"') return "";
  std::string value;
  for (std::size_t i = at + 1; i < text.size(); ++i) {
    const char c = text[i];
    if (c == '"') return value;
    if (c != '\\') {
      value += c;
    } else if (i + 1 < text.size() && text[i + 1] != 'u') {
      value += text[++i];
    } else if (i + 5 < text.size()) {
      value += static_cast<char>(
          std::strtol(text.substr(i + 2, 4).c_str(), nullptr, 16));
      i += 5;
    }
  }
  return "";
}

// How far into a log FirstBridgeStatusVersion looks, in record event time
// from its first record. The bridge logs a /hw/status SEND on its first pass
// and every 20 ms after (HardwareNode::kStatusPeriodUs), so each of its log
// segments holds one within its first 20 ms unless the bridge stalled; a
// bridge from before it logged its statuses holds none, and costs this much
// of each segment, not the whole log.
constexpr std::int64_t kBridgeStatusSearchNs = 1'000'000'000;

// The protocol_version_id of the first /hw/status SEND in the log at `path`,
// when its manifest registers /hw/status as a sender of this build's
// BridgeStatus schema (and, with `session`, its header has that session id)
// and the SEND is within kBridgeStatusSearchNs of the log's first record;
// nullopt otherwise, and for a log that will not open. Reads records only
// of a log that registers such a sender, and only to that record or that
// bound.
std::optional<std::uint16_t> FirstBridgeStatusVersion(
    const std::string& path, std::optional<std::uint64_t> session) {
  using Status = ::bridge::BridgeStatus;
  try {
    event::log::WindowedLogReader reader{path};
    if (session && reader.session_id() != *session) return std::nullopt;
    const schema::Identity expected = schema::Of<Status>();
    std::optional<std::uint16_t> id;
    for (const auto& source : reader.manifest()) {
      if (source.kind == event::SourceKind::SENDER &&
          source.name == ::bridge::topics::kStatus &&
          source.schema == expected) {
        id = source.id;
        break;
      }
    }
    if (!id) return std::nullopt;
    event::log::LogReader::Record record;
    std::optional<std::int64_t> first_ns;
    while (reader.next(record)) {
      if (!first_ns) first_ns = record.header.event_time_ns;
      if (record.header.event_time_ns - *first_ns > kBridgeStatusSearchNs) {
        return std::nullopt;
      }
      if (record.header.source_id != *id ||
          record.header.kind !=
              static_cast<std::uint16_t>(event::EventKind::SEND)) {
        continue;
      }
      if (record.payload.size() != sizeof(Status)) return std::nullopt;
      Status status;
      std::memcpy(&status, record.payload.data(), sizeof status);
      if (status.protocol_version_id() == 0) return std::nullopt;
      return status.protocol_version_id();
    }
  } catch (const std::exception&) {
  }
  return std::nullopt;
}

}  // namespace

const char* CanBusReadStateName(hardware::CanBusReadState state) {
  switch (state) {
    case hardware::CanBusReadState::kNone:
      return "none";
    case hardware::CanBusReadState::kOk:
      return "ok";
    case hardware::CanBusReadState::kPending:
      return "pending";
    case hardware::CanBusReadState::kFailed:
      return "failed";
    case hardware::CanBusReadState::kStale:
      return "stale";
    case hardware::CanBusReadState::kSimulated:
      return "simulated";
  }
  return "unknown";
}

const char* HardwareModeName(hardware::Mode mode) {
  switch (mode) {
    case hardware::Mode::kNeutral:
      return "neutral";
    case hardware::Mode::kDutyCycle:
      return "duty_cycle";
    case hardware::Mode::kVoltage:
      return "voltage";
    case hardware::Mode::kVelocity:
      return "velocity";
    case hardware::Mode::kPosition:
      return "position";
    case hardware::Mode::kMotionMagic:
      return "motion_magic";
  }
  return "unknown";
}

const char* DriverStationModeName(std::uint32_t flags) {
  if ((flags & ds::kTest) != 0) return "test";
  if ((flags & ds::kAutonomous) != 0) return "autonomous";
  if ((flags & ds::kTeleop) != 0) return "teleop";
  return "none";
}

const char* AllianceName(ds::Alliance alliance) {
  switch (alliance) {
    case ds::Alliance::kRed:
      return "red";
    case ds::Alliance::kBlue:
      return "blue";
    case ds::Alliance::kUnknown:
      break;
  }
  return "unknown";
}

const char* MatchTypeName(ds::MatchType type) {
  switch (type) {
    case ds::MatchType::kPractice:
      return "practice";
    case ds::MatchType::kQualification:
      return "qualification";
    case ds::MatchType::kElimination:
      return "elimination";
    case ds::MatchType::kNone:
      break;
  }
  return "none";
}

std::string SessionConfigPath(const std::string& config,
                              const std::vector<std::string>& inputs) {
  std::string path = config;
  if (path.empty()) {
    for (const auto& input : inputs) {
      std::error_code ec;
      if (!std::filesystem::is_directory(input, ec)) continue;
      path = ManifestConfigPath(input);
      if (!path.empty()) break;
    }
    if (path.empty()) return path;
  }
  const char* base = std::getenv("BUILD_WORKING_DIRECTORY");
  if (std::filesystem::path{path}.is_absolute() || base == nullptr ||
      *base == '\0') {
    return path;
  }
  return (std::filesystem::path{base} / path).string();
}

std::optional<DeviceNames> LoadDeviceNames(const std::string& path,
                                           std::string& error) {
  try {
    return DeviceNames::FromConfig(config::ParseRobotConfig(path).hardware);
  } catch (const std::exception& e) {
    error = e.what();
    return std::nullopt;
  }
}

HardwareRecord HardwareRecordOf(std::string_view topic,
                                std::string_view schema_name) {
  if (schema_name != schema::kPacketName) return HardwareRecord::kNone;
  if (topic == hardware::kStateTopic) return HardwareRecord::kState;
  if (topic == hardware::kDriverStationTopic) {
    return HardwareRecord::kDriverStation;
  }
  if (topic == hardware::kCommandTopic ||
      topic.starts_with(hardware::kRequestTopicPrefix)) {
    return HardwareRecord::kCommand;
  }
  return HardwareRecord::kNone;
}

DeviceNames DeviceNames::FromConfig(const hardware::Config& config) {
  DeviceNames names;
  hardware::Config commissioned = config;
  commissioned.commissioned = true;
  names.config_id = hardware::ConfigurationId(config);
  names.commissioned_config_id = hardware::ConfigurationId(commissioned);
  const auto add = [](auto& map, const auto& devices) {
    for (const auto& device : devices) map[device.id] = device.name;
  };
  add(names.motors, config.motors);
  add(names.sensors, config.sensors);
  add(names.digital_inputs, config.digital_inputs);
  add(names.digital_outputs, config.digital_outputs);
  add(names.analog_inputs, config.analog_inputs);
  add(names.encoders, config.encoders);
  add(names.pwm_outputs, config.pwm_outputs);
  const hardware::CanBusList buses = hardware::CanBuses(config);
  for (std::size_t i = 0; i < buses.count; ++i) {
    names.can_buses.emplace_back(buses.names[i]);
  }
  return names;
}

std::string DecodeHardwareRecord(HardwareRecord kind,
                                 std::span<const std::byte> payload,
                                 HardwarePacket& out) {
  if (payload.size() != sizeof(hardware::Packet)) {
    return "payload is " + std::to_string(payload.size()) + " bytes, not a " +
           std::to_string(sizeof(hardware::Packet)) + "-byte hardware Packet";
  }
  hardware::Packet packet{};
  std::memcpy(&packet, payload.data(), sizeof packet);
  if (packet.size > packet.data.size()) {
    return "Packet size " + std::to_string(packet.size) + " exceeds its " +
           std::to_string(packet.data.size()) + "-byte buffer";
  }
  const auto bytes = packet.bytes();
  const auto rejected = [&](const char* what) {
    return std::string{"the "} + what + " codec rejected " +
           std::to_string(bytes.size()) + " bytes";
  };
  try {
    switch (kind) {
      case HardwareRecord::kState:
        out.state = {};
        if (!hardware::Decode(bytes, out.state)) return rejected("State");
        return "";
      case HardwareRecord::kCommand:
        out.command = {};
        if (!hardware::Decode(bytes, out.command)) return rejected("Command");
        return "";
      case HardwareRecord::kDriverStation:
        out.driver_station = {};
        if (!ds::Decode(bytes, out.driver_station)) {
          return rejected("Driver Station");
        }
        return "";
      case HardwareRecord::kNone:
        break;
    }
    return "not a hardware packet topic";
  } catch (const std::exception& e) {
    return e.what();
  }
}

std::optional<std::uint16_t> RecordedProtocol::version() const {
  if (bridge.size() == 1) return bridge.front();
  if (bridge.empty()) return manifest;
  return std::nullopt;
}

bool RecordedProtocol::refused() const {
  const auto recorded = version();
  return bridge.size() > 1 ||
         (recorded && *recorded != protocol::kProtocolVersion);
}

std::string RecordedProtocol::Refusal() const {
  if (!refused()) return "";
  const std::string build = std::to_string(protocol::kProtocolVersion);
  if (bridge.size() > 1) {
    std::string versions;
    for (std::size_t i = 0; i < bridge.size(); ++i) {
      if (i != 0) versions += i + 1 == bridge.size() ? " and " : ", ";
      versions += std::to_string(bridge[i]);
    }
    return "recorded under hardware protocols " + versions +
           " by bridge runs of different builds (/hw/status "
           "protocol_version_id); this build's is " +
           build;
  }
  return "recorded under hardware protocol " + std::to_string(*version()) +
         "; this build's is " + build +
         (bridge.empty() ? " (manifest.json hardware_protocol_version)"
                         : " (/hw/status protocol_version_id, the bridge's)");
}

std::string RecordedProtocol::Disagreement() const {
  if (bridge.size() != 1 || !manifest || *manifest == bridge.front()) {
    return "";
  }
  return "warning: the session's bridge recorded hardware protocol " +
         std::to_string(bridge.front()) +
         " (/hw/status protocol_version_id), its launcher " +
         std::to_string(*manifest) +
         " (manifest.json hardware_protocol_version): read under the "
         "bridge's";
}

RecordedProtocol RecordedProtocolVersion(
    const std::vector<std::string>& inputs) {
  for (const auto& input : inputs) {
    RecordedProtocol found;
    std::vector<std::string> logs;
    std::optional<std::uint64_t> session;
    std::error_code ec;
    if (std::filesystem::is_directory(input, ec)) {
      found.manifest = ManifestProtocolVersion(ManifestText(input));
      logs = event::log::FindLogFiles(input);
    } else {
      // A log named alone: with its session when the manifest beside it is
      // that log's session's, else alone.
      const std::filesystem::path file{input};
      const std::filesystem::path dir = file.has_parent_path()
                                            ? file.parent_path()
                                            : std::filesystem::path{"."};
      const std::string text = ManifestText(dir);
      const auto manifest_session = ManifestInteger(text, "session_id");
      session = LogSessionId(file);
      if (manifest_session && manifest_session == session) {
        found.manifest = ManifestProtocolVersion(text);
        logs = event::log::FindLogFiles(dir.string());
      } else {
        logs = {input};
        session.reset();
      }
    }
    for (const auto& log : logs) {
      const auto version = FirstBridgeStatusVersion(log, session);
      if (version && std::find(found.bridge.begin(), found.bridge.end(),
                               *version) == found.bridge.end()) {
        found.bridge.push_back(*version);
      }
    }
    if (found.manifest || !found.bridge.empty()) return found;
  }
  return {};
}

HardwareProtocolCheck::HardwareProtocolCheck(RecordedProtocol recorded)
    : recorded_{std::move(recorded)} {}

const HardwarePacket* HardwareProtocolCheck::Add(
    HardwareRecord kind, std::string_view topic, bool reader_copy,
    std::span<const std::byte> payload) {
  if (recorded_.refused()) return nullptr;
  if (!reader_copy) {
    if (!sent_.contains(topic)) sent_.emplace(topic);
  } else if (sent_.contains(topic)) {
    return nullptr;
  }
  ++packets_;
  std::string why = DecodeHardwareRecord(kind, payload, scratch_);
  if (why.empty()) return &scratch_;
  if (rejected_++ == 0) {
    first_topic_ = topic;
    first_why_ = std::move(why);
  }
  return nullptr;
}

std::string HardwareProtocolCheck::Warning() const {
  if (!foreign()) return "";
  const std::string build = std::to_string(protocol::kProtocolVersion);
  const std::string prefix = "warning: hardware packets not decoded: ";
  if (recorded_.refused()) return prefix + recorded_.Refusal();
  const std::string rejected =
      "this build's codecs reject " + std::to_string(rejected_) + " of the " +
      std::to_string(packets_) + " tried, the first on " + first_topic_ + ": " +
      first_why_;
  if (recorded_.version()) {
    const bool bridge = !recorded_.bridge.empty();
    return prefix + "recorded under this build's hardware protocol " + build +
           (bridge ? " (the bridge's), yet " : " (the launcher's), yet ") +
           rejected + "; the session's " +
           (bridge ? "nodes" : "bridge or nodes") + " may be of another build";
  }
  return prefix +
         "recorded under a hardware protocol other than this build's "
         "protocol " +
         build + " (the session records no protocol version); " + rejected;
}

std::string HardwareRecordJson(HardwareRecord kind,
                               std::span<const std::byte> payload,
                               const DeviceNames* names) {
  std::string out{"{"};
  Json json{out};
  json.Key("type").Text(TypeName(kind));
  const auto fail = [&](const std::string& why) {
    json.Key("error").Text(why);
    out += '}';
    return out;
  };
  HardwarePacket packet;
  const std::string error = DecodeHardwareRecord(kind, payload, packet);
  if (!error.empty()) return fail(error);
  try {
    switch (kind) {
      case HardwareRecord::kState:
        StateJson(
            json, packet.state,
            names && names->Matches(packet.state.config_id) ? names : nullptr);
        break;
      case HardwareRecord::kCommand:
        CommandJson(json, packet.command,
                    names && names->Matches(packet.command.config_id)
                        ? names
                        : nullptr);
        break;
      case HardwareRecord::kDriverStation:
        DriverStationJson(json, packet.driver_station);
        break;
      case HardwareRecord::kNone:
        break;
    }
  } catch (const std::exception& e) {
    out = "{";
    json.Key("type").Text(TypeName(kind));
    return fail(e.what());
  }
  out += '}';
  return out;
}

}  // namespace talos::launcher
