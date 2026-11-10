#include "talOS/launcher/hardware_record.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>

#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
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

const char* ModeName(hardware::Mode mode) {
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
  json.Key("can_utilization_frac").Number(s.health.can_utilization_frac);
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
            json.Key("mode").Text(ModeName(m.mode));
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

// The mode the Driver Station selected (its flags), whether or not the robot
// is enabled in it; not a node's own view of the mode.
const char* DsModeName(std::uint32_t flags) {
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

void DriverStationJson(Json& json, const ds::DriverStationData& d) {
  const std::uint32_t flags = d.fms.flags;
  json.Key("sample_time_us").UInt(d.sample_time_us);
  json.Key("mode_since_us").UInt(d.mode_since_us);
  json.Key("boot_id").Id(d.boot_id);
  json.Key("flags").Int(flags);
  json.Key("mode").Text(DsModeName(flags));
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

}  // namespace

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
  return names;
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
  if (payload.size() != sizeof(hardware::Packet)) {
    return fail("payload is " + std::to_string(payload.size()) +
                " bytes, not a " + std::to_string(sizeof(hardware::Packet)) +
                "-byte hardware Packet");
  }
  hardware::Packet packet{};
  std::memcpy(&packet, payload.data(), sizeof packet);
  if (packet.size > packet.data.size()) {
    return fail("Packet size " + std::to_string(packet.size) + " exceeds its " +
                std::to_string(packet.data.size()) + "-byte buffer");
  }
  const auto bytes = packet.bytes();
  const auto rejected = [&](const char* what) {
    return fail(std::string{"the "} + what + " codec rejected " +
                std::to_string(bytes.size()) + " bytes");
  };
  try {
    switch (kind) {
      case HardwareRecord::kState: {
        hardware::State state;
        if (!hardware::Decode(bytes, state)) return rejected("State");
        StateJson(json, state,
                  names && names->Matches(state.config_id) ? names : nullptr);
        break;
      }
      case HardwareRecord::kCommand: {
        hardware::Command command;
        if (!hardware::Decode(bytes, command)) return rejected("Command");
        CommandJson(
            json, command,
            names && names->Matches(command.config_id) ? names : nullptr);
        break;
      }
      case HardwareRecord::kDriverStation: {
        ds::DriverStationData data;
        if (!ds::Decode(bytes, data)) return rejected("Driver Station");
        DriverStationJson(json, data);
        break;
      }
      case HardwareRecord::kNone:
        return fail("not a hardware packet topic");
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
