// tlog_to_wpilog: talOS dispatch logs (.tlog) as one WPILib DataLog
// (.wpilog), so a session opens in AdvantageScope. Offline only.
//
//   tlog_to_wpilog [--config robot.toml] <log-file-or-session-dir> [more ...]
//                  -o <out.wpilog>
//
// The logs are streamed through MergeLogs (log_merge.h, as log_dump
// reads them), and each payload is walked with flatbuffers/reflection.h
// from the schema the log carries, so no recording build is needed. The
// hardware packets, which carry no schema, are decoded by log_dump's
// decoder (hardware_record.h). The mapping, the clock and the caveats are
// in talOS/launcher/README.md, "Converting a session to WPILOG".

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <bit>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "flatbuffers/reflection.h"
#include "flatbuffers/verifier.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/hardware/messages.h"
#include "talOS/launcher/hardware_record.h"
#include "talOS/launcher/log_inputs.h"
#include "talOS/launcher/log_merge.h"
#include "talOS/launcher/wpilog.h"
#include "talOS/schema/identity.h"
#include "talOS/schema/unit_suffix.h"

namespace {

using reflection::BaseType;
using talos::event::EventKind;
using talos::launcher::MergedLog;
using talos::launcher::MergedRecord;

// Per-reader dropped-message counters are entries of this owner, named
// /talos_log/dropped/<node> followed by the topic.
constexpr std::string_view kCounterOwner = "/talos_log";
// SendStatus::OK (talOS/events/event_loop_base.h), a SEND record's aux.
constexpr std::uint32_t kSendOk = 0;
// Guards against a hostile schema: real messages are far smaller. Every
// struct instance the walk visits counts against kMaxStructs, so a schema
// whose structs fan out to each other cannot make the walk exponential.
constexpr int kMaxDepth = 16;
constexpr std::size_t kMaxLeaves = 4096;
constexpr std::size_t kMaxStructs = 4096;

// One scalar or fixed array of scalars in a message, and its entry.
struct Leaf {
  std::string suffix;  // "pose/x_m", "wheels/0/speed_mps", "samples_m"
  BaseType base{BaseType::None};
  std::size_t offset{0};
  bool array{false};
  std::size_t count{0};  // elements, when `array`
  std::string unit;
  // Filled by BuildPlan: "<topic>/<suffix>", the WPILOG type, metadata.
  std::string name;
  std::string type;
  std::string metadata;
};

// A geometry.Pose2d, also written as AdvantageScope's [x, y, heading_rad].
struct PoseLeaf {
  std::string suffix;
  std::size_t x_offset{0};
  std::size_t y_offset{0};
  std::size_t yaw_offset{0};
  std::string name;  // "<topic>/<suffix>"
};

// How one schema's payload becomes entries; or why it cannot.
struct Plan {
  bool ok{false};
  std::string why;
  std::string schema;
  std::size_t bytes{0};
  std::vector<Leaf> leaves;
  std::vector<PoseLeaf> poses;
  std::size_t structs{0};  // struct instances visited, against kMaxStructs
};

// A Type's base type (or element type, with reflection::Type::VT_ELEMENT),
// read as its stored byte: a hostile schema's value outside BaseType is
// None, never held in the enum.
BaseType BaseOf(const reflection::Type& type,
                flatbuffers::voffset_t field = reflection::Type::VT_BASE_TYPE) {
  // Type privately is a Table, as every generated table (reflection.h
  // reads fields the same way).
  const auto raw =
      reinterpret_cast<const flatbuffers::Table*>(&type)->GetField<std::int8_t>(
          field, 0);
  if (raw < 0 || raw >= static_cast<std::int8_t>(BaseType::MaxBaseType)) {
    return BaseType::None;
  }
  return static_cast<BaseType>(raw);
}

bool IsScalar(BaseType base) {
  return base >= BaseType::UType && base <= BaseType::Double;
}

std::string Join(std::string_view path, std::string_view name) {
  if (path.empty()) return std::string{name};
  std::string out{path};
  out += '/';
  out += name;
  return out;
}

bool Refuse(Plan& plan, std::string_view field, std::string_view why) {
  plan.why = "field ";
  plan.why += field;
  plan.why += ' ';
  plan.why += why;
  return false;
}

// The pose mapping table has one row: geometry.Pose2d (talOS/msgs) with its
// three double fields, at struct path P, adds the double[] entry P (or
// "pose" when the Pose2d is the whole message).
void AddPose(const reflection::Object& object, std::size_t offset,
             const std::string& path, Plan& plan) {
  if (object.name()->string_view() != "geometry.Pose2d") return;
  std::optional<std::size_t> x;
  std::optional<std::size_t> y;
  std::optional<std::size_t> yaw;
  for (const reflection::Field* field : *object.fields()) {
    if (BaseOf(*field->type()) != BaseType::Double) continue;
    const std::string_view name = field->name()->string_view();
    const std::size_t at = offset + field->offset();
    if (name == "x_m") x = at;
    if (name == "y_m") y = at;
    if (name == "yaw_rad") yaw = at;
  }
  if (x && y && yaw) {
    plan.poses.push_back(
        {path.empty() ? std::string{"pose"} : path, *x, *y, *yaw, {}});
  }
}

bool AddStruct(const reflection::Schema& schema,
               const reflection::Object& object, std::size_t offset,
               const std::string& path, int depth, Plan& plan) {
  if (depth > kMaxDepth) {
    plan.why = "structs nest more than " + std::to_string(kMaxDepth) + " deep";
    return false;
  }
  if (++plan.structs > kMaxStructs) {
    plan.why = "it expands to more than " + std::to_string(kMaxStructs) +
               " struct instances";
    return false;
  }
  const auto bytes = static_cast<std::size_t>(object.bytesize());
  // reflection lists fields by name; keep the declared order.
  std::vector<const reflection::Field*> fields(object.fields()->begin(),
                                               object.fields()->end());
  std::sort(fields.begin(), fields.end(),
            [](const reflection::Field* a, const reflection::Field* b) {
              return a->id() < b->id();
            });
  for (const reflection::Field* field : fields) {
    const reflection::Type& type = *field->type();
    const std::string_view name = field->name()->string_view();
    const bool array = BaseOf(type) == BaseType::Array;
    const BaseType base =
        array ? BaseOf(type, reflection::Type::VT_ELEMENT) : BaseOf(type);
    const std::size_t count = array ? type.fixed_length() : 1;
    const reflection::Object* nested = nullptr;
    std::size_t element_bytes = 0;
    if (base == BaseType::Obj) {
      if (type.index() < 0 ||
          static_cast<std::size_t>(type.index()) >= schema.objects()->size()) {
        return Refuse(plan, name, "names no object");
      }
      nested = schema.objects()->Get(
          static_cast<flatbuffers::uoffset_t>(type.index()));
      if (!nested->is_struct() || nested->bytesize() <= 0) {
        return Refuse(plan, name, "is not a struct");
      }
      element_bytes = static_cast<std::size_t>(nested->bytesize());
    } else if (IsScalar(base)) {
      element_bytes = flatbuffers::GetTypeSize(base);
    } else {
      return Refuse(plan, name, "is not a scalar, enum or struct");
    }
    if (field->offset() + count * element_bytes > bytes) {
      return Refuse(plan, name, "lies outside its struct");
    }
    const std::size_t at = offset + field->offset();
    const std::string field_path = Join(path, name);
    if (nested == nullptr) {
      if (plan.leaves.size() >= kMaxLeaves) {
        return Refuse(plan, name, "is past the field limit");
      }
      plan.leaves.push_back({field_path,
                             base,
                             at,
                             array,
                             array ? count : 0,
                             std::string{talos::schema::unit_for_field(name)},
                             {},
                             {},
                             {}});
      continue;
    }
    if (!array) {
      if (!AddStruct(schema, *nested, at, field_path, depth + 1, plan)) {
        return false;
      }
      continue;
    }
    for (std::size_t k = 0; k < count; ++k) {
      if (!AddStruct(schema, *nested, at + k * element_bytes,
                     Join(field_path, std::to_string(k)), depth + 1, plan)) {
        return false;
      }
    }
  }
  AddPose(object, offset, path, plan);
  return true;
}

// A topic's Plan from the schema identity its log carries; entry names and
// metadata are filled in by BuildPlan below.
Plan WalkSchema(const talos::schema::Identity& identity) {
  Plan plan;
  plan.schema = identity.name;
  if (identity.bfbs.empty()) {
    plan.why =
        (identity.name.empty() ? std::string{"no schema"} : identity.name) +
        " carries no FlatBuffers schema";
    return plan;
  }
  const auto* bfbs =
      reinterpret_cast<const std::uint8_t*>(identity.bfbs.data());
  flatbuffers::Verifier verifier{bfbs, identity.bfbs.size()};
  if (!reflection::VerifySchemaBuffer(verifier)) {
    plan.why = "its schema bytes are not a reflection schema";
    return plan;
  }
  const reflection::Schema& schema = *reflection::GetSchema(bfbs);
  const std::string_view inner = talos::schema::EnvelopeInner(identity.name);
  const std::string name{inner.empty() ? std::string_view{identity.name}
                                       : inner};
  const reflection::Object* object =
      schema.objects()->LookupByKey(name.c_str());
  if (object == nullptr || !object->is_struct() || object->bytesize() <= 0) {
    plan.why = "its schema has no struct " + name;
    return plan;
  }
  std::size_t offset = 0;
  std::string path;
  if (!inner.empty()) {
    // talOS/devices/target.h's header, as DecodeToText names it.
    plan.leaves.push_back(
        {"mode_since_us", BaseType::ULong, 0, false, 0, "us", {}, {}, {}});
    plan.leaves.push_back({"active",
                           BaseType::Bool,
                           talos::schema::kEnvelopeActiveOffset,
                           false,
                           0,
                           "",
                           {},
                           {},
                           {}});
    offset = talos::schema::kEnvelopeHeaderBytes;
    path = "message";
  }
  plan.bytes = offset + static_cast<std::size_t>(object->bytesize());
  plan.ok = AddStruct(schema, *object, offset, path, 0, plan);
  return plan;
}

// `bytes` little-endian bytes at `at`, whatever the host's byte order.
std::uint64_t Load(const std::byte* at, std::size_t bytes) {
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < bytes; ++i) {
    value |= static_cast<std::uint64_t>(at[i]) << (8U * i);
  }
  return value;
}

// An integer, enum or UType at `at`; a ulong keeps its bits (wraps above
// INT64_MAX).
std::int64_t LoadInteger(BaseType base, const std::byte* at) {
  switch (base) {
    case BaseType::Byte:
      return static_cast<std::int8_t>(static_cast<std::uint8_t>(Load(at, 1)));
    case BaseType::Short:
      return static_cast<std::int16_t>(static_cast<std::uint16_t>(Load(at, 2)));
    case BaseType::Int:
      return static_cast<std::int32_t>(static_cast<std::uint32_t>(Load(at, 4)));
    case BaseType::UShort:
      return static_cast<std::int64_t>(Load(at, 2));
    case BaseType::UInt:
      return static_cast<std::int64_t>(Load(at, 4));
    case BaseType::Long:
    case BaseType::ULong:
      return static_cast<std::int64_t>(Load(at, 8));
    default:  // UType, UByte
      return static_cast<std::int64_t>(Load(at, 1));
  }
}

double LoadDouble(const std::byte* at) {
  return std::bit_cast<double>(Load(at, 8));
}

void AppendElement(BaseType base, const std::byte* at,
                   std::vector<std::uint8_t>& out) {
  switch (base) {
    case BaseType::Bool:
      talos::wpilog::AppendBoolean(out, at[0] != std::byte{0});
      return;
    case BaseType::Float:
      talos::wpilog::AppendFloat(
          out, std::bit_cast<float>(static_cast<std::uint32_t>(Load(at, 4))));
      return;
    case BaseType::Double:
      talos::wpilog::AppendDouble(out, LoadDouble(at));
      return;
    default:
      talos::wpilog::AppendInt64(out, LoadInteger(base, at));
      return;
  }
}

std::string TypeOf(const Leaf& leaf) {
  std::string type = leaf.base == BaseType::Bool     ? "boolean"
                     : leaf.base == BaseType::Float  ? "float"
                     : leaf.base == BaseType::Double ? "double"
                                                     : "int64";
  if (leaf.array) type += "[]";
  return type;
}

std::string JsonString(std::string_view value) {
  std::string result{"\""};
  constexpr char kHex[] = "0123456789abcdef";
  for (const char c : value) {
    const auto byte = static_cast<unsigned char>(c);
    if (c == '"' || c == '\\') {
      result += '\\';
      result += c;
    } else if (byte < 0x20) {
      result += "\\u00";
      result += kHex[byte >> 4U];
      result += kHex[byte & 15U];
    } else {
      result += c;
    }
  }
  result += '"';
  return result;
}

// {"source":"talos.tlog","schema":...,"unit":...,"format":...}
std::string Metadata(std::string_view schema, std::string_view unit,
                     std::string_view format = {}) {
  std::string out = "{\"source\":\"talos.tlog\"";
  if (!schema.empty()) out += ",\"schema\":" + JsonString(schema);
  if (!unit.empty()) out += ",\"unit\":" + JsonString(unit);
  if (!format.empty()) out += ",\"format\":" + JsonString(format);
  out += '}';
  return out;
}

// `topic`'s plan for `identity`, with every entry's name, type and metadata.
Plan BuildPlan(std::string_view topic,
               const talos::schema::Identity& identity) {
  Plan plan = WalkSchema(identity);
  if (!plan.ok) return plan;
  for (Leaf& leaf : plan.leaves) {
    leaf.name = Join(topic, leaf.suffix);
    leaf.type = TypeOf(leaf);
    leaf.metadata = Metadata(plan.schema, leaf.unit);
  }
  for (PoseLeaf& pose : plan.poses) pose.name = Join(topic, pose.suffix);
  return plan;
}

// "dropped/<node><topic>", the counter's suffix below kCounterOwner.
std::string DroppedSuffix(std::string_view process, std::string_view topic) {
  std::string suffix{"dropped/"};
  suffix += process;
  suffix += topic;
  return suffix;
}

std::string SizeMismatch(std::size_t bytes, const Plan& plan) {
  std::string why = std::to_string(bytes);
  why += "-byte payload, ";
  why += plan.schema;
  why += " is ";
  why += std::to_string(plan.bytes);
  return why;
}

// Entries by name, started on first use, each owned by one topic.
class Entries {
 public:
  explicit Entries(talos::wpilog::Writer& writer) : writer_{writer} {}

  // The id of `owner`'s entry `name` (`owner/suffix`), starting it at
  // `timestamp_us` on first use. A name is its first owner's: another
  // topic whose field path lands on it (/arm/state field left.angle_rad and
  // /arm/state/left field angle_rad) gets `owner:suffix` instead, which no
  // other topic can produce (names hold no ':'). 0 (skip the value) when the
  // entry was started with another type.
  std::uint32_t get(const std::string& name, std::string_view owner,
                    std::string_view suffix, const std::string& type,
                    const std::string& metadata, std::uint64_t timestamp_us) {
    auto found = entries_.find(name);
    if (found != entries_.end() && found->second.owner != owner) {
      std::string renamed{owner};
      renamed += ':';
      renamed += suffix;
      if (renames_.insert(renamed).second) {
        std::fprintf(stderr,
                     "tlog_to_wpilog: warning: %s is %s's; %.*s's entry is "
                     "%s\n",
                     name.c_str(), found->second.owner.c_str(),
                     static_cast<int>(owner.size()), owner.data(),
                     renamed.c_str());
      }
      return start_or_find(renamed, owner, type, metadata, timestamp_us);
    }
    return start_or_find(name, owner, type, metadata, timestamp_us);
  }

  std::size_t size() const { return entries_.size(); }

 private:
  std::uint32_t start_or_find(const std::string& name, std::string_view owner,
                              const std::string& type,
                              const std::string& metadata,
                              std::uint64_t timestamp_us) {
    const auto found = entries_.find(name);
    if (found == entries_.end()) {
      const std::uint32_t id =
          writer_.start(name, type, metadata, timestamp_us);
      entries_.emplace(name, Entry{id, type, std::string{owner}});
      return id;
    }
    if (found->second.owner == owner && found->second.type == type) {
      return found->second.id;
    }
    if (conflicts_.insert(name).second) {
      std::fprintf(stderr,
                   "tlog_to_wpilog: warning: %s is %s, also logged as %s; "
                   "keeping the first\n",
                   name.c_str(), found->second.type.c_str(), type.c_str());
    }
    return 0;
  }

  struct Entry {
    std::uint32_t id;
    std::string type;
    std::string owner;
  };
  talos::wpilog::Writer& writer_;
  std::unordered_map<std::string, Entry> entries_;
  std::set<std::string> conflicts_;
  std::set<std::string> renames_;
};

// One decoded hardware packet's values (hardware_record.h), each written by
// `write` from `payload`: entries `<topic>/<field path>`, a device by its
// config name when `names` match the packet's config_id, else by its
// logical id. README.md lists the fields.
class HardwareValues {
 public:
  using Write = std::function<void(const std::string& suffix, const char* type,
                                   std::string_view unit)>;

  HardwareValues(std::vector<std::uint8_t>& payload, Write write)
      : payload_{payload}, write_{std::move(write)} {}

  // Whether a packet was written with logical ids where names were wanted:
  // no names, or a config_id that is not theirs.
  bool unnamed() const { return unnamed_; }

  void State(const talos::hardware::State& s,
             const talos::launcher::DeviceNames* names) {
    const auto* n = Names(names, s.config_id);
    UInt("config_id", s.config_id);
    UInt("boot_id", s.boot_id);
    UInt("epoch", s.epoch);
    UInt("sample_time_us", s.sample_time_us);
    UInt("sequence", s.sequence);
    UInt("last_command_sequence", s.last_command_sequence);
    namespace hw = talos::hardware;
    Bool("configured", (s.flags & hw::kConfigured) != 0);
    Bool("enabled", (s.flags & hw::kEnabled) != 0);
    Bool("command_active", (s.flags & hw::kCommandActive) != 0);
    Bool("hardware_fault", (s.flags & hw::kHardwareFault) != 0);
    Each(s.motors, s.motor_count, "motors/", n ? &n->motors : nullptr,
         [&](const std::string& p, const hw::MotorSample& m) {
           Bool(p + "valid", m.valid);
           Double(p + "position_rot", m.position_rot);
           Double(p + "velocity_rps", m.velocity_rps);
           Double(p + "voltage", m.voltage, "V");
           Double(p + "stator_current_a", m.stator_current_a);
           Double(p + "supply_current_a", m.supply_current_a);
           Double(p + "temperature_c", m.temperature_c, "°C");
           Int(p + "position_age_us", m.position_age_us);
           Int(p + "velocity_age_us", m.velocity_age_us);
           Int(p + "fault_flags", m.fault_flags);
           Bool(p + "faulted", hw::MotorFaulted(m.fault_flags));
           Bool(p + "forward_limit",
                (m.fault_flags & hw::kMotorForwardHardLimitFlag) != 0);
           Bool(p + "reverse_limit",
                (m.fault_flags & hw::kMotorReverseHardLimitFlag) != 0);
         });
    Each(s.sensors, s.sensor_count, "sensors/", n ? &n->sensors : nullptr,
         [&](const std::string& p, const hw::SensorSample& x) {
           Bool(p + "valid", x.valid);
           Double(p + "position_rot", x.position_rot);
           Double(p + "velocity_rps", x.velocity_rps);
           Int(p + "position_age_us", x.position_age_us);
           Int(p + "velocity_age_us", x.velocity_age_us);
         });
    Each(s.digital_inputs, s.digital_input_count, "digital_inputs/",
         n ? &n->digital_inputs : nullptr,
         [&](const std::string& p, const hw::DigitalInputSample& x) {
           Bool(p + "valid", x.valid);
           Bool(p + "value", x.value);
         });
    Each(s.digital_outputs, s.digital_output_count, "digital_outputs/",
         n ? &n->digital_outputs : nullptr,
         [&](const std::string& p, const hw::DigitalOutputSample& x) {
           Bool(p + "valid", x.valid);
           Bool(p + "value", x.value);
         });
    Each(s.analog_inputs, s.analog_input_count, "analog_inputs/",
         n ? &n->analog_inputs : nullptr,
         [&](const std::string& p, const hw::AnalogInputSample& x) {
           Bool(p + "valid", x.valid);
           Double(p + "voltage", x.voltage, "V");
           Int(p + "age_us", x.age_us);
         });
    Each(s.encoders, s.encoder_count, "encoders/", n ? &n->encoders : nullptr,
         [&](const std::string& p, const hw::EncoderSample& x) {
           Bool(p + "valid", x.valid);
           Double(p + "position_rot", x.position_rot);
           Double(p + "velocity_rps", x.velocity_rps);
           Int(p + "age_us", x.age_us);
         });
    Each(s.pwm_outputs, s.pwm_output_count, "pwm_outputs/",
         n ? &n->pwm_outputs : nullptr,
         [&](const std::string& p, const hw::PwmSample& x) {
           Bool(p + "valid", x.valid);
           Double(p + "output", x.output);
         });
    Bool("health/valid", s.health.valid);
    Double("health/battery_v", s.health.battery_v);
    Bool("health/brownout", s.health.brownout);
    Int("health/can_bus_count", s.health.can_bus_count);
    // Each State reports one CAN bus slot, the next one each time: an entry
    // per bus, by its name when the config gives it.
    const std::size_t index = s.health.can_bus_index;
    const std::string bus =
        "health/can_bus/" +
        (n != nullptr && index < n->can_buses.size() ? n->can_buses[index]
                                                     : std::to_string(index)) +
        "/";
    const hw::CanBusSample& b = s.health.can_bus;
    Text(bus + "read_state",
         talos::launcher::CanBusReadStateName(b.read_state));
    Double(bus + "utilization_frac", b.utilization_frac);
    Int(bus + "bus_off_count", b.bus_off_count);
    Int(bus + "transmit_buffer_full_count", b.transmit_buffer_full_count);
    Int(bus + "receive_error_count", b.receive_error_count);
    Int(bus + "transmit_error_count", b.transmit_error_count);
    Int(bus + "read_failure_count", b.read_failure_count);
    Int("counters/rejected_command_count", s.counters.rejected_command_count);
    Int("counters/neutralized_motor_count", s.counters.neutralized_motor_count);
    Int("counters/apply_failure_count", s.counters.apply_failure_count);
    Int("counters/device_reset_count", s.counters.device_reset_count);
  }

  void Command(const talos::hardware::Command& c,
               const talos::launcher::DeviceNames* names) {
    const auto* n = Names(names, c.config_id);
    UInt("config_id", c.config_id);
    UInt("boot_id", c.boot_id);
    UInt("epoch", c.epoch);
    UInt("observed_time_us", c.observed_time_us);
    namespace hw = talos::hardware;
    Each(c.motors, c.count, "motors/", n ? &n->motors : nullptr,
         [&](const std::string& p, const hw::MotorRequest& m) {
           Text(p + "mode", talos::launcher::HardwareModeName(m.mode));
           Int(p + "slot", m.slot);
           // Duty ratio, volts, rotations/s or rotations, by the mode.
           Double(p + "demand", m.demand);
           Double(p + "feedforward_v", m.feedforward_v);
         });
    Each(c.digital_outputs, c.digital_output_count, "digital_outputs/",
         n ? &n->digital_outputs : nullptr,
         [&](const std::string& p, const hw::DigitalOutputRequest& x) {
           Bool(p + "value", x.value);
         });
    Each(c.pwm_outputs, c.pwm_output_count, "pwm_outputs/",
         n ? &n->pwm_outputs : nullptr,
         [&](const std::string& p, const hw::PwmRequest& x) {
           Double(p + "output", x.output);
         });
  }

  // `seen` holds a bit per joystick port connected in an earlier packet of
  // this topic, so a port that disconnects reads connected = false.
  void DriverStation(const talos::driver_station::DriverStationData& d,
                     std::uint32_t& seen) {
    namespace ds = talos::driver_station;
    const std::uint32_t flags = d.fms.flags;
    UInt("sample_time_us", d.sample_time_us);
    UInt("mode_since_us", d.mode_since_us);
    UInt("boot_id", d.boot_id);
    Bool("ds_attached", (flags & ds::kDsAttached) != 0);
    Bool("fms_attached", (flags & ds::kFmsAttached) != 0);
    Bool("enabled", (flags & ds::kEnabled) != 0);
    Bool("autonomous", (flags & ds::kAutonomous) != 0);
    Bool("teleop", (flags & ds::kTeleop) != 0);
    Bool("test", (flags & ds::kTest) != 0);
    Bool("estop", (flags & ds::kEStop) != 0);
    Text("mode", talos::launcher::DriverStationModeName(flags));
    Text("alliance", talos::launcher::AllianceName(d.fms.alliance));
    Int("station", d.fms.station);
    Text("match_type", talos::launcher::MatchTypeName(d.fms.match_type));
    Int("match_number", d.fms.match_number);
    Int("replay_number", d.fms.replay_number);
    Double("match_time_s", d.fms.match_time_s, "s");
    for (std::size_t port = 0; port < d.joysticks.size(); ++port) {
      const ds::JoystickData& j = d.joysticks[port];
      const std::uint32_t bit = 1U << port;
      if (!j.connected && (seen & bit) == 0) continue;
      const std::string p = "joysticks/" + std::to_string(port) + "/";
      Bool(p + "connected", j.connected);
      if (!j.connected) continue;
      seen |= bit;
      payload_.clear();
      for (std::size_t i = 0; i < j.axis_count && i < j.axes.size(); ++i) {
        talos::wpilog::AppendDouble(payload_, j.axes[i]);
      }
      write_(p + "axes", "double[]", {});
      // Button i + 1 (as the Driver Station numbers them) is element i.
      payload_.clear();
      for (std::size_t i = 0; i < j.button_count && i < ds::kMaxButtons; ++i) {
        talos::wpilog::AppendBoolean(payload_, ((j.buttons >> i) & 1U) != 0);
      }
      write_(p + "buttons", "boolean[]", {});
      payload_.clear();
      for (std::size_t i = 0; i < j.pov_count && i < j.povs.size(); ++i) {
        talos::wpilog::AppendInt64(payload_, j.povs[i]);
      }
      write_(p + "povs", "int64[]", {});
    }
  }

 private:
  const talos::launcher::DeviceNames* Names(
      const talos::launcher::DeviceNames* names, std::uint64_t config_id) {
    if (names != nullptr && names->Matches(config_id)) return names;
    unnamed_ = true;
    return nullptr;
  }

  static std::string Find(const std::map<std::uint16_t, std::string>& names,
                          std::uint16_t id) {
    const auto found = names.find(id);
    return found == names.end() ? std::string{} : found->second;
  }

  // Calls `item(path, sample)` for the first `count` samples, `path` being
  // `group` then the device's name or logical id and a '/'.
  template <class Array, class Item>
  void Each(const Array& samples, std::size_t count, const char* group,
            const std::map<std::uint16_t, std::string>* names, Item item) {
    for (std::size_t i = 0; i < count && i < samples.size(); ++i) {
      const auto id = samples[i].id;
      std::string path{group};
      const auto name = names == nullptr ? std::string{} : Find(*names, id);
      path += name.empty() ? std::to_string(id) : name;
      path += '/';
      item(path, samples[i]);
    }
  }

  void Bool(const std::string& suffix, bool value) {
    payload_.clear();
    talos::wpilog::AppendBoolean(payload_, value);
    write_(suffix, "boolean", {});
  }
  void Int(const std::string& suffix, std::int64_t value) {
    payload_.clear();
    talos::wpilog::AppendInt64(payload_, value);
    write_(suffix, "int64", talos::schema::unit_for_field(suffix));
  }
  // An unsigned 64-bit value (an id, a time, a sequence): its bits, as a
  // ulong field is.
  void UInt(const std::string& suffix, std::uint64_t value) {
    Int(suffix, static_cast<std::int64_t>(value));
  }
  void Double(const std::string& suffix, double value) {
    Double(suffix, value, talos::schema::unit_for_field(suffix));
  }
  void Double(const std::string& suffix, double value, std::string_view unit) {
    payload_.clear();
    talos::wpilog::AppendDouble(payload_, value);
    write_(suffix, "double", unit);
  }
  void Text(const std::string& suffix, std::string_view value) {
    payload_.assign(value.begin(), value.end());
    write_(suffix, "string", {});
  }

  std::vector<std::uint8_t>& payload_;
  Write write_;
  bool unnamed_{false};
};

struct Options {
  std::vector<std::string> inputs;
  std::string output;
  std::string config;
};

bool ParseOptions(int argc, char** argv, Options& options, std::string& error) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--") {
      while (++i < argc) options.inputs.emplace_back(argv[i]);
      break;
    }
    if (arg == "-o" || arg == "--output" || arg == "--config") {
      if (++i == argc) {
        error = "missing value for " + arg;
        return false;
      }
      (arg == "--config" ? options.config : options.output) = argv[i];
      continue;
    }
    if (arg.size() > 1 && arg.starts_with('-')) {
      error = "unknown flag: " + arg;
      return false;
    }
    options.inputs.push_back(arg);
  }
  if (options.output.empty()) {
    error = "an output file is required (-o <out.wpilog>)";
    return false;
  }
  if (options.inputs.empty()) {
    error = "a log file or session directory is required";
    return false;
  }
  return true;
}

// Microseconds of `ns` after `start_ns`; 0 before it. Unsigned arithmetic:
// a damaged log's times must not overflow.
std::uint64_t Micros(std::int64_t ns, std::int64_t start_ns) {
  if (ns <= start_ns) return 0;
  return (static_cast<std::uint64_t>(ns) -
          static_cast<std::uint64_t>(start_ns)) /
         1000;
}

int Fail(const std::string& message) {
  std::fprintf(stderr, "tlog_to_wpilog: error: %s\n", message.c_str());
  return 1;
}

// One publication: its topic (an index into the topic table), RTMS
// sequence and publish time. A SEND and every reader's MESSAGE of it carry
// the same three.
using Publication = std::tuple<std::size_t, std::uint64_t, std::int64_t>;

// A record that becomes values or a dropped count, kept past the merge: its
// log (process and manifest, shared, not copied), source, kind, dropped
// count, and its payload only when it carries values.
struct Item {
  std::int64_t time_ns;
  std::shared_ptr<const MergedLog> log;
  const talos::event::Registration* source;
  std::uint16_t kind;
  std::uint32_t aux;
  bool values;
  std::vector<std::byte> payload;
};

// The records that become values or dropped counts, in publish-time order.
// Each publication's values come once: from its writer's SEND when that is
// logged (a refused send was never published), otherwise from the first
// reader's MESSAGE of it in merge order. So a reader's copy fills what the
// writer's log lacks (a capped log's deleted head, a cut tail) and every
// reader contributes what it received. Every MESSAGE with a dropped count
// is an item, for that reader's counter. The session is streamed twice:
// once for the logged SENDs, warning about damage then, and once to select.
// False, with `error` set, when the merge fails; `records` counts the
// records read.
bool SelectItems(const std::vector<std::string>& files,
                 std::vector<Item>& items, std::size_t& records,
                 std::string& error) {
  std::map<std::string, std::size_t, std::less<>> topics;
  const auto key = [&](const MergedRecord& r) {
    auto topic = topics.find(r.source_name());
    if (topic == topics.end()) {
      topic = topics.emplace(std::string{r.source_name()}, topics.size()).first;
    }
    return Publication{topic->second, r.header.sequence,
                       r.header.publish_time_ns};
  };
  const auto is = [](const MergedRecord& r, EventKind kind) {
    return r.header.kind == static_cast<std::uint16_t>(kind);
  };
  std::set<Publication> sent;
  if (!talos::launcher::MergeLogs(
          files,
          [&](const MergedRecord& r) {
            if (r.source != nullptr && is(r, EventKind::SEND) &&
                r.header.aux == kSendOk) {
              sent.insert(key(r));
            }
            return true;
          },
          error)) {
    return false;
  }
  std::set<Publication> taken;
  records = 0;
  const auto keep = [&](const MergedRecord& r, bool values) {
    Item& item = items.emplace_back(Item{r.header.publish_time_ns,
                                         r.log,
                                         r.source,
                                         r.header.kind,
                                         r.header.aux,
                                         values,
                                         {}});
    if (values) item.payload.assign(r.payload.begin(), r.payload.end());
  };
  if (!talos::launcher::MergeLogs(
          files,
          [&](const MergedRecord& r) {
            ++records;
            if (r.source == nullptr) return true;
            if (is(r, EventKind::SEND)) {
              if (r.header.aux == kSendOk && taken.insert(key(r)).second) {
                keep(r, true);
              }
            } else if (is(r, EventKind::MESSAGE)) {
              const Publication publication = key(r);
              const bool values = !sent.contains(publication) &&
                                  taken.insert(publication).second;
              if (values || r.header.aux > 0) keep(r, values);
            }
            return true;
          },
          error, talos::launcher::MergeWarnings::kQuiet)) {
    return false;
  }
  // MergeLogs orders by event time; a WPILOG is stamped by publication.
  std::stable_sort(
      items.begin(), items.end(),
      [](const Item& a, const Item& b) { return a.time_ns < b.time_ns; });
  return true;
}

struct Totals {
  std::size_t entries{0};
  std::size_t values{0};
  std::map<std::string, std::string> skipped;  // topic -> why
  bool hardware_unnamed{false};  // a hardware packet's devices by id
};

// The hardware packet topic an item carries (hardware_record.h), or kNone.
talos::launcher::HardwareRecord HardwareKind(const Item& item) {
  return talos::launcher::HardwareRecordOf(item.source->name,
                                           item.source->schema.name);
}

// Writes `items` to `writer`; see README.md for the mapping. The hardware
// packets are written only with `decode_hardware` (the session was recorded
// under this build's protocol: HardwareProtocolCheck), their devices named
// from `names` (may be null).
Totals WriteItems(const std::vector<Item>& items, std::int64_t start_ns,
                  bool decode_hardware,
                  const talos::launcher::DeviceNames* names,
                  talos::wpilog::Writer& writer) {
  Totals totals;
  Entries entries{writer};
  std::map<std::tuple<std::string, std::string, std::uint64_t>, Plan> plans;
  std::map<std::pair<std::string, std::string>, std::int64_t> dropped;
  std::vector<std::uint8_t> payload;
  const std::string pose_metadata =
      Metadata("geometry.Pose2d", {}, "[x_m, y_m, yaw_rad]");
  const std::string dropped_metadata =
      Metadata({}, {}, "messages this node's subscription lapped");
  const auto write = [&](const std::string& name, std::string_view owner,
                         std::string_view suffix, const std::string& type,
                         const std::string& metadata, std::uint64_t time_us) {
    const std::uint32_t id =
        entries.get(name, owner, suffix, type, metadata, time_us);
    if (id == 0) return;
    writer.append(id, payload, time_us);
    ++totals.values;
  };

  // The hardware packets, decoded by hardware_record.h. Metadata by unit:
  // few distinct ones, written for every value.
  std::map<std::string, std::string, std::less<>> hardware_metadata;
  std::map<std::string, std::uint32_t> joysticks_seen;  // topic -> ports
  talos::launcher::HardwarePacket packet;
  const std::string* hardware_topic = nullptr;
  std::uint64_t hardware_time_us = 0;
  HardwareValues hardware{
      payload,
      [&](const std::string& suffix, const char* type, std::string_view unit) {
        auto metadata = hardware_metadata.find(unit);
        if (metadata == hardware_metadata.end()) {
          metadata = hardware_metadata
                         .emplace(std::string{unit},
                                  Metadata(talos::schema::kPacketName, unit))
                         .first;
        }
        write(Join(*hardware_topic, suffix), *hardware_topic, suffix, type,
              metadata->second, hardware_time_us);
      }};

  for (const Item& item : items) {
    const std::uint64_t time_us = Micros(item.time_ns, start_ns);
    const std::string& process = item.log->process_name;
    const std::string& topic = item.source->name;
    if (item.kind == static_cast<std::uint16_t>(EventKind::MESSAGE) &&
        item.aux > 0) {
      std::int64_t& total = dropped[{process, topic}];
      total += item.aux;
      payload.clear();
      talos::wpilog::AppendInt64(payload, total);
      const std::string suffix = DroppedSuffix(process, topic);
      write(Join(kCounterOwner, suffix), kCounterOwner, suffix, "int64",
            dropped_metadata, time_us);
    }
    if (!item.values) continue;
    const talos::schema::Identity& identity = item.source->schema;
    const auto kind = HardwareKind(item);
    if (kind != talos::launcher::HardwareRecord::kNone) {
      // Every packet decodes once HardwareProtocolCheck found none this
      // build rejects.
      if (!decode_hardware ||
          !talos::launcher::DecodeHardwareRecord(kind, item.payload, packet)
               .empty()) {
        continue;
      }
      hardware_topic = &topic;
      hardware_time_us = time_us;
      switch (kind) {
        case talos::launcher::HardwareRecord::kState:
          hardware.State(packet.state, names);
          break;
        case talos::launcher::HardwareRecord::kCommand:
          hardware.Command(packet.command, names);
          break;
        case talos::launcher::HardwareRecord::kDriverStation:
          hardware.DriverStation(packet.driver_station, joysticks_seen[topic]);
          break;
        case talos::launcher::HardwareRecord::kNone:
          break;
      }
      continue;
    }
    auto plan_at = plans.find({topic, identity.name, identity.hash});
    if (plan_at == plans.end()) {
      plan_at = plans
                    .emplace(std::tuple{topic, identity.name, identity.hash},
                             BuildPlan(topic, identity))
                    .first;
    }
    const Plan& plan = plan_at->second;
    if (!plan.ok) {
      totals.skipped.try_emplace(topic, plan.why);
      continue;
    }
    if (item.payload.size() != plan.bytes) {
      totals.skipped.try_emplace(topic,
                                 SizeMismatch(item.payload.size(), plan));
      continue;
    }
    const std::byte* data = item.payload.data();
    for (const Leaf& leaf : plan.leaves) {
      payload.clear();
      const std::size_t element_bytes = flatbuffers::GetTypeSize(leaf.base);
      for (std::size_t k = 0; k < (leaf.array ? leaf.count : 1); ++k) {
        AppendElement(leaf.base, data + leaf.offset + k * element_bytes,
                      payload);
      }
      write(leaf.name, topic, leaf.suffix, leaf.type, leaf.metadata, time_us);
    }
    for (const PoseLeaf& pose : plan.poses) {
      payload.clear();
      talos::wpilog::AppendDouble(payload, LoadDouble(data + pose.x_offset));
      talos::wpilog::AppendDouble(payload, LoadDouble(data + pose.y_offset));
      talos::wpilog::AppendDouble(payload, LoadDouble(data + pose.yaw_offset));
      write(pose.name, topic, pose.suffix, "double[]", pose_metadata, time_us);
    }
  }
  totals.entries = entries.size();
  totals.hardware_unnamed = hardware.unnamed();
  return totals;
}

// The output is written to a temporary file beside it (same directory, so
// same filesystem) and renamed over it only once complete, so a failed
// conversion leaves no partial file and any previous output unchanged.
// Created before anything is converted: an output that cannot be written
// fails first. The destructor removes the temporary file unless commit()
// renamed it into place.
class TempOutput {
 public:
  TempOutput() = default;
  TempOutput(const TempOutput&) = delete;
  TempOutput& operator=(const TempOutput&) = delete;
  ~TempOutput() {
    if (!path_.empty()) {
      std::error_code ignored;
      std::filesystem::remove(path_, ignored);
    }
  }

  // Creates the temporary file for `output`; false with `error` set when
  // `output` is something other than a regular file (a device, a
  // directory) or its directory cannot take a new file.
  bool create(const std::string& output, std::string& error) {
    std::error_code ec;
    const auto status = std::filesystem::status(output, ec);
    if (std::filesystem::exists(status) &&
        !std::filesystem::is_regular_file(status)) {
      error = "cannot write " + output + ": not a regular file";
      return false;
    }
    const std::filesystem::path target{output};
    const std::filesystem::path directory = target.has_parent_path()
                                                ? target.parent_path()
                                                : std::filesystem::path{"."};
    std::string pattern =
        (directory / ("." + target.filename().string() + ".tmp.XXXXXX"))
            .string();
    const int fd = ::mkstemp(pattern.data());
    if (fd < 0) {
      error = "cannot write " + output + ": " + std::strerror(errno);
      return false;
    }
    path_ = pattern;
    // mkstemp creates the file 0600; give it what a new file gets.
    const mode_t mask = ::umask(0);
    ::umask(mask);
    const int chmod_result =
        ::fchmod(fd, static_cast<mode_t>(0666U & ~static_cast<unsigned>(mask)));
    const int saved = errno;
    ::close(fd);
    if (chmod_result != 0) {
      error = "cannot write " + output + ": " + std::strerror(saved);
      return false;
    }
    return true;
  }

  const std::string& path() const { return path_; }

  // Renames the temporary file over `output`; false with `error` set.
  bool commit(const std::string& output, std::string& error) {
    std::error_code ec;
    std::filesystem::rename(path_, output, ec);
    if (ec) {
      error = "cannot write " + output + ": " + ec.message();
      return false;
    }
    path_.clear();
    return true;
  }

 private:
  std::string path_;
};

int Convert(const Options& options) {
  TempOutput temp;
  std::string error;
  if (!temp.create(options.output, error)) return Fail(error);

  const std::vector<std::string> files =
      talos::launcher::LogInputFiles(options.inputs);
  if (files.empty()) return Fail("no .tlog files found");

  // Each file's header: its session and loop start. A file that cannot be
  // read fails here, named; one cut while it was being created holds no
  // records and is skipped (MergeLogs warns). Damaged records inside a log
  // are skipped by the reader, and MergeLogs names the log.
  std::int64_t start_ns = std::numeric_limits<std::int64_t>::max();
  std::uint64_t session_id = 0;
  for (const auto& file : files) {
    try {
      const talos::event::log::WindowedLogReader reader{file};
      start_ns = std::min(start_ns, reader.start_time().nanos());
      session_id = reader.session_id();
    } catch (const talos::event::log::IncompleteLogHeader&) {
      continue;
    } catch (const std::exception& e) {
      return Fail(file + ": " + e.what());
    }
  }
  if (start_ns == std::numeric_limits<std::int64_t>::max()) {
    return Fail("no readable .tlog files");
  }

  std::vector<Item> items;
  std::size_t records = 0;
  if (!SelectItems(files, items, records, error)) return Fail(error);

  // Whether the session's hardware packets are of this build's protocol,
  // judged by the version its manifest records, then on every one, before
  // any is converted (hardware_record.h).
  talos::launcher::HardwareProtocolCheck protocol{
      talos::launcher::RecordedProtocolVersion(options.inputs)};
  for (const Item& item : items) {
    const auto kind = HardwareKind(item);
    if (item.values && kind != talos::launcher::HardwareRecord::kNone) {
      protocol.Add(kind, item.source->name, item.payload);
    }
  }

  // Device names for the hardware packets, as log_dump finds them: --config,
  // else the config the session's manifest names. Without one, a device's
  // entries are named by its logical id.
  const std::string config_path =
      talos::launcher::SessionConfigPath(options.config, options.inputs);
  std::optional<talos::launcher::DeviceNames> device_names;
  std::string unnamed_why =
      "no robot config (--config, or a session "
      "directory whose manifest.json names one)";
  if (!config_path.empty()) {
    std::string config_error;
    device_names = talos::launcher::LoadDeviceNames(config_path, config_error);
    unnamed_why = device_names
                      ? "a packet's config_id is not that of " + config_path
                      : "config " + config_path + ": " + config_error;
  }

  std::ofstream out{temp.path(), std::ios::binary | std::ios::trunc};
  if (!out) return Fail("cannot write " + options.output);
  Totals totals;
  try {
    talos::wpilog::Writer writer{
        out, "talos tlog_to_wpilog session_id=" + std::to_string(session_id)};
    totals = WriteItems(items, start_ns, !protocol.foreign(),
                        device_names ? &*device_names : nullptr, writer);
    out.flush();
    const bool written = writer.ok() && out.good();
    out.close();
    if (!written || out.fail()) {
      return Fail("writing " + options.output + " failed");
    }
  } catch (const std::exception& e) {
    return Fail(options.output + ": " + e.what());
  }
  if (!temp.commit(options.output, error)) return Fail(error);
  for (const auto& [topic, why] : totals.skipped) {
    std::fprintf(stderr, "tlog_to_wpilog: skipped %s: %s\n", topic.c_str(),
                 why.c_str());
  }
  if (protocol.foreign()) {
    std::fprintf(stderr, "tlog_to_wpilog: %s\n", protocol.Warning().c_str());
  }
  if (totals.hardware_unnamed) {
    std::fprintf(stderr,
                 "tlog_to_wpilog: hardware devices named by logical id: %s\n",
                 unnamed_why.c_str());
  }
  std::printf(
      "tlog_to_wpilog: %zu files, %zu records -> %zu entries, %zu "
      "values in %s\n",
      files.size(), records, totals.entries, totals.values,
      options.output.c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  std::string error;
  if (!ParseOptions(argc, argv, options, error)) {
    std::fprintf(stderr,
                 "tlog_to_wpilog: %s\nusage: %s [--config robot.toml] "
                 "<log-file-or-session-dir> [more ...] -o <out.wpilog>\n",
                 error.c_str(), argv[0]);
    return 1;
  }
  try {
    return Convert(options);
  } catch (const std::exception& e) {
    return Fail(e.what());
  }
}
