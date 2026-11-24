#include "talOS/telemetry/channels.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "flatbuffers/flatbuffers.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/names.h"
#include "talOS/schema/identity.h"
#include "talOS/schema/unit_suffix.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {

namespace {

namespace naming = introspect::naming;
static_assert(static_cast<int>(naming::AlertLevel::kNone) ==
                      Talos::Telemetry::AlertLevel_None &&
                  static_cast<int>(naming::AlertLevel::kInfo) ==
                      Talos::Telemetry::AlertLevel_Info &&
                  static_cast<int>(naming::AlertLevel::kWarning) ==
                      Talos::Telemetry::AlertLevel_Warning &&
                  static_cast<int>(naming::AlertLevel::kError) ==
                      Talos::Telemetry::AlertLevel_Error,
              "naming::AlertLevel and telemetry.fbs AlertLevel must agree");

bool TryAdd(std::size_t a, std::size_t b, std::size_t& out) {
  if (b > std::numeric_limits<std::size_t>::max() - a) return false;
  out = a + b;
  return true;
}

std::string TruncateForError(std::string_view text) {
  constexpr std::size_t kMaxErrorSegment =
      telemetry_wire::kMaxDictionaryStringBytes;
  std::string out;
  const std::size_t n = std::min(text.size(), kMaxErrorSegment);
  out.reserve(n + (text.size() > kMaxErrorSegment ? 3 : 0));
  for (std::size_t i = 0; i < n; ++i) {
    out.push_back(text[i] == '\0' ? '?' : text[i]);
  }
  if (text.size() > kMaxErrorSegment) out += "...";
  return out;
}

// Strips at most one leading '/' from `topic` and rejects empty or malformed
// topic names (empty result, embedded NUL, or empty path segments).
std::string_view NormalizeTopicPrefix(std::string_view topic) {
  if (topic.find('\0') != std::string_view::npos) {
    throw std::invalid_argument("telemetry: malformed topic name '" +
                                TruncateForError(topic) + "'");
  }
  std::string_view prefix = topic;
  if (prefix.starts_with('/')) prefix.remove_prefix(1);
  if (prefix.empty() || prefix.starts_with('/') || prefix.ends_with('/') ||
      prefix.find("//") != std::string_view::npos) {
    throw std::invalid_argument("telemetry: invalid topic name '" +
                                TruncateForError(topic) + "'");
  }
  return prefix;
}

// Whether a normalized (slash-stripped) topic prefix has role `status`.
bool IsStatusTopicPrefix(std::string_view prefix) {
  const std::size_t slash = prefix.find('/');
  if (slash == std::string_view::npos) return false;
  const std::size_t rest = slash + 1;
  const std::size_t next = prefix.find('/', rest);
  const std::string_view role = next == std::string_view::npos
                                    ? prefix.substr(rest)
                                    : prefix.substr(rest, next - rest);
  return role == "status";
}

// Computes the metadata bytes (vector entries plus label text) that
// EnumLabels(schema, enum_index) would hold, without allocating. Returns false
// on std::size_t overflow.
bool MeasureEnumLabels(const reflection::Schema& schema,
                       std::int32_t enum_index, std::size_t& out_bytes) {
  out_bytes = 0;
  if (schema.enums() == nullptr || enum_index < 0 ||
      static_cast<std::size_t>(enum_index) >= schema.enums()->size()) {
    return true;
  }
  const auto* enum_def =
      schema.enums()->Get(static_cast<flatbuffers::uoffset_t>(enum_index));
  if (enum_def == nullptr || enum_def->values() == nullptr) return true;
  std::int64_t max_value = -1;
  for (const auto* value : *enum_def->values()) {
    if (value != nullptr) max_value = std::max(max_value, value->value());
  }
  if (max_value < 0 ||
      max_value >= static_cast<std::int64_t>(telemetry_wire::kMaxEnumLabels)) {
    return true;
  }
  const std::size_t count = static_cast<std::size_t>(max_value) + 1;
  std::array<std::size_t, telemetry_wire::kMaxEnumLabels> label_lens{};
  for (const auto* value : *enum_def->values()) {
    if (value != nullptr && value->name() != nullptr && value->value() >= 0) {
      label_lens[static_cast<std::size_t>(value->value())] =
          value->name()->size();
    }
  }
  std::size_t total = count * sizeof(std::string);
  for (std::size_t i = 0; i < count; ++i) {
    if (!TryAdd(total, label_lens[i], total)) return false;
  }
  out_bytes = total;
  return true;
}

// One topic's flattening, bounded against a schema that is damaged or
// crafted (a recorded log's, which nothing checked against this build).
class Flattener {
 public:
  Flattener(const reflection::Schema& schema, std::string_view topic,
            std::string_view prefix, const FlattenOptions& options,
            std::vector<Channel>& channels)
      : schema_{schema},
        topic_{topic},
        is_status_topic_{IsStatusTopicPrefix(prefix)},
        options_{options},
        channels_{channels},
        open_(schema.objects()->size()) {}

  // Every channel kept, those past a truncating flattening's cap included.
  std::size_t kept() const { return kept_; }

  // Flattens object `index` (reached through `field`) at `base_offset`.
  void Struct(std::size_t index, std::string_view field,
              std::string_view prefix, std::size_t base_offset,
              std::size_t depth) {
    if (depth > kMaxNestingDepth) {
      Refuse(field, "nests structs more than " +
                        std::to_string(kMaxNestingDepth) + " deep");
    }
    if (open_[index]) Refuse(field, "contains its own struct");
    if (++structs_ > kMaxTopicStructs) {
      Refuse(field, "expands to more than " + std::to_string(kMaxTopicStructs) +
                        " struct instances");
    }
    const reflection::Object* object_ptr =
        schema_.objects()->Get(static_cast<flatbuffers::uoffset_t>(index));
    if (object_ptr == nullptr || !object_ptr->is_struct() ||
        object_ptr->bytesize() <= 0 || object_ptr->fields() == nullptr) {
      Refuse(field, "is not a positive-sized struct");
    }
    const reflection::Object& object = *object_ptr;
    const auto bytes = static_cast<std::size_t>(object.bytesize());
    const std::size_t max_remaining_fields =
        (kMaxTopicStructs - structs_) + (kMaxTopicElements - elements_);
    if (object.fields()->size() > max_remaining_fields) {
      Refuse(field, "exceeds per-topic field expansion bounds");
    }
    open_[index] = true;
    std::vector<const reflection::Field*> fields;
    fields.reserve(object.fields()->size());
    for (const auto* f : *object.fields()) {
      if (f == nullptr || f->name() == nullptr || f->type() == nullptr) {
        Refuse(field, "has incomplete field metadata");
      }
      fields.push_back(f);
    }
    std::sort(fields.begin(), fields.end(),
              [](const reflection::Field* a, const reflection::Field* b) {
                return a->id() < b->id();
              });

    for (const auto* field_def : fields) {
      const std::string_view field_name = field_def->name()->string_view();
      if (field_name.empty() ||
          field_name.find('/') != std::string_view::npos ||
          field_name.find('\0') != std::string_view::npos) {
        Refuse(field_name, "has empty or malformed name");
      }
      if (field_name.size() >= options_.max_metadata_bytes ||
          prefix.size() > options_.max_metadata_bytes - 1 - field_name.size()) {
        RefuseMetadataLimit(field_name);
      }
      std::string channel_name;
      channel_name.reserve(prefix.size() + 1 + field_name.size());
      channel_name.append(prefix);
      channel_name.push_back('/');
      channel_name.append(field_name);
      const auto& type = *field_def->type();
      if (field_def->offset() > bytes ||
          field_def->offset() >
              std::numeric_limits<std::size_t>::max() - base_offset) {
        Refuse(field_name, "offset overflows");
      }
      const std::size_t field_offset = base_offset + field_def->offset();

      if (type.base_type() == reflection::BaseType::Obj) {
        const std::size_t nested_idx = ObjectIndex(type.index(), field_name);
        const reflection::Object* nested_obj = schema_.objects()->Get(
            static_cast<flatbuffers::uoffset_t>(nested_idx));
        if (nested_obj == nullptr || !nested_obj->is_struct() ||
            nested_obj->bytesize() <= 0) {
          Refuse(field_name, "has non-positive struct size");
        }
        const auto nested_bytes =
            static_cast<std::size_t>(nested_obj->bytesize());
        if (nested_bytes > bytes - field_def->offset() ||
            nested_bytes >
                std::numeric_limits<std::size_t>::max() - field_offset) {
          Refuse(field_name, "exceeds enclosing struct bounds");
        }
        Struct(nested_idx, field_name, channel_name, field_offset, depth + 1);
        continue;
      }

      if (type.base_type() == reflection::BaseType::Array) {
        const std::size_t count = type.fixed_length();
        if (count == 0) {
          Refuse(field_name, "has zero array length");
        }
        const bool nested = type.element() == reflection::BaseType::Obj;
        const std::size_t element =
            nested ? ObjectIndex(type.index(), field_name) : 0;
        std::size_t element_bytes = 0;
        if (nested) {
          const reflection::Object* elem_obj = schema_.objects()->Get(
              static_cast<flatbuffers::uoffset_t>(element));
          if (elem_obj == nullptr || !elem_obj->is_struct() ||
              elem_obj->bytesize() <= 0) {
            Refuse(field_name, "has non-positive struct element size");
          }
          element_bytes = static_cast<std::size_t>(elem_obj->bytesize());
        } else {
          element_bytes = schema::ScalarSize(type.element());
          if (element_bytes == 0) {
            Refuse(field_name, "has non-scalar array element type");
          }
        }
        if (element_bytes > (bytes - field_def->offset()) / count ||
            element_bytes >
                (std::numeric_limits<std::size_t>::max() - field_offset) /
                    count) {
          Refuse(field_name, "array offset overflows");
        }
        if (nested && count > kMaxTopicStructs - structs_) {
          Refuse(field_name, "expands to more than " +
                                 std::to_string(kMaxTopicStructs) +
                                 " struct instances");
        }
        if (!nested && count > kMaxTopicElements - elements_) {
          Refuse(field_name, "expands to more than " +
                                 std::to_string(kMaxTopicElements) +
                                 " scalar elements");
        }
        for (std::size_t i = 0; i < count; ++i) {
          const std::string index_str = std::to_string(i);
          if (index_str.size() >= options_.max_metadata_bytes ||
              channel_name.size() >
                  options_.max_metadata_bytes - 1 - index_str.size()) {
            RefuseMetadataLimit(field_name);
          }
          std::string element_name;
          element_name.reserve(channel_name.size() + 1 + index_str.size());
          element_name.append(channel_name);
          element_name.push_back('/');
          element_name.append(index_str);
          const std::size_t element_offset = field_offset + i * element_bytes;
          if (nested) {
            Struct(element, field_name, element_name, element_offset,
                   depth + 1);
          } else {
            AddScalar(type.element(), type.index(), field_name,
                      std::move(element_name), element_offset, false);
          }
        }
        continue;
      }

      const std::size_t scalar_bytes = schema::ScalarSize(type.base_type());
      if (scalar_bytes == 0) {
        Refuse(field_name, "has non-scalar field type");
      }
      if (scalar_bytes > bytes - field_def->offset() ||
          scalar_bytes >
              std::numeric_limits<std::size_t>::max() - field_offset) {
        Refuse(field_name, "exceeds enclosing struct bounds");
      }
      AddScalar(type.base_type(), type.index(), field_name,
                std::move(channel_name), field_offset, true);
    }
    open_[index] = false;
  }

 private:
  [[noreturn]] void Refuse(std::string_view field, std::string_view why) const {
    throw std::invalid_argument(
        "telemetry: schema for topic '" + TruncateForError(topic_) +
        "': field '" + TruncateForError(field) + "' " + std::string{why});
  }

  [[noreturn]] void RefuseMetadataLimit(std::string_view field) const {
    Refuse(field, "exceeds the topic's " +
                      std::to_string(options_.max_metadata_bytes) +
                      "-byte metadata limit");
  }

  // The index of object `index`, refusing one the schema does not hold.
  std::size_t ObjectIndex(std::int32_t index, std::string_view field) const {
    if (index < 0 ||
        static_cast<std::size_t>(index) >= schema_.objects()->size()) {
      Refuse(field, "names no object");
    }
    return static_cast<std::size_t>(index);
  }

  // `whole_field`: the scalar is the field itself, not an array element,
  // so a bool may be an alert.
  void AddScalar(reflection::BaseType base, std::int32_t type_index,
                 std::string_view field_name, std::string name,
                 std::size_t offset, bool whole_field) {
    if (++elements_ > kMaxTopicElements) {
      Refuse(field_name, "expands to more than " +
                             std::to_string(kMaxTopicElements) +
                             " scalar elements");
    }
    if (options_.keep && !options_.keep(name)) return;
    ++kept_;
    if (channels_.size() >= options_.max_channels) {
      // Counted, not kept: the walk is bounded by kMaxTopicStructs and
      // kMaxTopicElements.
      if (options_.truncate) return;
      Refuse(field_name, "is past the topic's " +
                             std::to_string(options_.max_channels) +
                             " channels");
    }
    const ChannelKind kind = KindOf(base);
    const bool is_enum = base != reflection::BaseType::Bool &&
                         type_index >= 0 &&
                         (kind == Talos::Telemetry::ChannelKind_Int ||
                          kind == Talos::Telemetry::ChannelKind_UInt);
    const std::string_view unit =
        (base != reflection::BaseType::Bool && !is_enum)
            ? schema::unit_for_field(field_name)
            : std::string_view{};
    std::size_t label_bytes = 0;
    if (is_enum && !MeasureEnumLabels(schema_, type_index, label_bytes)) {
      RefuseMetadataLimit(field_name);
    }
    std::size_t channel_bytes = 0;
    if (!TryAdd(sizeof(Channel), name.size(), channel_bytes) ||
        !TryAdd(channel_bytes, unit.size(), channel_bytes) ||
        !TryAdd(channel_bytes, label_bytes, channel_bytes) ||
        channel_bytes > options_.max_metadata_bytes - metadata_bytes_) {
      RefuseMetadataLimit(field_name);
    }
    metadata_bytes_ += channel_bytes;

    Channel channel;
    channel.name = std::move(name);
    channel.kind = kind;
    channel.offset = offset;
    channel.base = base;
    if (base == reflection::BaseType::Bool) {
      // A bool has no unit. Only a `status` topic carries alerts
      // (NAMING.md), as session_report reads them; elsewhere an
      // alert-shaped bool is a plain bool.
      if (whole_field && IsStatusTopic()) {
        channel.alert = AlertLevelOf(field_name);
      }
    } else if (is_enum) {
      channel.labels = EnumLabels(schema_, type_index);
    } else {
      channel.unit = std::string{unit};
    }
    channels_.push_back(std::move(channel));
  }

  bool IsStatusTopic() const { return is_status_topic_; }

  const reflection::Schema& schema_;
  std::string_view topic_;
  bool is_status_topic_{false};
  const FlattenOptions& options_;
  std::vector<Channel>& channels_;
  std::size_t kept_{0};
  std::vector<bool> open_;  // by object index: being flattened now
  std::size_t structs_{0};
  std::size_t elements_{0};
  std::size_t metadata_bytes_{0};
};

Channel Unread(std::string name, ChannelKind kind, std::string unit = "",
               std::vector<std::string> labels = {}) {
  Channel channel;
  channel.name = std::move(name);
  channel.unit = std::move(unit);
  channel.labels = std::move(labels);
  channel.kind = kind;
  return channel;
}

}  // namespace

AlertLevel AlertLevelOf(std::string_view field) {
  const auto alert = naming::ParseAlertField(field);
  return alert ? static_cast<AlertLevel>(alert->level)
               : Talos::Telemetry::AlertLevel_None;
}

std::vector<std::string> EnumLabels(const reflection::Schema& schema,
                                    std::int32_t enum_index) {
  std::vector<std::string> labels;
  if (schema.enums() != nullptr && enum_index >= 0 &&
      static_cast<std::size_t>(enum_index) < schema.enums()->size()) {
    const auto* enum_def =
        schema.enums()->Get(static_cast<flatbuffers::uoffset_t>(enum_index));
    if (enum_def == nullptr || enum_def->values() == nullptr) return labels;
    std::int64_t max_value = -1;
    for (const auto* value : *enum_def->values()) {
      if (value != nullptr) max_value = std::max(max_value, value->value());
    }
    if (max_value >= 0 &&
        max_value < static_cast<std::int64_t>(telemetry_wire::kMaxEnumLabels)) {
      labels.resize(static_cast<std::size_t>(max_value) + 1);
      for (const auto* value : *enum_def->values()) {
        if (value != nullptr && value->name() != nullptr &&
            value->value() >= 0) {
          labels[static_cast<std::size_t>(value->value())] =
              value->name()->str();
        }
      }
    }
  }
  return labels;
}

std::vector<Channel> FlattenTopic(std::string_view topic,
                                  std::string_view schema_name,
                                  std::string_view bfbs,
                                  const FlattenOptions& options,
                                  std::size_t* kept) {
  const std::string_view prefix = NormalizeTopicPrefix(topic);
  if (prefix.size() > options.max_metadata_bytes) {
    throw std::invalid_argument(
        "telemetry: schema for topic '" + TruncateForError(topic) +
        "': exceeds the topic's " + std::to_string(options.max_metadata_bytes) +
        "-byte metadata limit");
  }
  const reflection::Schema* schema = schema::VerifySchema(bfbs);
  if (schema == nullptr) {
    throw std::invalid_argument(
        "telemetry: failed to verify schema for topic '" +
        TruncateForError(topic) + "'");
  }
  if (schema_name.empty() || schema_name.find('\0') != std::string_view::npos ||
      schema_name.size() > bfbs.size()) {
    throw std::invalid_argument(
        "telemetry: root schema object '" + TruncateForError(schema_name) +
        "' for topic '" + TruncateForError(topic) + "' is missing");
  }
  const reflection::Object* object =
      schema->objects() != nullptr
          ? schema->objects()->LookupByKey(std::string{schema_name}.c_str())
          : nullptr;
  if (object == nullptr || object->name() == nullptr ||
      object->name()->string_view() != schema_name) {
    throw std::invalid_argument(
        "telemetry: root schema object '" + TruncateForError(schema_name) +
        "' for topic '" + TruncateForError(topic) + "' is missing");
  }
  std::vector<Channel> channels;
  const auto* objects = schema->objects();
  std::size_t root = 0;
  while (root < objects->size() &&
         objects->Get(static_cast<flatbuffers::uoffset_t>(root)) != object) {
    ++root;
  }
  if (root >= objects->size()) {
    throw std::invalid_argument(
        "telemetry: root schema object '" + TruncateForError(schema_name) +
        "' for topic '" + TruncateForError(topic) + "' is missing");
  }
  Flattener flattener{*schema, topic, prefix, options, channels};
  flattener.Struct(root, schema_name, prefix, 0, 0);
  if (kept != nullptr) *kept = flattener.kept();
  return channels;
}

std::vector<Channel> EnvelopeChannels(std::string_view topic,
                                      std::string_view inner,
                                      std::string_view bfbs) {
  const std::string_view prefix = NormalizeTopicPrefix(topic);
  constexpr std::string_view kSinceSuffix = "/mode_since_us";
  constexpr std::string_view kActiveSuffix = "/active";
  const std::string_view since_unit = schema::unit_for_field("mode_since_us");
  std::size_t header_bytes = 0;
  if (!TryAdd(sizeof(Channel), prefix.size(), header_bytes) ||
      !TryAdd(header_bytes, kSinceSuffix.size(), header_bytes) ||
      !TryAdd(header_bytes, since_unit.size(), header_bytes) ||
      !TryAdd(header_bytes, sizeof(Channel), header_bytes) ||
      !TryAdd(header_bytes, prefix.size(), header_bytes) ||
      !TryAdd(header_bytes, kActiveSuffix.size(), header_bytes) ||
      header_bytes > kMaxTopicMetadataBytes) {
    throw std::invalid_argument(
        "telemetry: schema for topic '" + TruncateForError(topic) +
        "': exceeds the topic's " + std::to_string(kMaxTopicMetadataBytes) +
        "-byte metadata limit");
  }
  std::vector<Channel> channels;
  Channel since;
  since.name.reserve(prefix.size() + kSinceSuffix.size());
  since.name.append(prefix);
  since.name.append(kSinceSuffix);
  since.unit = std::string{since_unit};
  since.kind = Talos::Telemetry::ChannelKind_UInt;
  since.offset = 0;
  since.base = reflection::BaseType::ULong;
  channels.push_back(std::move(since));
  Channel active;
  active.name.reserve(prefix.size() + kActiveSuffix.size());
  active.name.append(prefix);
  active.name.append(kActiveSuffix);
  active.kind = Talos::Telemetry::ChannelKind_Bool;
  active.offset = schema::kEnvelopeActiveOffset;
  active.base = reflection::BaseType::Bool;
  channels.push_back(std::move(active));
  FlattenOptions options;
  options.max_channels = kMaxTopicChannels - channels.size();
  options.max_metadata_bytes = kMaxTopicMetadataBytes - header_bytes;
  std::string inner_topic;
  inner_topic.reserve(topic.size() + std::string_view{"/message"}.size());
  inner_topic.append(topic);
  inner_topic.append("/message");
  for (Channel& channel : FlattenTopic(inner_topic, inner, bfbs, options)) {
    const std::size_t scalar_bytes = schema::ScalarSize(channel.base);
    if (scalar_bytes == 0 ||
        channel.offset > std::numeric_limits<std::size_t>::max() -
                             schema::kEnvelopeHeaderBytes ||
        channel.offset + schema::kEnvelopeHeaderBytes >
            std::numeric_limits<std::size_t>::max() - scalar_bytes) {
      throw std::invalid_argument("telemetry: schema for topic '" +
                                  TruncateForError(topic) +
                                  "': envelope offset overflows");
    }
    channel.offset += schema::kEnvelopeHeaderBytes;
    channels.push_back(std::move(channel));
  }
  return channels;
}

std::vector<Channel> DriverStationChannels() {
  constexpr auto kBool = Talos::Telemetry::ChannelKind_Bool;
  constexpr auto kUInt = Talos::Telemetry::ChannelKind_UInt;
  const std::string prefix = "hw/state/driver_station/";
  return {
      Unread(prefix + "connected", kBool),
      Unread(prefix + "enabled", kBool),
      Unread(prefix + "autonomous", kBool),
      Unread(prefix + "teleop", kBool),
      Unread(prefix + "test", kBool),
      Unread(prefix + "estop", kBool),
      Unread(prefix + "fms_attached", kBool),
      Unread(prefix + "alliance", kUInt, "", {"unknown", "red", "blue"}),
      Unread(prefix + "match_time_s", Talos::Telemetry::ChannelKind_Double,
             "s"),
      Unread(prefix + "match_number", kUInt),
      Unread(prefix + "match_type", kUInt, "",
             {"none", "practice", "qualification", "elimination"}),
      Unread(prefix + "station", kUInt),
  };
}

bool ReadDriverStation(
    std::span<const std::byte> packet,
    std::array<std::uint64_t, kDriverStationChannelCount>& raw) {
  // hardware::Packet's bytes(), read without assuming `packet` is aligned.
  constexpr std::size_t kData = offsetof(hardware::Packet, data);
  if (packet.size() < kData) return false;
  std::uint32_t size = 0;
  std::memcpy(&size, packet.data(), sizeof(size));
  if (size > sizeof(hardware::Packet::data) || packet.size() - kData < size) {
    return false;
  }
  const std::span<const std::uint8_t> bytes{
      reinterpret_cast<const std::uint8_t*>(packet.data() + kData), size};
  driver_station::DriverStationData data;
  if (!driver_station::Decode(bytes, data)) return false;
  const std::uint32_t flags = data.fms.flags;
  const auto flag = [flags](std::uint32_t bit) -> std::uint64_t {
    return (flags & bit) != 0 ? 1 : 0;
  };
  raw = {
      flag(driver_station::kDsAttached),
      flag(driver_station::kEnabled),
      flag(driver_station::kAutonomous),
      flag(driver_station::kTeleop),
      flag(driver_station::kTest),
      flag(driver_station::kEStop),
      flag(driver_station::kFmsAttached),
      static_cast<std::uint8_t>(data.fms.alliance),
      RawFromDouble(data.fms.match_time_s),
      data.fms.match_number,
      static_cast<std::uint8_t>(data.fms.match_type),
      data.fms.station,
  };
  return true;
}

}  // namespace talos::telemetry
