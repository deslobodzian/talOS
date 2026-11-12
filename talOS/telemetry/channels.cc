#include "talOS/telemetry/channels.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include "flatbuffers/flatbuffers.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/schema/unit_suffix.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {

namespace {

void AddScalar(const reflection::Schema& schema, reflection::BaseType base,
               std::int32_t type_index, std::string_view field_name,
               std::string name, std::size_t offset,
               std::vector<Channel>& channels) {
  Channel channel;
  channel.name = std::move(name);
  channel.kind = KindOf(base);
  channel.offset = offset;
  channel.base = base;
  if (base == reflection::BaseType::Bool) {
    // A bool has no unit.
  } else if (type_index >= 0) {
    channel.labels = EnumLabels(schema, type_index);
  } else {
    channel.unit = std::string{schema::unit_for_field(field_name)};
  }
  channels.push_back(std::move(channel));
}

void FlattenStruct(const reflection::Schema& schema,
                   const reflection::Object& object, std::string_view prefix,
                   std::size_t base_offset, std::vector<Channel>& channels) {
  std::vector<const reflection::Field*> fields(object.fields()->begin(),
                                               object.fields()->end());
  std::sort(fields.begin(), fields.end(),
            [](const reflection::Field* a, const reflection::Field* b) {
              return a->id() < b->id();
            });

  for (const auto* field : fields) {
    const std::string field_name = field->name()->str();
    const std::string channel_name = std::string{prefix} + "/" + field_name;
    const auto& type = *field->type();
    const std::size_t field_offset = base_offset + field->offset();

    if (type.base_type() == reflection::BaseType::Obj) {
      const auto* nested = schema.objects()->Get(type.index());
      if (nested != nullptr) {
        FlattenStruct(schema, *nested, channel_name, field_offset, channels);
      }
      continue;
    }

    if (type.base_type() == reflection::BaseType::Array) {
      const std::size_t count = type.fixed_length();
      const std::size_t element_bytes =
          type.element() == reflection::BaseType::Obj
              ? schema.objects()->Get(type.index())->bytesize()
              : flatbuffers::GetTypeSize(type.element());
      for (std::size_t i = 0; i < count; ++i) {
        const std::string element_name = channel_name + "/" + std::to_string(i);
        const std::size_t element_offset = field_offset + i * element_bytes;
        if (type.element() == reflection::BaseType::Obj) {
          const auto* nested = schema.objects()->Get(type.index());
          if (nested != nullptr) {
            FlattenStruct(schema, *nested, element_name, element_offset,
                          channels);
          }
        } else {
          AddScalar(schema, type.element(), type.index(), field_name,
                    element_name, element_offset, channels);
        }
      }
      continue;
    }

    AddScalar(schema, type.base_type(), type.index(), field_name, channel_name,
              field_offset, channels);
  }
}

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

std::vector<std::string> EnumLabels(const reflection::Schema& schema,
                                    std::int32_t enum_index) {
  std::vector<std::string> labels;
  if (enum_index >= 0 &&
      enum_index < static_cast<std::int32_t>(schema.enums()->size())) {
    const auto* enum_def = schema.enums()->Get(enum_index);
    std::int64_t max_value = -1;
    for (const auto* value : *enum_def->values()) {
      max_value = std::max(max_value, value->value());
    }
    if (max_value >= 0 &&
        max_value < static_cast<std::int64_t>(telemetry_wire::kMaxEnumLabels)) {
      labels.resize(static_cast<std::size_t>(max_value) + 1);
      for (const auto* value : *enum_def->values()) {
        if (value->value() >= 0) {
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
                                  std::string_view bfbs) {
  const reflection::Schema* schema = schema::VerifySchema(bfbs);
  if (schema == nullptr) {
    throw std::invalid_argument(
        "telemetry: failed to verify schema for topic '" + std::string{topic} +
        "'");
  }
  const reflection::Object* object =
      schema->objects()->LookupByKey(std::string{schema_name}.c_str());
  if (object == nullptr) {
    throw std::invalid_argument("telemetry: root schema object '" +
                                std::string{schema_name} + "' for topic '" +
                                std::string{topic} + "' is missing");
  }
  std::string_view prefix = topic;
  if (prefix.starts_with('/')) prefix.remove_prefix(1);
  std::vector<Channel> channels;
  FlattenStruct(*schema, *object, prefix, 0, channels);
  return channels;
}

std::vector<Channel> DriverStationChannels() {
  constexpr auto kBool = Talos::Telemetry::ChannelKind_Bool;
  const std::string prefix = "hw/state/driver_station/";
  return {
      Unread(prefix + "connected", kBool),
      Unread(prefix + "enabled", kBool),
      Unread(prefix + "autonomous", kBool),
      Unread(prefix + "teleop", kBool),
      Unread(prefix + "test", kBool),
      Unread(prefix + "estop", kBool),
      Unread(prefix + "fms_attached", kBool),
      Unread(prefix + "alliance", Talos::Telemetry::ChannelKind_UInt, "",
             {"unknown", "red", "blue"}),
      Unread(prefix + "match_time_s", Talos::Telemetry::ChannelKind_Double,
             "s"),
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
  std::span<const std::uint8_t> bytes;
  if (size <= sizeof(hardware::Packet::data)) {
    if (packet.size() - kData < size) return false;
    bytes = {reinterpret_cast<const std::uint8_t*>(packet.data() + kData),
             size};
  }
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
  };
  return true;
}

}  // namespace talos::telemetry
