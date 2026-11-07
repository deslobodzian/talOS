#pragma once

#include <fnmatch.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection.h"
#include "flatbuffers/verifier.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/node/access.h"
#include "talOS/node/node.h"
#include "talOS/schema/identity.h"
#include "talOS/schema/unit_suffix.h"
#include "talOS/telemetry/telemetry_generated.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {

// One dictionary entry, in wire order.
struct ChannelDef {
  std::string name;
  std::string unit;
  std::vector<std::string> labels;  // non-empty: enum channel
};

// FNV-1a 32 over names, units and labels in dictionary order, with NUL
// separators between fields and 0xFF between channels.
inline std::uint32_t DictionaryId(const std::vector<ChannelDef>& dictionary) {
  std::uint32_t hash = 2166136261u;
  auto mix = [&hash](std::string_view text) {
    for (const char c : text) {
      hash ^= static_cast<std::uint8_t>(c);
      hash *= 16777619u;
    }
  };
  auto separator = [&hash](std::uint8_t byte) {
    hash ^= byte;
    hash *= 16777619u;
  };
  for (const ChannelDef& channel : dictionary) {
    mix(channel.name);
    separator(0);
    mix(channel.unit);
    separator(0);
    for (const std::string& label : channel.labels) {
      mix(label);
      separator(0);
    }
    separator(0xff);
  }
  return hash;
}

// Size in bytes (prefix included) of a value-only frame with every channel
// present. FlatBuffers vector size does not depend on the values, so this
// is the worst case the construction-time budget check measures.
inline std::size_t MeasureValueFrameBytes(std::size_t channel_count) {
  flatbuffers::FlatBufferBuilder builder{1024};
  const std::vector<double> values(channel_count, 0.0);
  const auto values_offset = builder.CreateVector(values.data(), values.size());
  const Talos::Telemetry::Pose chassis{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Talos::Telemetry::FinishSizePrefixedFrameBuffer(
      builder, Talos::Telemetry::CreateFrame(builder, /*stream_id=*/0,
                                             /*sequence_id=*/0,
                                             /*timestamp_ns=*/0,
                                             /*dictionary_id=*/0, &chassis,
                                             values_offset,
                                             /*dictionary=*/0));
  return builder.GetSize();
}

namespace detail {

template <class T>
inline T Read(const std::uint8_t* data) {
  T value;
  std::memcpy(&value, data, sizeof value);
  return value;
}

inline double ReadScalarAsDouble(reflection::BaseType base,
                                 const std::uint8_t* data) {
  using reflection::BaseType;
  switch (base) {
    case BaseType::Bool:
      return Read<std::uint8_t>(data) != 0 ? 1.0 : 0.0;
    case BaseType::UByte:
    case BaseType::UType:
      return static_cast<double>(Read<std::uint8_t>(data));
    case BaseType::Byte:
      return static_cast<double>(Read<std::int8_t>(data));
    case BaseType::Short:
      return static_cast<double>(Read<std::int16_t>(data));
    case BaseType::UShort:
      return static_cast<double>(Read<std::uint16_t>(data));
    case BaseType::Int:
      return static_cast<double>(Read<std::int32_t>(data));
    case BaseType::UInt:
      return static_cast<double>(Read<std::uint32_t>(data));
    case BaseType::Long:
      return static_cast<double>(Read<std::int64_t>(data));
    case BaseType::ULong:
      return static_cast<double>(Read<std::uint64_t>(data));
    case BaseType::Float:
      return static_cast<double>(Read<float>(data));
    case BaseType::Double:
      return Read<double>(data);
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

inline std::vector<std::string> ExtractEnumLabels(
    const reflection::Schema& schema, std::int32_t enum_index) {
  std::vector<std::string> labels;
  if (enum_index >= 0 &&
      enum_index < static_cast<std::int32_t>(schema.enums()->size())) {
    const auto* enum_def = schema.enums()->Get(enum_index);
    std::int64_t max_val = -1;
    for (const auto* val : *enum_def->values()) {
      if (val->value() > max_val) max_val = val->value();
    }
    if (max_val >= 0 && max_val < 1024) {
      labels.resize(max_val + 1);
      for (const auto* val : *enum_def->values()) {
        if (val->value() >= 0) {
          labels[val->value()] = val->name()->str();
        }
      }
    }
  }
  return labels;
}

inline bool MatchesAnyGlob(std::string_view name,
                           const std::vector<std::string>& patterns) {
  const std::string unslashed{
      !name.empty() && name.front() == '/' ? name.substr(1) : name};
  const std::string full_str = "/" + unslashed;
  for (const auto& pattern : patterns) {
    if (::fnmatch(pattern.c_str(), full_str.c_str(), 0) == 0 ||
        ::fnmatch(pattern.c_str(), unslashed.c_str(), 0) == 0) {
      return true;
    }
  }
  return false;
}

inline bool MatchesGlobs(std::string_view topic,
                         const std::vector<std::string>& includes,
                         const std::vector<std::string>& excludes) {
  return !MatchesAnyGlob(topic, excludes) && MatchesAnyGlob(topic, includes);
}

}  // namespace detail

class Telemetry {
 public:
  explicit Telemetry(Node& node)
      : node_{node},
        stale_after_{std::chrono::microseconds{
            node.param<std::int64_t>("stale_timeout_us", 100'000)}},
        include_globs_{node.param<std::vector<std::string>>(
            "include", std::vector<std::string>{"*"})},
        exclude_globs_{node.param<std::vector<std::string>>(
            "exclude", std::vector<std::string>{})},
        topic_{ResolveTopic(node)},
        pose_topic_{ResolvePoseTopic(node)},
        studio_{node.publish_bytes(topic_, telemetry_wire::slot_bytes,
                                   {.slots = telemetry_wire::slot_count,
                                    .drop_newest = true,
                                    .external = true})} {
    using talos::detail::NodeAccess;

    idx_pose_valid_ = AddChannel("pose/valid");

    const introspect::DeclaredGraph* graph = NodeAccess::declared_graph(node);
    if (graph != nullptr) {
      BuildFromGraph(*graph);
    }

    dictionary_id_ = DictionaryId(dictionary_);
    stream_id_ = RandomStreamId();
    values_.assign(dictionary_.size(),
                   std::numeric_limits<double>::quiet_NaN());

    CheckFrameBudget();
  }

  std::uint64_t stream_id() const { return stream_id_; }
  std::uint32_t dictionary_id() const { return dictionary_id_; }
  const std::vector<ChannelDef>& dictionary() const { return dictionary_; }

  void update() {
    using talos::detail::NodeAccess;

    bool pose_ok = false;
    Talos::Telemetry::Pose chassis{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (has_pose_) {
      const bool fresh =
          NodeAccess::subscription_fresh(node_, pose_sub_index_, stale_after_);
      if (fresh) {
        const auto* raw = static_cast<const std::uint8_t*>(
            NodeAccess::subscription_value(node_, pose_sub_index_));
        bool valid = true;
        if (pose_has_valid_field_) {
          valid = *(raw + pose_valid_offset_) != 0;
        }
        double x_m = 0.0;
        double y_m = 0.0;
        double yaw_rad = 0.0;
        std::memcpy(&x_m, raw + pose_x_offset_, sizeof(double));
        std::memcpy(&y_m, raw + pose_y_offset_, sizeof(double));
        std::memcpy(&yaw_rad, raw + pose_yaw_offset_, sizeof(double));
        if (valid && std::isfinite(x_m) && std::isfinite(y_m) &&
            std::isfinite(yaw_rad)) {
          pose_ok = true;
          chassis = {x_m, y_m, 0.0, 0.0, 0.0, yaw_rad};
        }
      }
    }

    bool any_source_fresh = pose_ok;
    for (const auto& topic : observed_topics_) {
      if (NodeAccess::subscription_fresh(node_, topic.sub_index,
                                         stale_after_)) {
        any_source_fresh = true;
        break;
      }
    }

    if (!any_source_fresh) return;

    std::fill(values_.begin(), values_.end(),
              std::numeric_limits<double>::quiet_NaN());

    SetValue(idx_pose_valid_, pose_ok ? 1.0 : 0.0);

    for (const auto& topic : observed_topics_) {
      const bool fresh =
          NodeAccess::subscription_fresh(node_, topic.sub_index, stale_after_);
      if (!fresh) continue;

      const auto* raw = static_cast<const std::uint8_t*>(
          NodeAccess::subscription_value(node_, topic.sub_index));

      if (topic.is_driver_station) {
        const auto* packet = reinterpret_cast<const hardware::Packet*>(raw);
        driver_station::DriverStationData ds_data;
        if (driver_station::Decode(packet->bytes(), ds_data)) {
          const uint32_t flags = ds_data.fms.flags;
          const bool connected = (flags & driver_station::kDsAttached) != 0;
          if (topic.ds_channels.connected < dictionary_.size()) {
            SetValue(topic.ds_channels.connected, connected ? 1.0 : 0.0);
          }
          if (topic.ds_channels.enabled < dictionary_.size()) {
            SetValue(topic.ds_channels.enabled,
                     (flags & driver_station::kEnabled) != 0 ? 1.0 : 0.0);
          }
          if (topic.ds_channels.autonomous < dictionary_.size()) {
            SetValue(topic.ds_channels.autonomous,
                     (flags & driver_station::kAutonomous) != 0 ? 1.0 : 0.0);
          }
          if (topic.ds_channels.teleop < dictionary_.size()) {
            SetValue(topic.ds_channels.teleop,
                     (flags & driver_station::kTeleop) != 0 ? 1.0 : 0.0);
          }
          if (topic.ds_channels.test < dictionary_.size()) {
            SetValue(topic.ds_channels.test,
                     (flags & driver_station::kTest) != 0 ? 1.0 : 0.0);
          }
          if (topic.ds_channels.estop < dictionary_.size()) {
            SetValue(topic.ds_channels.estop,
                     (flags & driver_station::kEStop) != 0 ? 1.0 : 0.0);
          }
          if (topic.ds_channels.fms_attached < dictionary_.size()) {
            SetValue(topic.ds_channels.fms_attached,
                     (flags & driver_station::kFmsAttached) != 0 ? 1.0 : 0.0);
          }
          if (topic.ds_channels.alliance < dictionary_.size()) {
            SetValue(topic.ds_channels.alliance,
                     static_cast<double>(
                         static_cast<std::uint8_t>(ds_data.fms.alliance)));
          }
          if (topic.ds_channels.match_time_s < dictionary_.size()) {
            SetValue(topic.ds_channels.match_time_s,
                     static_cast<double>(ds_data.fms.match_time_s));
          }
        }
      } else {
        for (const auto& reader : topic.readers) {
          const double val =
              detail::ReadScalarAsDouble(reader.base_type, raw + reader.offset);
          SetValue(reader.channel_index, val);
        }
      }
    }

    flatbuffers::FlatBufferBuilder& builder = builder_;
    builder.Clear();
    const auto values_offset =
        builder.CreateVector(values_.data(), values_.size());
    flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<Talos::Telemetry::ChannelInfo>>>
        dictionary_offset = 0;

    // First frame and every 100th frame; never otherwise.
    if (sequence_ % 100 == 0) {
      std::vector<flatbuffers::Offset<Talos::Telemetry::ChannelInfo>> entries;
      entries.reserve(dictionary_.size());
      for (const ChannelDef& channel : dictionary_) {
        std::vector<flatbuffers::Offset<flatbuffers::String>> labels;
        labels.reserve(channel.labels.size());
        for (const std::string& label : channel.labels) {
          labels.push_back(builder.CreateString(label));
        }
        entries.push_back(Talos::Telemetry::CreateChannelInfo(
            builder, builder.CreateString(channel.name),
            channel.unit.empty() ? 0 : builder.CreateString(channel.unit),
            channel.labels.empty() ? 0 : builder.CreateVector(labels)));
      }
      dictionary_offset = builder.CreateVector(entries);
    }

    Talos::Telemetry::FinishSizePrefixedFrameBuffer(
        builder,
        Talos::Telemetry::CreateFrame(
            builder, stream_id_, sequence_,
            static_cast<std::uint64_t>(node_.now().nanos()), dictionary_id_,
            &chassis, values_offset, dictionary_offset));

    studio_.send(std::as_bytes(
        std::span{builder.GetBufferPointer(), builder.GetSize()}));
    ++sequence_;
  }

 private:
  struct FieldReader {
    std::size_t channel_index;
    std::size_t offset;
    reflection::BaseType base_type;
  };

  struct DriverStationChannels {
    std::size_t connected{std::numeric_limits<std::size_t>::max()};
    std::size_t enabled{std::numeric_limits<std::size_t>::max()};
    std::size_t autonomous{std::numeric_limits<std::size_t>::max()};
    std::size_t teleop{std::numeric_limits<std::size_t>::max()};
    std::size_t test{std::numeric_limits<std::size_t>::max()};
    std::size_t estop{std::numeric_limits<std::size_t>::max()};
    std::size_t fms_attached{std::numeric_limits<std::size_t>::max()};
    std::size_t alliance{std::numeric_limits<std::size_t>::max()};
    std::size_t match_time_s{std::numeric_limits<std::size_t>::max()};
  };

  struct ObservedTopic {
    std::string name;
    std::size_t sub_index;
    std::vector<FieldReader> readers;
    bool is_driver_station = false;
    DriverStationChannels ds_channels;
  };

  static std::string ResolveTopic(Node& node) {
    using talos::detail::NodeAccess;
    return NodeAccess::robot_config(node).studio.topic;
  }

  static std::string ResolvePoseTopic(Node& node) {
    using talos::detail::NodeAccess;
    return NodeAccess::robot_config(node).studio.pose_topic;
  }

  static std::uint64_t RandomStreamId() {
    std::random_device device;
    const std::uint64_t id =
        (static_cast<std::uint64_t>(device()) << 32) | device();
    return id != 0 ? id : 1;
  }

  void SetValue(std::size_t index, double value) {
    if (index < values_.size() && std::isfinite(value)) {
      values_[index] = value;
    }
  }

  std::size_t AddChannel(std::string name, std::string unit = "",
                         std::vector<std::string> labels = {}) {
    if (detail::MatchesAnyGlob(name, exclude_globs_)) {
      return std::numeric_limits<std::size_t>::max();
    }
    for (const ChannelDef& existing : dictionary_) {
      if (existing.name == name) {
        throw std::invalid_argument("telemetry: duplicate channel '" + name +
                                    "'");
      }
    }
    dictionary_.push_back(
        ChannelDef{std::move(name), std::move(unit), std::move(labels)});
    return dictionary_.size() - 1;
  }

  void CheckFrameBudget() {
    const std::size_t worst_case = MeasureValueFrameBytes(dictionary_.size());
    if (worst_case > telemetry_wire::kMaxValueFrameBytes) {
      std::map<std::string, std::size_t> counts;
      for (const auto& topic : observed_topics_) {
        if (topic.is_driver_station) {
          const auto& channels = topic.ds_channels;
          for (const auto index :
               {channels.connected, channels.enabled, channels.autonomous,
                channels.teleop, channels.test, channels.estop,
                channels.fms_attached, channels.alliance,
                channels.match_time_s}) {
            if (index < dictionary_.size()) ++counts[topic.name];
          }
        } else {
          counts[topic.name] = topic.readers.size();
        }
      }
      std::vector<std::pair<std::string, std::size_t>> sorted_counts(
          counts.begin(), counts.end());
      std::sort(
          sorted_counts.begin(), sorted_counts.end(),
          [](const auto& a, const auto& b) { return a.second > b.second; });
      std::string err =
          "telemetry: worst-case value frame is " + std::to_string(worst_case) +
          " bytes, over the " +
          std::to_string(telemetry_wire::kMaxValueFrameBytes) +
          "-byte budget (see talOS/telemetry/README.md). Largest sources: ";
      for (std::size_t i = 0; i < sorted_counts.size(); ++i) {
        if (i > 0) err += ", ";
        err += sorted_counts[i].first + " (" +
               std::to_string(sorted_counts[i].second) + " channels)";
      }
      throw std::runtime_error(err);
    }
  }

  void BuildFromGraph(const introspect::DeclaredGraph& graph) {
    using talos::detail::NodeAccess;

    // Check if pose_topic is defined in graph
    if (!pose_topic_.empty()) {
      bool found_pose_topic = false;
      for (const auto& node_desc : graph) {
        for (const auto& source : node_desc.sources) {
          if (source.name == pose_topic_ &&
              source.kind == event::SourceKind::SENDER) {
            found_pose_topic = true;
            SetupPoseExtraction(source);
            break;
          }
        }
        if (has_pose_) break;
      }
      if (!found_pose_topic) {
        throw std::invalid_argument("telemetry: pose topic '" + pose_topic_ +
                                    "' is absent from graph");
      }
      if (!has_pose_) {
        throw std::invalid_argument("telemetry: pose topic '" + pose_topic_ +
                                    "' has no geometry.Pose2d field");
      }
    }

    std::set<std::string> seen_topics;
    for (const auto& node_desc : graph) {
      for (const auto& source : node_desc.sources) {
        if (source.kind != event::SourceKind::SENDER) continue;
        if (seen_topics.contains(source.name)) continue;

        const auto segments = introspect::naming::detail::Segments(source.name);
        if (segments.size() < 2) continue;
        const std::string_view role = segments[1];
        if (role != "state" && role != "status") continue;

        if (!detail::MatchesGlobs(source.name, include_globs_,
                                  exclude_globs_)) {
          continue;
        }

        seen_topics.insert(source.name);
        // Schema-less state/status packets (notably /hw/state) contribute
        // neither subscriptions nor dictionary channels.

        if (source.name == "/hw/state/driver_station") {
          ObserveDriverStation(source);
        } else if (!source.bfbs.empty()) {
          ObserveDescribedTopic(source);
        }
      }
    }
  }

  void SetupPoseExtraction(const introspect::naming::SourceShape& source) {
    using talos::detail::NodeAccess;

    if (source.bfbs.empty()) {
      throw std::invalid_argument("telemetry: pose topic '" + source.name +
                                  "' has no schema");
    }

    flatbuffers::Verifier verifier{
        reinterpret_cast<const std::uint8_t*>(source.bfbs.data()),
        source.bfbs.size()};
    if (!reflection::VerifySchemaBuffer(verifier)) {
      throw std::invalid_argument(
          "telemetry: failed to verify schema for pose topic '" + source.name +
          "'");
    }

    const reflection::Schema* schema =
        reflection::GetSchema(source.bfbs.data());
    const reflection::Object* object =
        schema->objects()->LookupByKey(source.schema.c_str());
    if (!object) {
      throw std::invalid_argument("telemetry: schema object '" + source.schema +
                                  "' for pose topic '" + source.name +
                                  "' is missing");
    }

    bool found_pose = false;
    for (const auto* field : *object->fields()) {
      const auto& type = *field->type();
      if (type.base_type() == reflection::BaseType::Obj) {
        const auto* sub_obj = schema->objects()->Get(type.index());
        if (sub_obj != nullptr &&
            sub_obj->name()->string_view() == "geometry.Pose2d") {
          const std::size_t pose_offset = field->offset();
          bool found_x = false;
          bool found_y = false;
          bool found_yaw = false;
          for (const auto* sub_field : *sub_obj->fields()) {
            if (sub_field->name()->string_view() == "x_m") {
              pose_x_offset_ = pose_offset + sub_field->offset();
              found_x = true;
            } else if (sub_field->name()->string_view() == "y_m") {
              pose_y_offset_ = pose_offset + sub_field->offset();
              found_y = true;
            } else if (sub_field->name()->string_view() == "yaw_rad") {
              pose_yaw_offset_ = pose_offset + sub_field->offset();
              found_yaw = true;
            }
          }
          if (found_x && found_y && found_yaw) {
            found_pose = true;
            break;
          }
        }
      }
    }

    if (!found_pose) {
      throw std::invalid_argument("telemetry: pose topic '" + source.name +
                                  "' has no geometry.Pose2d field");
    }

    has_pose_ = true;
    for (const auto* field : *object->fields()) {
      if (field->name()->string_view() == "valid" &&
          field->type()->base_type() == reflection::BaseType::Bool) {
        pose_has_valid_field_ = true;
        pose_valid_offset_ = field->offset();
        break;
      }
    }
    pose_sub_index_ = NodeAccess::add_observer(
        node_, source.name, source.message_bytes, source.alignment,
        {source.schema, source.schema_hash, source.bfbs});
  }

  void ObserveDriverStation(const introspect::naming::SourceShape& source) {
    using talos::detail::NodeAccess;

    const std::size_t sub_idx = NodeAccess::add_observer(
        node_, source.name, source.message_bytes, source.alignment,
        {source.schema, source.schema_hash, source.bfbs});

    ObservedTopic topic;
    topic.name = source.name;
    topic.sub_index = sub_idx;
    topic.is_driver_station = true;

    topic.ds_channels.connected =
        AddChannel("hw/state/driver_station/connected");
    topic.ds_channels.enabled = AddChannel("hw/state/driver_station/enabled");
    topic.ds_channels.autonomous =
        AddChannel("hw/state/driver_station/autonomous");
    topic.ds_channels.teleop = AddChannel("hw/state/driver_station/teleop");
    topic.ds_channels.test = AddChannel("hw/state/driver_station/test");
    topic.ds_channels.estop = AddChannel("hw/state/driver_station/estop");
    topic.ds_channels.fms_attached =
        AddChannel("hw/state/driver_station/fms_attached");
    topic.ds_channels.alliance = AddChannel("hw/state/driver_station/alliance",
                                            "", {"unknown", "red", "blue"});
    topic.ds_channels.match_time_s =
        AddChannel("hw/state/driver_station/match_time_s", "s");

    observed_topics_.push_back(std::move(topic));
  }

  void ObserveDescribedTopic(const introspect::naming::SourceShape& source) {
    using talos::detail::NodeAccess;

    flatbuffers::Verifier verifier{
        reinterpret_cast<const std::uint8_t*>(source.bfbs.data()),
        source.bfbs.size()};
    if (!reflection::VerifySchemaBuffer(verifier)) {
      throw std::invalid_argument(
          "telemetry: failed to verify schema for topic '" + source.name + "'");
    }

    const reflection::Schema* schema =
        reflection::GetSchema(source.bfbs.data());
    const reflection::Object* object =
        schema->objects()->LookupByKey(source.schema.c_str());
    if (!object) {
      throw std::invalid_argument("telemetry: root schema object '" +
                                  source.schema + "' for topic '" +
                                  source.name + "' is missing");
    }

    std::size_t sub_idx = 0;
    if (has_pose_ && source.name == pose_topic_) {
      sub_idx = pose_sub_index_;
    } else {
      sub_idx = NodeAccess::add_observer(
          node_, source.name, source.message_bytes, source.alignment,
          {source.schema, source.schema_hash, source.bfbs});
    }

    ObservedTopic topic;
    topic.name = source.name;
    topic.sub_index = sub_idx;

    const std::string prefix =
        (!source.name.empty() && source.name.front() == '/')
            ? source.name.substr(1)
            : source.name;

    FlattenStruct(*schema, *object, prefix, 0, topic.readers);

    observed_topics_.push_back(std::move(topic));
  }

  void FlattenStruct(const reflection::Schema& schema,
                     const reflection::Object& object, std::string_view prefix,
                     std::size_t base_offset,
                     std::vector<FieldReader>& readers) {
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
        const auto* nested_obj = schema.objects()->Get(type.index());
        if (nested_obj != nullptr) {
          FlattenStruct(schema, *nested_obj, channel_name, field_offset,
                        readers);
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
          const std::string elem_prefix =
              channel_name + "/" + std::to_string(i);
          const std::size_t elem_offset = field_offset + i * element_bytes;

          if (type.element() == reflection::BaseType::Obj) {
            const auto* nested_obj = schema.objects()->Get(type.index());
            if (nested_obj != nullptr) {
              FlattenStruct(schema, *nested_obj, elem_prefix, elem_offset,
                            readers);
            }
          } else {
            AddScalarField(schema, type.element(), type.index(), field_name,
                           elem_prefix, elem_offset, readers);
          }
        }
        continue;
      }

      AddScalarField(schema, type.base_type(), type.index(), field_name,
                     channel_name, field_offset, readers);
    }
  }

  void AddScalarField(const reflection::Schema& schema,
                      reflection::BaseType base, std::int32_t type_index,
                      std::string_view field_name, std::string channel_name,
                      std::size_t offset, std::vector<FieldReader>& readers) {
    std::string unit;
    std::vector<std::string> labels;

    if (base == reflection::BaseType::Bool) {
      // Bool: unit is empty
    } else if (type_index >= 0) {
      // Enum: extract labels from reflection
      labels = detail::ExtractEnumLabels(schema, type_index);
    } else {
      unit = std::string{schema::unit_for_field(field_name)};
    }

    const std::size_t channel_idx =
        AddChannel(std::move(channel_name), std::move(unit), std::move(labels));
    if (channel_idx == std::numeric_limits<std::size_t>::max()) return;
    readers.push_back(FieldReader{
        .channel_index = channel_idx,
        .offset = offset,
        .base_type = base,
    });
  }

  Node& node_;
  Duration stale_after_;
  std::vector<std::string> include_globs_;
  std::vector<std::string> exclude_globs_;
  std::string topic_;
  std::string pose_topic_;
  BytePublisher studio_;

  std::vector<ChannelDef> dictionary_;
  std::uint32_t dictionary_id_{0};
  std::uint64_t stream_id_{0};
  std::vector<double> values_;
  flatbuffers::FlatBufferBuilder builder_{1024};
  std::uint64_t sequence_ = 0;

  std::size_t idx_pose_valid_{0};
  bool has_pose_{false};
  std::size_t pose_sub_index_{0};
  std::size_t pose_x_offset_{0};
  std::size_t pose_y_offset_{0};
  std::size_t pose_yaw_offset_{0};
  bool pose_has_valid_field_{false};
  std::size_t pose_valid_offset_{0};

  std::vector<ObservedTopic> observed_topics_;
};

}  // namespace talos::telemetry
