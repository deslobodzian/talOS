#pragma once

#include <fnmatch.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
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
#include "talOS/telemetry/frames.h"
#include "talOS/telemetry/telemetry_generated.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {

namespace detail {

template <class T>
inline T Read(const std::uint8_t* data) {
  T value;
  std::memcpy(&value, data, sizeof value);
  return value;
}

// The channel kind a reflected scalar travels as: integers keep their
// width's exact value, floats become doubles.
inline ChannelKind KindOf(reflection::BaseType base) {
  using reflection::BaseType;
  switch (base) {
    case BaseType::Bool:
      return Talos::Telemetry::ChannelKind_Bool;
    case BaseType::Byte:
    case BaseType::Short:
    case BaseType::Int:
    case BaseType::Long:
      return Talos::Telemetry::ChannelKind_Int;
    case BaseType::UByte:
    case BaseType::UType:
    case BaseType::UShort:
    case BaseType::UInt:
    case BaseType::ULong:
      return Talos::Telemetry::ChannelKind_UInt;
    default:
      return Talos::Telemetry::ChannelKind_Double;
  }
}

// The raw sample of one reflected scalar, or false for a type telemetry does
// not carry. A NaN or infinite float travels as itself: a reading, not an
// absence.
inline bool ReadScalarRaw(reflection::BaseType base, const std::uint8_t* data,
                          std::uint64_t& raw) {
  using reflection::BaseType;
  const auto signed_raw = [](std::int64_t value) {
    return static_cast<std::uint64_t>(value);
  };
  switch (base) {
    case BaseType::Bool:
      raw = Read<std::uint8_t>(data) != 0 ? 1 : 0;
      return true;
    case BaseType::UByte:
    case BaseType::UType:
      raw = Read<std::uint8_t>(data);
      return true;
    case BaseType::Byte:
      raw = signed_raw(Read<std::int8_t>(data));
      return true;
    case BaseType::Short:
      raw = signed_raw(Read<std::int16_t>(data));
      return true;
    case BaseType::UShort:
      raw = Read<std::uint16_t>(data);
      return true;
    case BaseType::Int:
      raw = signed_raw(Read<std::int32_t>(data));
      return true;
    case BaseType::UInt:
      raw = Read<std::uint32_t>(data);
      return true;
    case BaseType::Long:
      raw = signed_raw(Read<std::int64_t>(data));
      return true;
    case BaseType::ULong:
      raw = Read<std::uint64_t>(data);
      return true;
    case BaseType::Float:
      raw = RawFromDouble(Read<float>(data));
      return true;
    case BaseType::Double:
      raw = RawFromDouble(Read<double>(data));
      return true;
    default:
      return false;
  }
}

// Labels index by value, so an enum whose largest value is negative or not
// below kMaxEnumLabels (wide bit flags, sparse codes) carries none.
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
    if (max_val >= 0 &&
        max_val < static_cast<std::int64_t>(telemetry_wire::kMaxEnumLabels)) {
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

// How the telemetry node opens its byte topic. The stream is for viewers,
// not a record (each node's event log is), so it overwrites: a stalled or
// stopped reader is lapped and counts the frames it missed, and never makes
// the writer refuse frames for every other reader.
inline constexpr BytePublishOptions kTopicOptions{
    .slots = telemetry_wire::slot_count,
    .drop_newest = false,
    .external = true};

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
                                   kTopicOptions)} {
    using talos::detail::NodeAccess;

    idx_pose_valid_ =
        AddChannel("pose/valid", Talos::Telemetry::ChannelKind_Bool);

    const introspect::DeclaredGraph* graph = NodeAccess::declared_graph(node);
    if (graph != nullptr) {
      BuildFromGraph(*graph);
    }

    CheckChannelCap();
    dictionary_id_ = DictionaryId(dictionary_);
    stream_id_ = RandomStreamId();
    // update() fills these without reallocating.
    tick_channels_.reserve(dictionary_.size());
    tick_values_.reserve(dictionary_.size());

    ReportPages(graph != nullptr);
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

    if (!any_source_fresh && !last_frame_had_fresh_) return;
    last_frame_had_fresh_ = any_source_fresh;

    tick_channels_.clear();
    tick_values_.clear();

    SetBool(idx_pose_valid_, pose_ok);

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
          const auto& channels = topic.ds_channels;
          SetBool(channels.connected,
                  (flags & driver_station::kDsAttached) != 0);
          SetBool(channels.enabled, (flags & driver_station::kEnabled) != 0);
          SetBool(channels.autonomous,
                  (flags & driver_station::kAutonomous) != 0);
          SetBool(channels.teleop, (flags & driver_station::kTeleop) != 0);
          SetBool(channels.test, (flags & driver_station::kTest) != 0);
          SetBool(channels.estop, (flags & driver_station::kEStop) != 0);
          SetBool(channels.fms_attached,
                  (flags & driver_station::kFmsAttached) != 0);
          SetRaw(channels.alliance,
                 static_cast<std::uint8_t>(ds_data.fms.alliance));
          SetRaw(channels.match_time_s,
                 RawFromDouble(ds_data.fms.match_time_s));
        }
      } else {
        for (const auto& reader : topic.readers) {
          std::uint64_t value = 0;
          if (detail::ReadScalarRaw(reader.base_type, raw + reader.offset,
                                    value)) {
            SetRaw(reader.channel_index, value);
          }
        }
      }
    }

    const PageHeader header{
        .stream_id = stream_id_,
        .sequence_id = sequence_,
        .timestamp_ns = static_cast<std::uint64_t>(node_.now().nanos()),
        .dictionary_id = dictionary_id_,
        .channel_count = static_cast<std::uint32_t>(dictionary_.size())};
    const auto send = [this](std::span<const std::uint8_t> page) {
      studio_.send(std::as_bytes(page));
    };
    // The dictionary on the first tick and every 100th, before its values.
    if (sequence_ % 100 == 0) {
      EncodeDictionaryPages(builder_, header, dictionary_, send);
    }
    // A producer tick is a keyframe: a channel it does not carry is absent.
    // It cannot fail: every page is bounded (frames_test) and the dictionary
    // is within kMaxChannels (construction).
    const bool encoded = EncodeValuePages(
        builder_, header, chassis, true, tick_channels_, tick_values_, {},
        [&send](std::span<const std::uint8_t> page, const PageSpan&) {
          send(page);
        });
    static_cast<void>(encoded);
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

  // Carries channel `index` this tick; an excluded channel (index past the
  // dictionary) carries nothing.
  void SetRaw(std::size_t index, std::uint64_t raw) {
    if (index >= dictionary_.size()) return;
    tick_channels_.push_back(static_cast<std::uint32_t>(index));
    tick_values_.push_back(raw);
  }
  void SetBool(std::size_t index, bool value) { SetRaw(index, value ? 1 : 0); }

  std::size_t AddChannel(std::string name, ChannelKind kind,
                         std::string unit = "",
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
    // Studio refuses a whole dictionary over one channel it cannot decode,
    // blanking the stream; refuse that channel here instead, by name, so it
    // can be excluded. ExtractEnumLabels keeps label counts within the limit.
    if (name.starts_with("derived/")) {
      throw std::invalid_argument(
          "telemetry: channel '" + name +
          "' is in Studio's reserved derived/ namespace (see "
          "talOS/telemetry/README.md)");
    }
    const auto check_length = [&name](std::string_view what,
                                      std::string_view text) {
      if (text.size() > telemetry_wire::kMaxDictionaryStringBytes) {
        throw std::invalid_argument(
            "telemetry: channel '" + name + "' " + std::string{what} + " is " +
            std::to_string(text.size()) + " bytes, over the " +
            std::to_string(telemetry_wire::kMaxDictionaryStringBytes) +
            "-byte dictionary string limit (see talOS/telemetry/README.md)");
      }
    };
    check_length("name", name);
    check_length("unit", unit);
    for (const std::string& label : labels) check_length("label", label);
    dictionary_.push_back(ChannelDef{std::move(name), std::move(unit),
                                     std::move(labels), kind});
    return dictionary_.size() - 1;
  }

  // Studio's bridge sends a client an update only whole, so the largest one,
  // a keyframe of every channel, must fit its burst: refuse a dictionary past
  // kMaxChannels, naming the topics with the most channels to exclude.
  void CheckChannelCap() const {
    if (dictionary_.size() <= telemetry_wire::kMaxChannels) return;
    std::map<std::string, std::size_t> per_topic;
    for (const ChannelDef& channel : dictionary_) {
      const std::size_t owner = channel.name.find('/');
      const std::size_t role = owner == std::string::npos
                                   ? std::string::npos
                                   : channel.name.find('/', owner + 1);
      ++per_topic[channel.name.substr(0, role)];
    }
    std::vector<std::pair<std::size_t, std::string>> largest;
    for (const auto& [topic, count] : per_topic) {
      largest.emplace_back(count, topic);
    }
    std::sort(largest.begin(), largest.end(), std::greater<>{});
    std::string names;
    for (std::size_t i = 0; i < largest.size() && i < 3; ++i) {
      if (i != 0) names += ", ";
      names += largest[i].second + " (" + std::to_string(largest[i].first) +
               ")";
    }
    throw std::runtime_error(
        "telemetry: " + std::to_string(dictionary_.size()) +
        " channels, over the " + std::to_string(telemetry_wire::kMaxChannels) +
        "-channel cap; largest sources: " + names +
        "; exclude channels in telemetry.toml (see "
        "talOS/telemetry/README.md)");
  }

  // Pages the dictionary once, which throws, naming the channel, when one
  // channel's entry cannot fit a dictionary page alone. For a graph-built
  // dictionary (the launcher's --describe-only and every launch) it reports
  // the pages a tick takes, so a config author sees the cost of a channel.
  void ReportPages(bool report) const {
    const PageBudget budget = MeasurePages(dictionary_);
    if (!report) return;
    std::cerr << "telemetry: " << dictionary_.size() << " channels; "
              << budget.value_pages << " value page(s) per tick, "
              << budget.value_bytes << " B worst case (largest "
              << budget.largest_value_page << " of "
              << telemetry_wire::kMaxValueFrameBytes << " B); dictionary "
              << budget.dictionary_pages << " page(s), "
              << budget.dictionary_bytes << " B\n";
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

    constexpr auto kBool = Talos::Telemetry::ChannelKind_Bool;
    auto& channels = topic.ds_channels;
    channels.connected = AddChannel("hw/state/driver_station/connected", kBool);
    channels.enabled = AddChannel("hw/state/driver_station/enabled", kBool);
    channels.autonomous =
        AddChannel("hw/state/driver_station/autonomous", kBool);
    channels.teleop = AddChannel("hw/state/driver_station/teleop", kBool);
    channels.test = AddChannel("hw/state/driver_station/test", kBool);
    channels.estop = AddChannel("hw/state/driver_station/estop", kBool);
    channels.fms_attached =
        AddChannel("hw/state/driver_station/fms_attached", kBool);
    channels.alliance =
        AddChannel("hw/state/driver_station/alliance",
                   Talos::Telemetry::ChannelKind_UInt, "",
                   {"unknown", "red", "blue"});
    channels.match_time_s =
        AddChannel("hw/state/driver_station/match_time_s",
                   Talos::Telemetry::ChannelKind_Double, "s");

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
        AddChannel(std::move(channel_name), detail::KindOf(base),
                   std::move(unit), std::move(labels));
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
  // This tick's carried channels and their raw samples, index-aligned.
  std::vector<std::uint32_t> tick_channels_;
  std::vector<std::uint64_t> tick_values_;
  flatbuffers::FlatBufferBuilder builder_{2048};
  std::uint64_t sequence_ = 0;
  bool last_frame_had_fresh_{false};

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
